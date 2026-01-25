#!/usr/bin/env python3
"""
Oak-D Lite Feature Tracker Test Script (with IMU compensation)

Run this on your Mac to see exactly what data would be sent to PX4.
Now includes gyroscope compensation to distinguish rotation from translation.

Install:
    pip install depthai numpy

Usage:
    python test_feature_tracker.py
"""

import depthai as dai
import numpy as np
import time
import warnings
from collections import deque

warnings.filterwarnings("ignore", category=DeprecationWarning)


class GyroBuffer:
    """Buffers gyro data and provides interpolated values."""

    def __init__(self, max_age=0.1):
        self.buffer = deque(maxlen=100)
        self.max_age = max_age

    def add(self, timestamp, gyro):
        """Add gyro reading to buffer."""
        self.buffer.append((timestamp, gyro))

    def get_average(self, start_time, end_time):
        """Get average gyro between two timestamps."""
        if not self.buffer:
            return np.zeros(3)

        samples = []
        for ts, gyro in self.buffer:
            if start_time <= ts <= end_time:
                samples.append([gyro.x, gyro.y, gyro.z])

        if not samples:
            # Use most recent if no samples in range
            _, gyro = self.buffer[-1]
            return np.array([gyro.x, gyro.y, gyro.z])

        return np.mean(samples, axis=0)


class FeatureFlowCalculator:
    """Converts tracked features to velocity estimates with IMU compensation."""

    def __init__(self, fx=400.0, fy=400.0, cx=320.0, cy=200.0, min_features=10):
        self.fx = fx  # focal length x (pixels)
        self.fy = fy  # focal length y (pixels)
        self.cx = cx  # principal point x
        self.cy = cy  # principal point y
        self.min_features = min_features

        self.prev_features = {}
        self.prev_time = None

        self.gyro_buffer = GyroBuffer()

    def add_gyro(self, timestamp, gyro):
        """Add gyro reading for rotation compensation."""
        self.gyro_buffer.add(timestamp, gyro)

    def update(self, features, timestamp):
        """
        Process features and return velocity estimate.
        Returns: (vx, vy, quality, matched, raw_vx, raw_vy) or None
        """
        # Build current feature dict with positions
        current = {}
        for f in features:
            current[f.id] = (f.position.x, f.position.y)

        if not self.prev_features or self.prev_time is None:
            self.prev_features = current
            self.prev_time = timestamp
            return None

        dt = timestamp - self.prev_time
        if dt <= 0 or dt > 0.5:
            self.prev_features = current
            self.prev_time = timestamp
            return None

        # Get average gyro during this interval
        gyro = self.gyro_buffer.get_average(self.prev_time, timestamp)

        # Match features and compute flow
        flow_vectors = []
        compensated_flow = []

        for fid, (x, y) in current.items():
            if fid in self.prev_features:
                px, py = self.prev_features[fid]
                dx = x - px  # raw pixel flow
                dy = y - py

                # Compute expected pixel motion from rotation
                # For a pinhole camera:
                # dx_rot = -fy * ωx * dt + x' * ωz * dt + fx * x' * y' * ωy * dt / fy  (simplified)
                # For small angles near center, approximate:
                # Pan (ωy): dx ≈ -fx * ωy * dt
                # Tilt (ωx): dy ≈ fy * ωx * dt
                # Roll (ωz): circular motion

                # Rotation around Y axis (yaw/pan) causes horizontal flow
                # Rotation around X axis (pitch/tilt) causes vertical flow
                # Note: camera frame may differ - adjust signs as needed

                # Expected flow from rotation (camera convention: x-right, y-down, z-forward)
                # ωx = pitch (rotation around camera x) -> vertical image motion
                # ωy = yaw (rotation around camera y) -> horizontal image motion
                # ωz = roll (rotation around camera z) -> rotational image motion

                dx_rot = self.fx * gyro[1] * dt   # yaw causes horizontal flow
                dy_rot = -self.fy * gyro[0] * dt  # pitch causes vertical flow

                # Compensated flow (translation only)
                dx_trans = dx - dx_rot
                dy_trans = dy - dy_rot

                flow_vectors.append((dx, dy))
                compensated_flow.append((dx_trans, dy_trans))

        self.prev_features = current
        self.prev_time = timestamp

        if len(flow_vectors) < self.min_features:
            return None

        # Raw flow (without compensation)
        raw_flow = np.array(flow_vectors)
        raw_median_dx = np.median(raw_flow[:, 0])
        raw_median_dy = np.median(raw_flow[:, 1])
        raw_vx = -(raw_median_dx / self.fx) / dt
        raw_vy = -(raw_median_dy / self.fy) / dt

        # Compensated flow (rotation removed)
        comp_flow = np.array(compensated_flow)
        median_dx = np.median(comp_flow[:, 0])
        median_dy = np.median(comp_flow[:, 1])

        # Convert to velocity
        vx = -(median_dx / self.fx) / dt
        vy = -(median_dy / self.fy) / dt

        # Quality based on consistency of compensated flow
        std_dx = np.std(comp_flow[:, 0])
        std_dy = np.std(comp_flow[:, 1])
        quality = 1.0 / (1.0 + std_dx + std_dy)

        return (vx, vy, quality, len(flow_vectors), raw_vx, raw_vy, gyro)


def format_px4_message(vx, vy, quality):
    """Format what would be sent to PX4."""
    return {
        'timestamp': int(time.time() * 1e6),
        'velocity': [vx, -vy, 0.0],
        'velocity_variance': [
            0.1 / (quality + 0.01),
            0.1 / (quality + 0.01),
            999.0
        ],
        'velocity_frame': 'BODY_FRD',
    }


def main():
    print("=" * 70)
    print("Oak-D Lite Feature Tracker Test (with IMU Rotation Compensation)")
    print("=" * 70)
    print()

    pipeline = dai.Pipeline()

    # Mono camera
    mono = pipeline.create(dai.node.MonoCamera)
    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono.setCamera("left")
    mono.setFps(30)

    # Feature tracker
    feature_tracker = pipeline.create(dai.node.FeatureTracker)
    feature_tracker.setHardwareResources(2, 2)

    # IMU
    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 200)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    # Outputs
    xout_features = pipeline.create(dai.node.XLinkOut)
    xout_features.setStreamName("features")
    xout_imu = pipeline.create(dai.node.XLinkOut)
    xout_imu.setStreamName("imu")

    # Links
    mono.out.link(feature_tracker.inputImage)
    feature_tracker.outputFeatures.link(xout_features.input)
    imu.out.link(xout_imu.input)

    print("Connecting to Oak-D Lite...")

    with dai.Device(pipeline) as device:
        print(f"Device: {device.getDeviceName()}")
        print(f"Product: {device.getProductName()}")
        print(f"USB Speed: {device.getUsbSpeed().name}")
        print(f"IMU: {device.getConnectedIMU()}")
        print()

        q_features = device.getOutputQueue("features", maxSize=4, blocking=False)
        q_imu = device.getOutputQueue("imu", maxSize=50, blocking=False)

        # Camera intrinsics for OAK-D Lite at 400P (approximate)
        # You can get exact values from calibration
        flow_calc = FeatureFlowCalculator(
            fx=400.0,   # focal length in pixels
            fy=400.0,
            cx=320.0,   # principal point (image center)
            cy=200.0,
            min_features=10
        )

        print("=" * 70)
        print("Streaming... (Ctrl+C to stop)")
        print("=" * 70)
        print()
        print("Try these tests:")
        print("  1. Keep camera still      → velocities should be ~0")
        print("  2. Pan left/right         → RAW shows motion, COMPENSATED should be ~0")
        print("  3. Move camera sideways   → COMPENSATED shows actual translation")
        print()

        frame_count = 0
        start_time = time.time()
        last_print = 0

        try:
            while True:
                # Process all available IMU data first
                while True:
                    imu_data = q_imu.tryGet()
                    if imu_data is None:
                        break
                    for packet in imu_data.packets:
                        gyro = packet.gyroscope
                        ts = packet.gyroscope.getTimestamp().total_seconds()
                        flow_calc.add_gyro(ts, gyro)

                # Get tracked features
                features_data = q_features.tryGet()

                if features_data is not None:
                    features = features_data.trackedFeatures
                    timestamp = features_data.getTimestamp().total_seconds()

                    result = flow_calc.update(features, timestamp)
                    frame_count += 1
                    now = time.time()

                    if now - last_print > 0.5:
                        last_print = now
                        fps = frame_count / (now - start_time) if (now - start_time) > 0 else 0

                        print("-" * 70)
                        print(f"Frame: {frame_count} | FPS: {fps:.1f} | Features: {len(features)}")

                        if result:
                            vx, vy, quality, matched, raw_vx, raw_vy, gyro = result
                            print(f"Matched: {matched} | Quality: {quality:.2f}")
                            print()
                            print(f"Gyro (rad/s):  [{gyro[0]:+.3f}, {gyro[1]:+.3f}, {gyro[2]:+.3f}]")
                            print()
                            print("                    RAW (no compensation)    COMPENSATED (rotation removed)")
                            print(f"  vx (forward):     {raw_vx:+.3f} m/s              {vx:+.3f} m/s")
                            print(f"  vy (left):        {raw_vy:+.3f} m/s              {vy:+.3f} m/s")
                            print()

                            px4_msg = format_px4_message(vx, vy, quality)
                            print("PX4 VehicleVisualOdometry (compensated):")
                            print(f"  velocity: [{px4_msg['velocity'][0]:+.3f}, {px4_msg['velocity'][1]:+.3f}, {px4_msg['velocity'][2]:+.3f}]")
                            print(f"  variance: [{px4_msg['velocity_variance'][0]:.3f}, {px4_msg['velocity_variance'][1]:.3f}, {px4_msg['velocity_variance'][2]:.1f}]")
                        else:
                            if len(features) < 10:
                                print("(not enough features - point at textured surface)")
                            else:
                                print("(waiting for motion...)")

                time.sleep(0.001)

        except KeyboardInterrupt:
            print()
            print("=" * 70)
            print("Stopped.")
            elapsed = time.time() - start_time
            if elapsed > 0 and frame_count > 0:
                print(f"Processed {frame_count} frames in {elapsed:.1f}s ({frame_count/elapsed:.1f} FPS)")


if __name__ == "__main__":
    main()
