#!/usr/bin/env python3
"""
Oak-D Lite Feature Tracker Test Script for DepthAI 3.x

Run this on the Raspberry Pi with the OAK-D Lite connected.

Usage:
    source ~/oak_venv/bin/activate
    python test_feature_tracker_v3.py
"""

import depthai as dai
import numpy as np
import time
from collections import deque


class GyroBuffer:
    """Buffers gyro data and provides interpolated values."""

    def __init__(self, max_age=0.1):
        self.buffer = deque(maxlen=100)
        self.max_age = max_age

    def add(self, timestamp, gyro_xyz):
        """Add gyro reading to buffer."""
        self.buffer.append((timestamp, gyro_xyz))

    def get_average(self, start_time, end_time):
        """Get average gyro between two timestamps."""
        if not self.buffer:
            return np.zeros(3)

        samples = []
        for ts, gyro in self.buffer:
            if start_time <= ts <= end_time:
                samples.append(gyro)

        if not samples:
            _, gyro = self.buffer[-1]
            return np.array(gyro)

        return np.mean(samples, axis=0)


class FeatureFlowCalculator:
    """Converts tracked features to velocity estimates with IMU compensation."""

    def __init__(self, fx=400.0, fy=400.0, cx=320.0, cy=200.0, min_features=10):
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy
        self.min_features = min_features

        self.prev_features = {}
        self.prev_time = None
        self.gyro_buffer = GyroBuffer()

    def add_gyro(self, timestamp, gyro_xyz):
        """Add gyro reading for rotation compensation."""
        self.gyro_buffer.add(timestamp, gyro_xyz)

    def update(self, features, timestamp):
        """Process features and return velocity estimate."""
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

        gyro = self.gyro_buffer.get_average(self.prev_time, timestamp)

        flow_vectors = []
        compensated_flow = []

        for fid, (x, y) in current.items():
            if fid in self.prev_features:
                px, py = self.prev_features[fid]
                dx = x - px
                dy = y - py

                dx_rot = self.fx * gyro[1] * dt
                dy_rot = -self.fy * gyro[0] * dt

                dx_trans = dx - dx_rot
                dy_trans = dy - dy_rot

                flow_vectors.append((dx, dy))
                compensated_flow.append((dx_trans, dy_trans))

        self.prev_features = current
        self.prev_time = timestamp

        if len(flow_vectors) < self.min_features:
            return None

        raw_flow = np.array(flow_vectors)
        raw_median_dx = np.median(raw_flow[:, 0])
        raw_median_dy = np.median(raw_flow[:, 1])
        raw_vx = -(raw_median_dx / self.fx) / dt
        raw_vy = -(raw_median_dy / self.fy) / dt

        comp_flow = np.array(compensated_flow)
        median_dx = np.median(comp_flow[:, 0])
        median_dy = np.median(comp_flow[:, 1])

        vx = -(median_dx / self.fx) / dt
        vy = -(median_dy / self.fy) / dt

        std_dx = np.std(comp_flow[:, 0])
        std_dy = np.std(comp_flow[:, 1])
        quality = 1.0 / (1.0 + std_dx + std_dy)

        return (vx, vy, quality, len(flow_vectors), raw_vx, raw_vy, gyro)


def main():
    print("=" * 70)
    print("Oak-D Lite Feature Tracker Test (DepthAI 3.x)")
    print("=" * 70)
    print()

    # Create device and start pipeline
    with dai.Device() as device:
        print(f"Device: {device.name}")
        print(f"USB Speed: {device.getUsbSpeed().name}")
        print(f"Connected cameras: {device.getConnectedCameras()}")
        print()

        # Create pipeline
        pipeline = device.createPipeline()

        # Mono camera (left)
        mono = pipeline.create(dai.node.MonoCamera)
        mono.setCamera("left")
        mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
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

        # Link camera to feature tracker
        mono.out.link(feature_tracker.inputImage)

        # Create output queues
        q_features = feature_tracker.outputFeatures.createOutputQueue()
        q_imu = imu.out.createOutputQueue()

        # Start pipeline
        pipeline.start()

        flow_calc = FeatureFlowCalculator(
            fx=400.0, fy=400.0, cx=320.0, cy=200.0, min_features=10
        )

        print("=" * 70)
        print("Streaming... (Ctrl+C to stop)")
        print("=" * 70)
        print()
        print("Try these tests:")
        print("  1. Keep camera still      -> velocities should be ~0")
        print("  2. Pan left/right         -> RAW shows motion, COMPENSATED ~0")
        print("  3. Move camera sideways   -> COMPENSATED shows translation")
        print()

        frame_count = 0
        start_time = time.time()
        last_print = 0

        try:
            while True:
                # Process IMU data
                while q_imu.has():
                    imu_data = q_imu.get()
                    for packet in imu_data.packets:
                        gyro = packet.gyroscope
                        ts = gyro.getTimestamp().total_seconds()
                        flow_calc.add_gyro(ts, [gyro.x, gyro.y, gyro.z])

                # Get features
                if q_features.has():
                    features_data = q_features.get()
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
                            print(f"Gyro (rad/s): [{gyro[0]:+.3f}, {gyro[1]:+.3f}, {gyro[2]:+.3f}]")
                            print()
                            print("                 RAW          COMPENSATED")
                            print(f"  vx (forward):  {raw_vx:+.3f} m/s    {vx:+.3f} m/s")
                            print(f"  vy (left):     {raw_vy:+.3f} m/s    {vy:+.3f} m/s")
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
