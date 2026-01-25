#!/usr/bin/env python3
"""
Oak-D Feature Tracker - ROS2 Pipeline Simulator

Simulates the full ROS2 data flow on Mac without needing ROS2 installed:
  Oak-D → feature_tracker_flow.py → px4_dds_bridge.py → PX4

Shows debug output for each stage so you can see exactly what would be
published to each ROS2 topic and sent to PX4.

Install:
    pip install depthai numpy

Usage:
    python test_ros2_sim.py
"""

import depthai as dai
import numpy as np
import time
import warnings
from collections import deque
from dataclasses import dataclass
from typing import Optional, List, Tuple

warnings.filterwarnings("ignore", category=DeprecationWarning)


# =============================================================================
# Data structures (simulating ROS2 messages)
# =============================================================================

@dataclass
class Vector3:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

@dataclass
class Twist:
    linear: Vector3 = None
    angular: Vector3 = None

    def __post_init__(self):
        if self.linear is None:
            self.linear = Vector3()
        if self.angular is None:
            self.angular = Vector3()

@dataclass
class TwistWithCovariance:
    twist: Twist = None
    covariance: List[float] = None

    def __post_init__(self):
        if self.twist is None:
            self.twist = Twist()
        if self.covariance is None:
            self.covariance = [0.0] * 36

@dataclass
class TwistWithCovarianceStamped:
    """geometry_msgs/TwistWithCovarianceStamped"""
    frame_id: str = "oak_link"
    timestamp_sec: float = 0.0
    twist: TwistWithCovariance = None

    def __post_init__(self):
        if self.twist is None:
            self.twist = TwistWithCovariance()

@dataclass
class VehicleVisualOdometry:
    """PX4 VehicleVisualOdometry (px4_msgs/msg/VehicleVisualOdometry)"""
    timestamp: int = 0  # microseconds
    timestamp_sample: int = 0

    # Position (NaN = not provided)
    position: Tuple[float, float, float] = (float('nan'), float('nan'), float('nan'))

    # Orientation quaternion (NaN = not provided)
    q: Tuple[float, float, float, float] = (float('nan'), float('nan'), float('nan'), float('nan'))

    # Velocity in body frame FRD (Front-Right-Down)
    velocity: Tuple[float, float, float] = (float('nan'), float('nan'), float('nan'))

    # Angular velocity (NaN = not provided)
    angular_velocity: Tuple[float, float, float] = (float('nan'), float('nan'), float('nan'))

    # Variances
    position_variance: Tuple[float, float, float] = (float('nan'), float('nan'), float('nan'))
    orientation_variance: Tuple[float, float, float] = (float('nan'), float('nan'), float('nan'))
    velocity_variance: Tuple[float, float, float] = (float('nan'), float('nan'), float('nan'))

    # Reference frames
    pose_frame: int = 1  # BODY_FRD
    velocity_frame: int = 1  # BODY_FRD


# =============================================================================
# Gyro Buffer (same as in test script)
# =============================================================================

class GyroBuffer:
    def __init__(self, max_age=0.1):
        self.buffer = deque(maxlen=100)
        self.max_age = max_age

    def add(self, timestamp, gyro):
        self.buffer.append((timestamp, gyro))

    def get_average(self, start_time, end_time):
        if not self.buffer:
            return np.zeros(3)
        samples = []
        for ts, gyro in self.buffer:
            if start_time <= ts <= end_time:
                samples.append([gyro.x, gyro.y, gyro.z])
        if not samples:
            _, gyro = self.buffer[-1]
            return np.array([gyro.x, gyro.y, gyro.z])
        return np.mean(samples, axis=0)


# =============================================================================
# Stage 1: Feature Tracker Flow (simulates feature_tracker_flow.py)
# =============================================================================

class FeatureTrackerFlowSim:
    """Simulates the ROS2 feature_tracker_flow node."""

    def __init__(self, fx=400.0, fy=400.0, min_features=10, flow_scale=1.0):
        self.fx = fx
        self.fy = fy
        self.min_features = min_features
        self.flow_scale = flow_scale

        self.prev_features = {}
        self.prev_time = None
        self.gyro_buffer = GyroBuffer()

    def add_gyro(self, timestamp, gyro):
        self.gyro_buffer.add(timestamp, gyro)

    def update(self, features, timestamp) -> Optional[TwistWithCovarianceStamped]:
        """Process features, return ROS2 TwistWithCovarianceStamped or None."""

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

        # Get gyro for compensation
        gyro = self.gyro_buffer.get_average(self.prev_time, timestamp)

        # Compute compensated flow
        compensated_flow = []
        for fid, (x, y) in current.items():
            if fid in self.prev_features:
                px, py = self.prev_features[fid]
                dx = x - px
                dy = y - py

                # Rotation compensation
                dx_rot = self.fx * gyro[1] * dt
                dy_rot = -self.fy * gyro[0] * dt

                dx_trans = dx - dx_rot
                dy_trans = dy - dy_rot
                compensated_flow.append((dx_trans, dy_trans))

        self.prev_features = current
        self.prev_time = timestamp

        if len(compensated_flow) < self.min_features:
            return None

        flow_array = np.array(compensated_flow)
        median_dx = np.median(flow_array[:, 0])
        median_dy = np.median(flow_array[:, 1])

        # Convert to velocity
        vx = -(median_dx / self.fx) / dt * self.flow_scale
        vy = -(median_dy / self.fy) / dt * self.flow_scale

        # Quality
        std_dx = np.std(flow_array[:, 0])
        std_dy = np.std(flow_array[:, 1])
        quality = 1.0 / (1.0 + std_dx + std_dy)

        # Build ROS2 message
        msg = TwistWithCovarianceStamped()
        msg.timestamp_sec = timestamp
        msg.frame_id = "oak_link"

        # Frame conversion: Oak camera → body frame
        # Oak: x=right, y=down, z=forward
        # Body: x=forward, y=left, z=up
        msg.twist.twist.linear.x = float(vy)   # forward
        msg.twist.twist.linear.y = float(-vx)  # left
        msg.twist.twist.linear.z = 0.0

        # Covariance
        base_cov = 0.1 / (quality + 0.01)
        msg.twist.covariance[0] = base_cov   # x
        msg.twist.covariance[7] = base_cov   # y
        msg.twist.covariance[14] = 999.0     # z
        msg.twist.covariance[21] = 999.0     # roll
        msg.twist.covariance[28] = 999.0     # pitch
        msg.twist.covariance[35] = 999.0     # yaw

        return msg, quality, len(compensated_flow), gyro


# =============================================================================
# Stage 2: PX4 DDS Bridge (simulates px4_dds_bridge.py)
# =============================================================================

class PX4DDSBridgeSim:
    """Simulates the ROS2 px4_dds_bridge node."""

    def __init__(self):
        pass

    def twist_to_px4(self, twist_msg: TwistWithCovarianceStamped) -> VehicleVisualOdometry:
        """Convert ROS2 TwistWithCovarianceStamped to PX4 VehicleVisualOdometry."""

        px4_msg = VehicleVisualOdometry()
        px4_msg.timestamp = int(twist_msg.timestamp_sec * 1e6)
        px4_msg.timestamp_sample = px4_msg.timestamp

        # ROS2 uses ENU (East-North-Up), PX4 uses NED (North-East-Down)
        # ROS2 body: x=forward, y=left, z=up
        # PX4 FRD:   x=forward, y=right, z=down

        ros_vx = twist_msg.twist.twist.linear.x  # forward
        ros_vy = twist_msg.twist.twist.linear.y  # left
        ros_vz = twist_msg.twist.twist.linear.z  # up

        # Convert to PX4 FRD
        px4_vx = ros_vx      # forward → forward
        px4_vy = -ros_vy     # left → right (negate)
        px4_vz = -ros_vz     # up → down (negate)

        px4_msg.velocity = (px4_vx, px4_vy, px4_vz)

        # Velocity variance from covariance diagonal
        px4_msg.velocity_variance = (
            twist_msg.twist.covariance[0],   # x variance
            twist_msg.twist.covariance[7],   # y variance
            twist_msg.twist.covariance[14],  # z variance
        )

        px4_msg.velocity_frame = 1  # BODY_FRD

        return px4_msg


# =============================================================================
# Main
# =============================================================================

def main():
    print("=" * 75)
    print("Oak-D Feature Tracker - ROS2 Pipeline Simulator")
    print("=" * 75)
    print()
    print("Simulating: Oak-D → feature_tracker_flow → px4_dds_bridge → PX4")
    print()

    # Create pipeline
    pipeline = dai.Pipeline()

    mono = pipeline.create(dai.node.MonoCamera)
    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    mono.setCamera("left")
    mono.setFps(30)

    feature_tracker = pipeline.create(dai.node.FeatureTracker)
    feature_tracker.setHardwareResources(2, 2)

    imu = pipeline.create(dai.node.IMU)
    imu.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 200)
    imu.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
    imu.setBatchReportThreshold(1)
    imu.setMaxBatchReports(10)

    xout_features = pipeline.create(dai.node.XLinkOut)
    xout_features.setStreamName("features")
    xout_imu = pipeline.create(dai.node.XLinkOut)
    xout_imu.setStreamName("imu")

    mono.out.link(feature_tracker.inputImage)
    feature_tracker.outputFeatures.link(xout_features.input)
    imu.out.link(xout_imu.input)

    print("Connecting to Oak-D Lite...")

    with dai.Device(pipeline) as device:
        print(f"Connected: {device.getProductName()}")
        print()

        q_features = device.getOutputQueue("features", maxSize=4, blocking=False)
        q_imu = device.getOutputQueue("imu", maxSize=50, blocking=False)

        # Create simulated ROS2 nodes
        feature_flow = FeatureTrackerFlowSim(fx=400.0, fy=400.0, min_features=10)
        px4_bridge = PX4DDSBridgeSim()

        print("=" * 75)
        print("Streaming... (Ctrl+C to stop)")
        print("=" * 75)
        print()

        last_print = 0
        frame_count = 0

        try:
            while True:
                # Process IMU
                while True:
                    imu_data = q_imu.tryGet()
                    if imu_data is None:
                        break
                    for packet in imu_data.packets:
                        gyro = packet.gyroscope
                        ts = packet.gyroscope.getTimestamp().total_seconds()
                        feature_flow.add_gyro(ts, gyro)

                # Process features
                features_data = q_features.tryGet()

                if features_data is not None:
                    features = features_data.trackedFeatures
                    timestamp = features_data.getTimestamp().total_seconds()
                    frame_count += 1

                    result = feature_flow.update(features, timestamp)
                    now = time.time()

                    if now - last_print > 0.5:
                        last_print = now

                        print("-" * 75)
                        print(f"Frame {frame_count} | Features: {len(features)}")
                        print("-" * 75)

                        if result:
                            twist_msg, quality, matched, gyro = result

                            # Stage 1: feature_tracker_flow output
                            print()
                            print("[STAGE 1] feature_tracker_flow.py → /oak/flow/twist")
                            print(f"  Topic: geometry_msgs/TwistWithCovarianceStamped")
                            print(f"  frame_id: {twist_msg.frame_id}")
                            print(f"  Matched features: {matched} | Quality: {quality:.2f}")
                            print(f"  Gyro compensation: [{gyro[0]:+.3f}, {gyro[1]:+.3f}, {gyro[2]:+.3f}] rad/s")
                            print()
                            print(f"  twist.linear:")
                            print(f"    x: {twist_msg.twist.twist.linear.x:+.4f}  (forward)")
                            print(f"    y: {twist_msg.twist.twist.linear.y:+.4f}  (left)")
                            print(f"    z: {twist_msg.twist.twist.linear.z:+.4f}  (up)")
                            print(f"  covariance: [x={twist_msg.twist.covariance[0]:.3f}, y={twist_msg.twist.covariance[7]:.3f}, z={twist_msg.twist.covariance[14]:.1f}]")

                            # Stage 2: px4_dds_bridge conversion
                            px4_msg = px4_bridge.twist_to_px4(twist_msg)

                            print()
                            print("[STAGE 2] px4_dds_bridge.py → /fmu/in/vehicle_visual_odometry")
                            print(f"  Topic: px4_msgs/VehicleVisualOdometry")
                            print(f"  Frame: BODY_FRD (Front-Right-Down)")
                            print(f"  ENU→NED conversion applied")
                            print()
                            print(f"  velocity (m/s):")
                            print(f"    x: {px4_msg.velocity[0]:+.4f}  (front)")
                            print(f"    y: {px4_msg.velocity[1]:+.4f}  (right)")
                            print(f"    z: {px4_msg.velocity[2]:+.4f}  (down)")
                            print(f"  velocity_variance: [{px4_msg.velocity_variance[0]:.3f}, {px4_msg.velocity_variance[1]:.3f}, {px4_msg.velocity_variance[2]:.1f}]")
                            print(f"  velocity_frame: BODY_FRD")

                            # What PX4 EKF2 will do
                            print()
                            print("[STAGE 3] PX4 EKF2 Fusion")
                            print(f"  This velocity estimate will be fused with:")
                            print(f"    - ARK Flow optical flow (if EKF2_OF_CTRL=1)")
                            print(f"    - IMU accelerometer integration")
                            print(f"    - GPS (if available)")
                            print(f"  Result: Improved position hold accuracy")

                        else:
                            if len(features) < 10:
                                print("  (Not enough features - point at textured surface)")
                            else:
                                print("  (Waiting for motion...)")

                        print()

                time.sleep(0.001)

        except KeyboardInterrupt:
            print()
            print("=" * 75)
            print("Stopped.")
            print("=" * 75)


if __name__ == "__main__":
    main()
