#!/usr/bin/env python3
"""
Feature Tracker to Optical Flow Converter (with IMU Rotation Compensation)

Converts Oak-D's hardware feature tracker output to velocity estimates.
The Oak-D Myriad X runs feature tracking on-chip, so this is lightweight on Pi.

Uses gyroscope data to subtract rotational motion from optical flow,
leaving only translational velocity - this prevents panning from appearing
as lateral movement.

Publishes:
- geometry_msgs/TwistWithCovarianceStamped for robot_localization
- Can also feed into px4_dds_bridge for PX4
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from depthai_ros_msgs.msg import TrackedFeatures
from geometry_msgs.msg import TwistWithCovarianceStamped, Vector3
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
from collections import deque


class GyroBuffer:
    """Buffers gyro data and provides interpolated/averaged values."""

    def __init__(self, max_samples=100):
        self.buffer = deque(maxlen=max_samples)

    def add(self, timestamp_sec, angular_velocity):
        """Add gyro reading to buffer (timestamp in seconds, angular_velocity is Vector3)."""
        self.buffer.append((timestamp_sec, angular_velocity))

    def get_average(self, start_time, end_time):
        """Get average gyro between two timestamps (in seconds)."""
        if not self.buffer:
            return np.zeros(3)

        samples = []
        for ts, gyro in self.buffer:
            if start_time <= ts <= end_time:
                samples.append([gyro.x, gyro.y, gyro.z])

        if not samples:
            # Use most recent if no samples in range
            if self.buffer:
                _, gyro = self.buffer[-1]
                return np.array([gyro.x, gyro.y, gyro.z])
            return np.zeros(3)

        return np.mean(samples, axis=0)


class FeatureTrackerFlow(Node):
    def __init__(self):
        super().__init__('feature_tracker_flow')

        # Parameters
        self.declare_parameter('features_topic', '/oak/stereo/left/tracked_features')
        self.declare_parameter('imu_topic', '/oak/imu/data')
        self.declare_parameter('depth_scale', 1.0)  # meters per depth unit
        self.declare_parameter('camera_fx', 400.0)  # focal length x (pixels)
        self.declare_parameter('camera_fy', 400.0)  # focal length y (pixels)
        self.declare_parameter('min_features', 10)  # minimum features for valid flow
        self.declare_parameter('flow_scale', 1.0)   # velocity scaling factor
        self.declare_parameter('enable_imu_compensation', True)  # enable gyro compensation

        self.features_topic = self.get_parameter('features_topic').value
        self.imu_topic = self.get_parameter('imu_topic').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.min_features = self.get_parameter('min_features').value
        self.flow_scale = self.get_parameter('flow_scale').value
        self.enable_imu_compensation = self.get_parameter('enable_imu_compensation').value

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # IMU buffer for rotation compensation
        self.gyro_buffer = GyroBuffer()

        # Store previous features for tracking
        self.prev_features = {}
        self.prev_timestamp = None
        self.prev_timestamp_sec = None

        # Subscriber to tracked features
        self.features_sub = self.create_subscription(
            TrackedFeatures,
            self.features_topic,
            self.features_callback,
            sensor_qos
        )

        # Subscriber to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            sensor_qos
        )

        # Publisher for velocity (robot_localization compatible)
        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/oak/flow/twist',
            10
        )

        # Stats
        self.feature_count = 0
        self.valid_flow_count = 0
        self.imu_count = 0

        self.get_logger().info(f'Feature Tracker Flow started (with IMU compensation)')
        self.get_logger().info(f'  Features topic: {self.features_topic}')
        self.get_logger().info(f'  IMU topic: {self.imu_topic}')
        self.get_logger().info(f'  IMU compensation: {self.enable_imu_compensation}')
        self.get_logger().info(f'  Publishing to: /oak/flow/twist')

    def imu_callback(self, msg: Imu):
        """Buffer IMU data for rotation compensation."""
        timestamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.gyro_buffer.add(timestamp_sec, msg.angular_velocity)
        self.imu_count += 1

    def features_callback(self, msg: TrackedFeatures):
        """Convert tracked features to velocity estimate with IMU rotation compensation."""
        current_time = self.get_clock().now()
        current_time_sec = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

        # Build dict of current features by ID
        current_features = {}
        for feature in msg.features:
            current_features[feature.id] = (feature.position.x, feature.position.y)

        self.feature_count = len(current_features)

        # Need previous frame to compute flow
        if not self.prev_features or self.prev_timestamp is None:
            self.prev_features = current_features
            self.prev_timestamp = current_time
            self.prev_timestamp_sec = current_time_sec
            return

        # Compute dt
        dt = (current_time - self.prev_timestamp).nanoseconds / 1e9
        if dt <= 0 or dt > 0.5:  # Skip if too long/short
            self.prev_features = current_features
            self.prev_timestamp = current_time
            self.prev_timestamp_sec = current_time_sec
            return

        # Get average gyro during this interval for rotation compensation
        gyro = np.zeros(3)
        if self.enable_imu_compensation and self.imu_count > 0:
            gyro = self.gyro_buffer.get_average(self.prev_timestamp_sec, current_time_sec)

        # Find matched features and compute flow with rotation compensation
        compensated_flow = []
        for fid, (x, y) in current_features.items():
            if fid in self.prev_features:
                px, py = self.prev_features[fid]
                dx = x - px  # raw pixel flow
                dy = y - py

                # Compute expected pixel motion from rotation
                # For a pinhole camera:
                # - Yaw (rotation around Y/up) causes horizontal flow
                # - Pitch (rotation around X/right) causes vertical flow
                # Camera convention: x-right, y-down, z-forward
                dx_rot = self.fx * gyro[1] * dt   # yaw causes horizontal flow
                dy_rot = -self.fy * gyro[0] * dt  # pitch causes vertical flow

                # Compensated flow (translation only)
                dx_trans = dx - dx_rot
                dy_trans = dy - dy_rot

                compensated_flow.append((dx_trans, dy_trans))

        # Need minimum features for reliable estimate
        if len(compensated_flow) < self.min_features:
            self.prev_features = current_features
            self.prev_timestamp = current_time
            self.prev_timestamp_sec = current_time_sec
            return

        # Compute median flow (robust to outliers)
        flow_array = np.array(compensated_flow)
        median_dx = np.median(flow_array[:, 0])
        median_dy = np.median(flow_array[:, 1])

        # Convert pixel flow to velocity (assumes known height/focal length)
        # v = (pixel_flow * height) / (focal_length * dt)
        # For now, output normalized flow - scale with depth externally
        vx = -(median_dx / self.fx) / dt * self.flow_scale
        vy = -(median_dy / self.fy) / dt * self.flow_scale

        # Compute flow quality (std dev of compensated flow vectors)
        std_dx = np.std(flow_array[:, 0])
        std_dy = np.std(flow_array[:, 1])
        quality = 1.0 / (1.0 + std_dx + std_dy)  # 0-1, higher is better

        # Publish twist
        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.stamp = current_time.to_msg()
        twist_msg.header.frame_id = 'oak_link'

        # Linear velocity (x=forward, y=left in body frame)
        # Oak-D camera frame: x=right, y=down, z=forward
        # Convert to body frame: x=forward (camera z), y=left (-camera x)
        twist_msg.twist.twist.linear.x = float(vy)   # forward = camera y flow
        twist_msg.twist.twist.linear.y = float(-vx)  # left = -camera x flow
        twist_msg.twist.twist.linear.z = 0.0

        # Covariance (diagonal, based on quality)
        base_cov = 0.1 / (quality + 0.01)  # Lower quality = higher covariance
        cov = [0.0] * 36
        cov[0] = base_cov   # x
        cov[7] = base_cov   # y
        cov[14] = 999.0     # z (no info)
        cov[21] = 999.0     # roll (no info)
        cov[28] = 999.0     # pitch (no info)
        cov[35] = 999.0     # yaw (no info)
        twist_msg.twist.covariance = cov

        self.twist_pub.publish(twist_msg)
        self.valid_flow_count += 1

        # Update for next iteration
        self.prev_features = current_features
        self.prev_timestamp = current_time
        self.prev_timestamp_sec = current_time_sec


def main(args=None):
    rclpy.init(args=args)
    node = FeatureTrackerFlow()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f'Shutting down. Processed {node.valid_flow_count} valid flow frames'
        )
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
