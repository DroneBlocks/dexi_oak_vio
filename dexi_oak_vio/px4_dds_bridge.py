#!/usr/bin/env python3
"""
PX4 DDS Bridge

Bridges ROS2 odometry/velocity to PX4 via uXRCE-DDS.
Also subscribes to ARK Flow data from PX4 and republishes as ROS2 twist.

Uses px4_msgs directly via uXRCE-DDS.

PX4 Topics:
- /fmu/in/vehicle_visual_odometry  (VIO → PX4)
- /fmu/out/vehicle_optical_flow    (ARK Flow ← PX4)
- /fmu/out/vehicle_odometry        (Fused odom ← PX4)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
import numpy as np

# PX4 messages
from px4_msgs.msg import (
    VehicleVisualOdometry,
    VehicleOdometry,
    VehicleOpticalFlow,
    TimesyncStatus,
)


class PX4DDSBridge(Node):
    def __init__(self):
        super().__init__('px4_dds_bridge')

        # Parameters
        self.declare_parameter('send_odometry', True)
        self.declare_parameter('send_velocity_only', False)  # True = just velocity, no pose
        self.declare_parameter('republish_ark_flow', True)
        self.declare_parameter('republish_fused_odom', True)

        self.send_odometry = self.get_parameter('send_odometry').value
        self.send_velocity_only = self.get_parameter('send_velocity_only').value
        self.republish_ark_flow = self.get_parameter('republish_ark_flow').value
        self.republish_fused_odom = self.get_parameter('republish_fused_odom').value

        # QoS for PX4 topics (must match PX4's QoS)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Standard ROS2 QoS
        ros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Time sync with PX4
        self.px4_timestamp_offset = 0
        self.timesync_sub = self.create_subscription(
            TimesyncStatus,
            '/fmu/out/timesync_status',
            self.timesync_callback,
            px4_qos
        )

        # === TO PX4 ===

        # Subscriber for ROS2 odometry (from RTAB-Map or robot_localization)
        if self.send_odometry:
            self.odom_sub = self.create_subscription(
                Odometry,
                '/vio/odom',
                self.odom_callback,
                ros_qos
            )

        # Subscriber for velocity only (from feature_tracker_flow)
        self.twist_sub = self.create_subscription(
            TwistWithCovarianceStamped,
            '/oak/flow/twist',
            self.twist_callback,
            ros_qos
        )

        # Publisher to PX4 VIO input
        self.vio_pub = self.create_publisher(
            VehicleVisualOdometry,
            '/fmu/in/vehicle_visual_odometry',
            px4_qos
        )

        # === FROM PX4 ===

        # ARK Flow data from PX4
        if self.republish_ark_flow:
            self.ark_flow_sub = self.create_subscription(
                VehicleOpticalFlow,
                '/fmu/out/vehicle_optical_flow',
                self.ark_flow_callback,
                px4_qos
            )
            self.ark_flow_pub = self.create_publisher(
                TwistWithCovarianceStamped,
                '/ark_flow/twist',
                10
            )

        # Fused odometry from PX4 EKF2
        if self.republish_fused_odom:
            self.fused_odom_sub = self.create_subscription(
                VehicleOdometry,
                '/fmu/out/vehicle_odometry',
                self.fused_odom_callback,
                px4_qos
            )
            self.fused_odom_pub = self.create_publisher(
                Odometry,
                '/px4/odom',
                10
            )

        self.get_logger().info('PX4 DDS Bridge started')
        self.get_logger().info(f'  Send odometry: {self.send_odometry}')
        self.get_logger().info(f'  Republish ARK Flow: {self.republish_ark_flow}')

    def timesync_callback(self, msg: TimesyncStatus):
        """Sync time with PX4."""
        self.px4_timestamp_offset = msg.observed_offset

    def get_px4_timestamp(self):
        """Get current time in PX4 microseconds."""
        now_ns = self.get_clock().now().nanoseconds
        return int(now_ns / 1000) + self.px4_timestamp_offset

    def odom_callback(self, msg: Odometry):
        """Convert ROS2 Odometry to PX4 VehicleVisualOdometry."""
        vio = VehicleVisualOdometry()
        vio.timestamp = self.get_px4_timestamp()
        vio.timestamp_sample = vio.timestamp

        # Position (ENU to NED)
        vio.position[0] = msg.pose.pose.position.y   # N = E
        vio.position[1] = msg.pose.pose.position.x   # E = N
        vio.position[2] = -msg.pose.pose.position.z  # D = -U

        # Orientation (ENU to NED quaternion)
        q = msg.pose.pose.orientation
        vio.q[0] = q.w
        vio.q[1] = q.y   # swap x/y
        vio.q[2] = q.x
        vio.q[3] = -q.z  # negate z

        # Velocity (ENU to NED)
        vio.velocity[0] = msg.twist.twist.linear.y
        vio.velocity[1] = msg.twist.twist.linear.x
        vio.velocity[2] = -msg.twist.twist.linear.z

        # Angular velocity
        vio.angular_velocity[0] = msg.twist.twist.angular.y
        vio.angular_velocity[1] = msg.twist.twist.angular.x
        vio.angular_velocity[2] = -msg.twist.twist.angular.z

        # Covariance (simplified - use diagonal)
        # PX4 expects upper-right triangular
        vio.position_variance[0] = max(msg.pose.covariance[0], 0.01)
        vio.position_variance[1] = max(msg.pose.covariance[7], 0.01)
        vio.position_variance[2] = max(msg.pose.covariance[14], 0.01)

        vio.velocity_variance[0] = max(msg.twist.covariance[0], 0.01)
        vio.velocity_variance[1] = max(msg.twist.covariance[7], 0.01)
        vio.velocity_variance[2] = max(msg.twist.covariance[14], 0.01)

        # Reference frames
        vio.pose_frame = VehicleVisualOdometry.POSE_FRAME_NED
        vio.velocity_frame = VehicleVisualOdometry.VELOCITY_FRAME_NED

        self.vio_pub.publish(vio)

    def twist_callback(self, msg: TwistWithCovarianceStamped):
        """Convert velocity-only to PX4 VehicleVisualOdometry."""
        vio = VehicleVisualOdometry()
        vio.timestamp = self.get_px4_timestamp()
        vio.timestamp_sample = vio.timestamp

        # No position - set to NaN
        vio.position[0] = float('nan')
        vio.position[1] = float('nan')
        vio.position[2] = float('nan')

        # No orientation - set to NaN
        vio.q[0] = float('nan')
        vio.q[1] = float('nan')
        vio.q[2] = float('nan')
        vio.q[3] = float('nan')

        # Velocity (body frame to NED - assumes msg is in body frame)
        # Body: x=forward, y=left, z=up
        # NED: x=north, y=east, z=down
        # For body-frame velocity, PX4 handles rotation internally
        vio.velocity[0] = msg.twist.twist.linear.x
        vio.velocity[1] = -msg.twist.twist.linear.y  # left to right
        vio.velocity[2] = -msg.twist.twist.linear.z

        # Covariance
        vio.velocity_variance[0] = max(msg.twist.covariance[0], 0.01)
        vio.velocity_variance[1] = max(msg.twist.covariance[7], 0.01)
        vio.velocity_variance[2] = 999.0  # No Z velocity info

        # Use body frame for velocity
        vio.pose_frame = VehicleVisualOdometry.POSE_FRAME_UNKNOWN
        vio.velocity_frame = VehicleVisualOdometry.VELOCITY_FRAME_BODY_FRD

        self.vio_pub.publish(vio)

    def ark_flow_callback(self, msg: VehicleOpticalFlow):
        """Republish ARK Flow as ROS2 TwistWithCovarianceStamped."""
        twist = TwistWithCovarianceStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = 'ark_flow_link'

        # Optical flow is in rad/s integrated over integration_timespan
        # Convert to velocity using distance
        if msg.distance_m > 0.1:
            # v = flow_rate * distance
            dt = msg.integration_timespan_us / 1e6
            if dt > 0:
                twist.twist.twist.linear.x = (msg.pixel_flow[0] / dt) * msg.distance_m
                twist.twist.twist.linear.y = (msg.pixel_flow[1] / dt) * msg.distance_m

        twist.twist.twist.linear.z = 0.0

        # Quality to covariance
        quality = msg.quality / 255.0 if msg.quality > 0 else 0.01
        base_cov = 0.1 / quality
        twist.twist.covariance[0] = base_cov
        twist.twist.covariance[7] = base_cov
        twist.twist.covariance[14] = 999.0

        self.ark_flow_pub.publish(twist)

    def fused_odom_callback(self, msg: VehicleOdometry):
        """Republish PX4 fused odometry as ROS2 Odometry."""
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        # Position (NED to ENU)
        odom.pose.pose.position.x = msg.position[1]   # E = N
        odom.pose.pose.position.y = msg.position[0]   # N = E
        odom.pose.pose.position.z = -msg.position[2]  # U = -D

        # Orientation (NED to ENU)
        odom.pose.pose.orientation.w = msg.q[0]
        odom.pose.pose.orientation.x = msg.q[2]
        odom.pose.pose.orientation.y = msg.q[1]
        odom.pose.pose.orientation.z = -msg.q[3]

        # Velocity (NED to ENU)
        odom.twist.twist.linear.x = msg.velocity[1]
        odom.twist.twist.linear.y = msg.velocity[0]
        odom.twist.twist.linear.z = -msg.velocity[2]

        odom.twist.twist.angular.x = msg.angular_velocity[1]
        odom.twist.twist.angular.y = msg.angular_velocity[0]
        odom.twist.twist.angular.z = -msg.angular_velocity[2]

        self.fused_odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = PX4DDSBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
