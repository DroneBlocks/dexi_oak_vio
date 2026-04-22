#!/usr/bin/env python3
"""
Minimal OAK-D Flow Node for ROS2

Directly connects to OAK-D Lite, runs feature tracking with IMU compensation,
and publishes velocity to ROS2. No depthai_ros required.

Publishes:
- /oak/flow/twist (TwistWithCovarianceStamped) - feeds into px4_dds_bridge
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistWithCovarianceStamped
import depthai as dai
import numpy as np
from collections import deque
import threading


class GyroBuffer:
    """Thread-safe gyro buffer."""

    def __init__(self):
        self.buffer = deque(maxlen=100)
        self.lock = threading.Lock()

    def add(self, timestamp, gyro_xyz):
        with self.lock:
            self.buffer.append((timestamp, gyro_xyz))

    def get_average(self, start_time, end_time):
        with self.lock:
            if not self.buffer:
                return np.zeros(3)
            samples = [g for t, g in self.buffer if start_time <= t <= end_time]
            if not samples:
                return np.array(self.buffer[-1][1]) if self.buffer else np.zeros(3)
            return np.mean(samples, axis=0)


class OakFlowNode(Node):
    def __init__(self):
        super().__init__('oak_flow_node')

        # Parameters
        self.declare_parameter('camera_fx', 400.0)
        self.declare_parameter('camera_fy', 400.0)
        self.declare_parameter('min_features', 10)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('enable_imu_compensation', True)  # Enabled with corrected signs

        self.fx = self.get_parameter('camera_fx').value
        self.fy = self.get_parameter('camera_fy').value
        self.min_features = self.get_parameter('min_features').value
        self.enable_imu_compensation = self.get_parameter('enable_imu_compensation').value

        # Publisher
        self.twist_pub = self.create_publisher(
            TwistWithCovarianceStamped,
            '/oak/flow/twist',
            10
        )

        # State
        self.gyro_buffer = GyroBuffer()
        self.prev_features = {}
        self.prev_time = None
        self.running = True

        self.get_logger().info('OAK Flow Node starting...')
        self.get_logger().info(f'  Publishing to: /oak/flow/twist')
        self.get_logger().info(f'  IMU compensation: {self.enable_imu_compensation}')

        # Start camera thread
        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.camera_thread.start()

    def camera_loop(self):
        """Main camera processing loop (runs in separate thread)."""
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

        try:
            with dai.Device(pipeline) as device:
                self.get_logger().info(f'Connected to {device.getDeviceName()}')
                self.get_logger().info(f'USB Speed: {device.getUsbSpeed().name}')

                q_features = device.getOutputQueue("features", maxSize=4, blocking=False)
                q_imu = device.getOutputQueue("imu", maxSize=50, blocking=False)

                while self.running and rclpy.ok():
                    # Process IMU
                    while True:
                        imu_data = q_imu.tryGet()
                        if imu_data is None:
                            break
                        for packet in imu_data.packets:
                            gyro = packet.gyroscope
                            ts = gyro.getTimestamp().total_seconds()
                            self.gyro_buffer.add(ts, [gyro.x, gyro.y, gyro.z])

                    # Process features
                    features_data = q_features.tryGet()
                    if features_data is not None:
                        self.process_features(features_data)

        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')

    def process_features(self, features_data):
        """Process features and publish velocity."""
        features = features_data.trackedFeatures
        timestamp = features_data.getTimestamp().total_seconds()

        # Build current features dict
        current = {f.id: (f.position.x, f.position.y) for f in features}

        # Debug: log feature count periodically
        if not hasattr(self, '_debug_counter'):
            self._debug_counter = 0
        self._debug_counter += 1
        if self._debug_counter % 30 == 0:
            matched = sum(1 for fid in current if fid in self.prev_features) if self.prev_features else 0
            self.get_logger().info(f'Features: {len(current)} tracked, {matched} matched (need {self.min_features})')

        if not self.prev_features or self.prev_time is None:
            self.prev_features = current
            self.prev_time = timestamp
            return

        dt = timestamp - self.prev_time
        if dt <= 0 or dt > 0.5:
            self.prev_features = current
            self.prev_time = timestamp
            return

        # Get gyro for compensation
        gyro = self.gyro_buffer.get_average(self.prev_time, timestamp)

        # Compute compensated flow
        compensated_flow = []
        for fid, (x, y) in current.items():
            if fid in self.prev_features:
                px, py = self.prev_features[fid]
                dx = x - px
                dy = y - py

                if self.enable_imu_compensation:
                    # Remove rotation (OAK-D Lite IMU frame)
                    # Testing: try gyro[1] for yaw (horizontal compensation)
                    dx_rot = self.fx * gyro[1] * dt   # Try Y axis for yaw
                    dy_rot = self.fy * gyro[0] * dt   # Try X axis for pitch
                    compensated_flow.append((dx - dx_rot, dy - dy_rot))
                else:
                    compensated_flow.append((dx, dy))

        self.prev_features = current
        self.prev_time = timestamp

        if len(compensated_flow) < self.min_features:
            return

        # Compute velocity
        flow = np.array(compensated_flow)
        median_dx = np.median(flow[:, 0])
        median_dy = np.median(flow[:, 1])

        vx = -(median_dx / self.fx) / dt
        vy = -(median_dy / self.fy) / dt

        # Quality
        std_dx = np.std(flow[:, 0])
        std_dy = np.std(flow[:, 1])
        quality = 1.0 / (1.0 + std_dx + std_dy)

        # Publish
        msg = TwistWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oak_link'

        # Body frame: x=forward, y=left, z=up
        # vx = horizontal pixel motion = left/right camera movement
        # vy = vertical pixel motion = up/down camera movement (or forward if tilted down)
        msg.twist.twist.linear.x = float(vy)   # Forward from vertical flow (flipped)
        msg.twist.twist.linear.y = float(-vx)  # Left from horizontal flow
        msg.twist.twist.linear.z = 0.0

        # Covariance
        base_cov = 0.1 / (quality + 0.01)
        cov = [0.0] * 36
        cov[0] = base_cov
        cov[7] = base_cov
        cov[14] = 999.0
        cov[21] = 999.0
        cov[28] = 999.0
        cov[35] = 999.0
        msg.twist.covariance = cov

        self.twist_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakFlowNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
