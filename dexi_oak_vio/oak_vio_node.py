#!/usr/bin/env python3
"""
OAK-D Lite Stereo VIO Node for PX4 EKF2 fusion.

Uses stereo depth + feature tracking + IMU to estimate position,
then publishes to PX4 EKF2 as visual odometry. Designed to work
alongside ARK Flow (velocity + height) to reduce horizontal drift.

Pipeline:
    OAK-D Lite (stereo + features + IMU on Myriad X VPU)
    -> PnP pose estimation with IMU rotation constraint
    -> Moving average filter
    -> /fmu/in/vehicle_visual_odometry (VehicleOdometry)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import VehicleOdometry, VehicleLocalPosition
import depthai as dai
import numpy as np
import cv2
import time
import threading
from collections import deque


class IMUTracker:
    """Integrates gyro for rotation, detects stationary from accel."""

    def __init__(self, logger):
        self.logger = logger
        self.rotation = np.eye(3)
        self.last_gyro_time = None

        self.gyro_bias = np.zeros(3)
        self.gyro_bias_samples = []
        self.gyro_calibrated = False
        self.gravity = None
        self.gravity_samples = []
        self.gravity_calibrated = False

        self.accel_buffer = deque(maxlen=50)

    def add_imu(self, accel_xyz, gyro_xyz, timestamp):
        if not self.gyro_calibrated:
            self.gyro_bias_samples.append(gyro_xyz)
            if len(self.gyro_bias_samples) >= 100:
                self.gyro_bias = np.mean(self.gyro_bias_samples, axis=0)
                self.gyro_calibrated = True
                self.logger.info(
                    f'Gyro calibrated - bias: [{self.gyro_bias[0]:.4f}, '
                    f'{self.gyro_bias[1]:.4f}, {self.gyro_bias[2]:.4f}]')
            return

        if not self.gravity_calibrated:
            self.gravity_samples.append(accel_xyz)
            if len(self.gravity_samples) >= 100:
                self.gravity = np.mean(self.gravity_samples, axis=0)
                self.gravity_calibrated = True
            return

        gyro_corrected = np.array(gyro_xyz) - self.gyro_bias

        if self.last_gyro_time is not None:
            dt = timestamp - self.last_gyro_time
            if 0 < dt < 0.05:
                angle = np.linalg.norm(gyro_corrected) * dt
                if angle > 1e-8:
                    axis = gyro_corrected / np.linalg.norm(gyro_corrected)
                    K = np.array([
                        [0, -axis[2], axis[1]],
                        [axis[2], 0, -axis[0]],
                        [-axis[1], axis[0], 0]
                    ])
                    dR = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
                    self.rotation = self.rotation @ dR

        self.last_gyro_time = timestamp
        self.accel_buffer.append(accel_xyz)

    def is_stationary(self, threshold=0.15):
        if len(self.accel_buffer) < 20:
            return False
        accel_var = np.std(list(self.accel_buffer), axis=0)
        return np.all(accel_var < threshold)

    def get_rotation(self):
        return self.rotation.copy()

    def is_ready(self):
        return self.gyro_calibrated and self.gravity_calibrated


class OakVIONode(Node):
    def __init__(self):
        super().__init__('oak_vio_node')

        self.declare_parameter('position_variance', [2.0, 2.0, 100.0])
        self.declare_parameter('filter_length', 5)
        self.declare_parameter('max_jump_m', 0.5)
        self.declare_parameter('dry_run', False)

        self.position_variance = self.get_parameter('position_variance').value
        self.filter_length = self.get_parameter('filter_length').value
        self.max_jump_m = self.get_parameter('max_jump_m').value
        self.dry_run = self.get_parameter('dry_run').value

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )

        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', px4_qos)

        self.local_pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position',
            self.local_position_callback, px4_qos)

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0
        self.position_valid = False

        self.imu = IMUTracker(self.get_logger())
        self.vio_position = np.zeros(3)
        self.prev_features_3d = {}
        self.K = None
        self.dist_coeffs = np.zeros(5)

        self.origin_locked = False
        self.offset_north = 0.0
        self.offset_east = 0.0
        self.offset_down = 0.0

        self.north_buffer = deque(maxlen=self.filter_length)
        self.east_buffer = deque(maxlen=self.filter_length)
        self.down_buffer = deque(maxlen=self.filter_length)

        self.frame_count = 0
        self.good_frames = 0
        self.stationary_holds = 0
        self.publish_count = 0
        self.last_log_time = time.time()

        self.running = True

        if self.dry_run:
            self.get_logger().warn('DRY RUN MODE - logging only, NOT publishing to EKF2')

        self.get_logger().info('OAK VIO Node starting...')
        self.get_logger().info(f'Position variance: {self.position_variance}')
        self.get_logger().info(f'Filter length: {self.filter_length}')

        self.camera_thread = threading.Thread(target=self.camera_loop, daemon=True)
        self.camera_thread.start()

    def local_position_callback(self, msg):
        self.drone_x = msg.x
        self.drone_y = msg.y
        self.drone_z = msg.z
        self.position_valid = True

    def camera_loop(self):
        pipeline = dai.Pipeline()

        mono_left = pipeline.create(dai.node.MonoCamera)
        mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setCamera("left")
        mono_left.setFps(30)

        mono_right = pipeline.create(dai.node.MonoCamera)
        mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setCamera("right")
        mono_right.setFps(30)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setSubpixel(True)
        stereo.setLeftRightCheck(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_B)

        feature_tracker = pipeline.create(dai.node.FeatureTracker)
        feature_tracker.setHardwareResources(2, 2)

        imu_node = pipeline.create(dai.node.IMU)
        imu_node.enableIMUSensor(dai.IMUSensor.ACCELEROMETER_RAW, 200)
        imu_node.enableIMUSensor(dai.IMUSensor.GYROSCOPE_RAW, 200)
        imu_node.setBatchReportThreshold(1)
        imu_node.setMaxBatchReports(10)

        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)
        mono_left.out.link(feature_tracker.inputImage)

        xout_depth = pipeline.create(dai.node.XLinkOut)
        xout_depth.setStreamName("depth")
        stereo.depth.link(xout_depth.input)

        xout_feat = pipeline.create(dai.node.XLinkOut)
        xout_feat.setStreamName("features")
        feature_tracker.outputFeatures.link(xout_feat.input)

        xout_imu = pipeline.create(dai.node.XLinkOut)
        xout_imu.setStreamName("imu")
        imu_node.out.link(xout_imu.input)

        try:
            with dai.Device(pipeline) as device:
                self.get_logger().info(
                    f'OAK-D connected: {device.getDeviceName()}, '
                    f'USB: {device.getUsbSpeed().name}')

                calib = device.readCalibration()
                intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, 640, 400)
                self.K = np.array(intrinsics)
                self.get_logger().info(
                    f'Intrinsics: fx={intrinsics[0][0]:.1f}, fy={intrinsics[1][1]:.1f}, '
                    f'cx={intrinsics[0][2]:.1f}, cy={intrinsics[1][2]:.1f}')

                q_depth = device.getOutputQueue("depth", maxSize=4, blocking=False)
                q_feat = device.getOutputQueue("features", maxSize=4, blocking=False)
                q_imu = device.getOutputQueue("imu", maxSize=50, blocking=False)

                last_depth = None

                self.get_logger().info('Calibrating IMU - hold camera still...')

                while self.running and rclpy.ok():
                    imu_data = q_imu.tryGet()
                    if imu_data is not None:
                        for packet in imu_data.packets:
                            acc = packet.acceleroMeter
                            gyro = packet.gyroscope
                            ts = gyro.getTimestamp().total_seconds()
                            self.imu.add_imu(
                                [acc.x, acc.y, acc.z],
                                [gyro.x, gyro.y, gyro.z],
                                ts
                            )

                    depth_data = q_depth.tryGet()
                    if depth_data is not None:
                        last_depth = depth_data.getFrame()

                    feat_data = q_feat.tryGet()
                    if feat_data is not None and last_depth is not None and self.imu.is_ready():
                        self.process_frame(feat_data.trackedFeatures, last_depth)

                    time.sleep(0.001)

        except Exception as e:
            self.get_logger().error(f'Camera error: {e}')

    def process_frame(self, features, depth_frame):
        self.frame_count += 1

        current_2d = {}
        for f in features:
            u, v = int(f.position.x), int(f.position.y)
            if 0 <= u < depth_frame.shape[1] and 0 <= v < depth_frame.shape[0]:
                current_2d[f.id] = (u, v)

        if self.prev_features_3d and self.K is not None:
            matched_3d = []
            matched_2d = []

            for fid, (u, v) in current_2d.items():
                if fid in self.prev_features_3d:
                    matched_3d.append(self.prev_features_3d[fid])
                    matched_2d.append([u, v])

            if len(matched_3d) >= 8:
                if self.imu.is_stationary():
                    self.stationary_holds += 1
                else:
                    pts_3d = np.array(matched_3d, dtype=np.float64)
                    pts_2d = np.array(matched_2d, dtype=np.float64)

                    R_imu = self.imu.get_rotation()
                    rvec_imu, _ = cv2.Rodrigues(R_imu)

                    success, rvec, tvec, inliers = cv2.solvePnPRansac(
                        pts_3d, pts_2d, self.K, self.dist_coeffs,
                        rvec=rvec_imu.copy(),
                        tvec=np.zeros((3, 1)),
                        useExtrinsicGuess=True,
                        iterationsCount=100,
                        reprojectionError=2.0,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )

                    if success and inliers is not None and len(inliers) >= 6:
                        t = tvec.flatten()
                        displacement = np.linalg.norm(t)

                        if displacement < self.max_jump_m:
                            world_t = R_imu.T @ t
                            self.vio_position += world_t
                            self.good_frames += 1

        self.prev_features_3d = {}
        fx, fy = self.K[0, 0], self.K[1, 1]
        cx, cy = self.K[0, 2], self.K[1, 2]

        for fid, (u, v) in current_2d.items():
            depth_mm = depth_frame[v, u]
            if 200 < depth_mm < 8000:
                Z = depth_mm / 1000.0
                X = (u - cx) * Z / fx
                Y = (v - cy) * Z / fy
                self.prev_features_3d[fid] = (X, Y, Z)

        self.publish_vio()

    def publish_vio(self):
        # Forward-facing camera frame to NED:
        #   Camera X (right)   -> NED East
        #   Camera Y (down)    -> NED Down
        #   Camera Z (forward) -> NED North
        vio_north = self.vio_position[2]
        vio_east = self.vio_position[0]
        vio_down = self.vio_position[1]

        if not self.origin_locked and self.position_valid:
            self.offset_north = self.drone_x - vio_north
            self.offset_east = self.drone_y - vio_east
            self.offset_down = self.drone_z - vio_down
            self.origin_locked = True
            self.get_logger().info(
                f'Origin locked - offset: [{self.offset_north:.2f}, '
                f'{self.offset_east:.2f}, {self.offset_down:.2f}]')

        if not self.origin_locked:
            return

        north_raw = vio_north + self.offset_north
        east_raw = vio_east + self.offset_east
        down_raw = vio_down + self.offset_down

        self.north_buffer.append(north_raw)
        self.east_buffer.append(east_raw)
        self.down_buffer.append(down_raw)

        north = sum(self.north_buffer) / len(self.north_buffer)
        east = sum(self.east_buffer) / len(self.east_buffer)
        down = sum(self.down_buffer) / len(self.down_buffer)

        msg = VehicleOdometry()
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        msg.timestamp = now_us
        msg.timestamp_sample = now_us

        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.position = [float(north), float(east), float(down)]

        msg.q = [float('nan')] * 4

        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_UNKNOWN
        msg.velocity = [float('nan')] * 3
        msg.angular_velocity = [float('nan')] * 3

        msg.position_variance = [float(v) for v in self.position_variance]
        msg.orientation_variance = [float('nan')] * 3
        msg.velocity_variance = [float('nan')] * 3

        msg.quality = 50

        if not self.dry_run:
            self.odom_pub.publish(msg)
        self.publish_count += 1

        now = time.time()
        if now - self.last_log_time >= 2.0:
            n_3d = len(self.prev_features_3d)
            stationary = "STILL" if self.imu.is_stationary() else "MOVING"
            self.get_logger().info(
                f'NED: [{north:.3f}, {east:.3f}, {down:.3f}] | '
                f'{stationary} | 3D={n_3d} | '
                f'good={self.good_frames}/{self.frame_count} | '
                f'pub={self.publish_count}')
            self.last_log_time = now

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OakVIONode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
