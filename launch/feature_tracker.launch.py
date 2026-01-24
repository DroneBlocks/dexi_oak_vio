"""
Oak-D Feature Tracker to PX4 (Simple, no RTAB-Map)

Uses Oak-D's hardware feature tracker + PX4 uXRCE-DDS.
Optionally fuses with ARK Flow in robot_localization.

Launch modes:
  1. Direct to PX4:     Oak-D flow → PX4 DDS → PX4 EKF2 (fuses with ARK Flow)
  2. ROS2 fusion:       Oak-D flow + ARK Flow → robot_localization → PX4 DDS

Usage:
  ros2 launch dexi_oak_vio feature_tracker.launch.py
  ros2 launch dexi_oak_vio feature_tracker.launch.py use_robot_localization:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dexi_oak_vio')

    # Launch arguments
    use_rl_arg = DeclareLaunchArgument(
        'use_robot_localization',
        default_value='false',
        description='Fuse sensors in ROS2 instead of PX4 EKF2'
    )

    use_robot_localization = LaunchConfiguration('use_robot_localization')

    # === Oak-D Camera (Feature Tracker Enabled) ===
    oak_camera = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak_camera',
        output='screen',
        parameters=[{
            # Camera settings
            'camera.i_pipeline_type': 'Stereo',
            'camera.i_nn_type': 'none',
            'camera.i_enable_imu': True,
            'camera.i_usb_speed': 'SUPER_PLUS',

            # Enable hardware feature tracker on left camera
            'left.i_publish_topic': True,
            'left.i_resolution': '400P',
            'left.i_fps': 30,

            'stereo.i_enable_feature_tracker': True,  # Hardware feature tracker

            # IMU
            'imu.i_publish_topic': True,
            'imu.i_acc_freq': 200,
            'imu.i_gyro_freq': 200,
        }],
        remappings=[
            ('oak/imu/data', '/imu/data'),
            ('oak/stereo/left/tracked_features', '/oak/tracked_features'),
        ]
    )

    # === Feature Tracker to Flow Converter ===
    # Now includes IMU compensation to subtract rotation from optical flow
    feature_flow = Node(
        package='dexi_oak_vio',
        executable='feature_tracker_flow',
        name='feature_tracker_flow',
        output='screen',
        parameters=[{
            'features_topic': '/oak/tracked_features',
            'imu_topic': '/imu/data',  # Oak-D IMU for rotation compensation
            'camera_fx': 400.0,  # Adjust for your camera calibration
            'camera_fy': 400.0,
            'min_features': 10,
            'flow_scale': 1.0,
            'enable_imu_compensation': True,  # Subtract rotation from flow
        }]
    )

    # === PX4 DDS Bridge ===
    # Mode 1: Direct to PX4 (no robot_localization)
    px4_bridge_direct = Node(
        condition=UnlessCondition(use_robot_localization),
        package='dexi_oak_vio',
        executable='px4_dds_bridge',
        name='px4_dds_bridge',
        output='screen',
        parameters=[{
            'send_odometry': False,        # Not using full odometry
            'republish_ark_flow': True,    # Get ARK Flow into ROS2
            'republish_fused_odom': True,  # Get PX4 fused odom
        }]
    )

    # Mode 2: With robot_localization
    px4_bridge_rl = Node(
        condition=IfCondition(use_robot_localization),
        package='dexi_oak_vio',
        executable='px4_dds_bridge',
        name='px4_dds_bridge',
        output='screen',
        parameters=[{
            'send_odometry': True,          # Send fused odom to PX4
            'republish_ark_flow': True,     # Get ARK Flow for fusion
            'republish_fused_odom': True,
        }],
        remappings=[
            # Subscribe to robot_localization output
            ('/vio/odom', '/odometry/filtered'),
        ]
    )

    # === Robot Localization (Optional) ===
    robot_localization = Node(
        condition=IfCondition(use_robot_localization),
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'robot_localization.yaml')],
        remappings=[
            ('odometry/filtered', '/odometry/filtered'),
        ]
    )

    return LaunchDescription([
        use_rl_arg,
        oak_camera,
        feature_flow,
        px4_bridge_direct,
        px4_bridge_rl,
        robot_localization,
    ])
