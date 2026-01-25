"""
Oak-D Flow Node Launch File

Simple launch file for the standalone oak_flow_node.
Uses depthai directly (no depthai_ros_driver required).

Usage:
  ros2 launch dexi_oak_vio oak_flow.launch.py
  ros2 launch dexi_oak_vio oak_flow.launch.py enable_imu_compensation:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    camera_fx_arg = DeclareLaunchArgument(
        'camera_fx',
        default_value='400.0',
        description='Camera focal length X (pixels)'
    )

    camera_fy_arg = DeclareLaunchArgument(
        'camera_fy',
        default_value='400.0',
        description='Camera focal length Y (pixels)'
    )

    min_features_arg = DeclareLaunchArgument(
        'min_features',
        default_value='10',
        description='Minimum features for valid velocity estimate'
    )

    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu_compensation',
        default_value='true',
        description='Enable IMU-based rotation compensation'
    )

    # Oak Flow Node
    oak_flow_node = Node(
        package='dexi_oak_vio',
        executable='oak_flow_node',
        name='oak_flow_node',
        output='screen',
        parameters=[{
            'camera_fx': LaunchConfiguration('camera_fx'),
            'camera_fy': LaunchConfiguration('camera_fy'),
            'min_features': LaunchConfiguration('min_features'),
            'enable_imu_compensation': LaunchConfiguration('enable_imu_compensation'),
        }]
    )

    return LaunchDescription([
        camera_fx_arg,
        camera_fy_arg,
        min_features_arg,
        enable_imu_arg,
        oak_flow_node,
    ])
