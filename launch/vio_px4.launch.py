"""
Oak-D Lite Full VIO to PX4 launch file

Uses RTAB-Map for full visual-inertial odometry.
Heavier compute than feature_tracker mode but more robust.

Launches: Oak-D camera, RTAB-Map odometry, PX4 DDS bridge
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('dexi_oak_vio')

    # Oak-D Lite camera node
    oak_camera = Node(
        package='depthai_ros_driver',
        executable='camera_node',
        name='oak_camera',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'oak_d_lite.yaml')],
        remappings=[
            # Remap to standard names for RTAB-Map
            ('oak/left/image_rect', '/stereo/left/image_rect'),
            ('oak/right/image_rect', '/stereo/right/image_rect'),
            ('oak/left/camera_info', '/stereo/left/camera_info'),
            ('oak/right/camera_info', '/stereo/right/camera_info'),
            ('oak/imu/data', '/imu/data'),
        ]
    )

    # RTAB-Map stereo odometry (odometry only, no SLAM)
    stereo_odometry = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        name='stereo_odometry',
        output='screen',
        parameters=[os.path.join(pkg_dir, 'config', 'rtabmap_odom.yaml')],
        remappings=[
            ('left/image_rect', '/stereo/left/image_rect'),
            ('right/image_rect', '/stereo/right/image_rect'),
            ('left/camera_info', '/stereo/left/camera_info'),
            ('right/camera_info', '/stereo/right/camera_info'),
            ('imu', '/imu/data'),
            ('odom', '/vio/odom'),  # Output odometry
        ]
    )

    # PX4 DDS bridge - sends VIO odometry to PX4
    px4_bridge = Node(
        package='dexi_oak_vio',
        executable='px4_dds_bridge',
        name='px4_dds_bridge',
        output='screen',
        parameters=[{
            'send_odometry': True,          # Send full odometry to PX4
            'republish_ark_flow': True,     # Get ARK Flow into ROS2
            'republish_fused_odom': True,   # Get PX4 fused odom
        }]
    )

    return LaunchDescription([
        oak_camera,
        stereo_odometry,
        px4_bridge,
    ])
