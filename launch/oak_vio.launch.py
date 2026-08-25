"""
Launch file for OAK-D Lite stereo VIO node.

Runs the VIO node that publishes VehicleOdometry to PX4 EKF2.
Designed to run alongside ARK Flow (which handles velocity + height).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    dry_run_arg = DeclareLaunchArgument(
        'dry_run',
        default_value='false',
        description='If true, log VIO position but do not publish to EKF2'
    )

    oak_vio = Node(
        package='dexi_oak_vio',
        executable='oak_vio_node',
        name='oak_vio_node',
        output='screen',
        parameters=[{
            'position_variance': [2.0, 2.0, 100.0],
            'filter_length': 5,
            'max_jump_m': 0.5,
            'dry_run': LaunchConfiguration('dry_run'),
        }]
    )

    return LaunchDescription([
        dry_run_arg,
        oak_vio,
    ])
