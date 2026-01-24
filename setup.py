from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dexi_oak_vio'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Dennis Baldwin',
    maintainer_email='db@droneblocks.io',
    description='Oak-D Lite VIO to PX4 bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            # Standalone OAK-D flow node (uses depthai directly, no depthai_ros needed)
            'oak_flow_node = dexi_oak_vio.oak_flow_node:main',

            # Feature tracker with IMU compensation (requires depthai_ros)
            'feature_tracker_flow = dexi_oak_vio.feature_tracker_flow:main',

            # PX4 uXRCE-DDS bridge
            'px4_dds_bridge = dexi_oak_vio.px4_dds_bridge:main',
        ],
    },
)
