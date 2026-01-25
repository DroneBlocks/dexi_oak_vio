from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the package directory
    pkg_dexi_bringup = get_package_share_directory('dexi_bringup')

    # Create the launch description
    ld = LaunchDescription()

    # Declare the launch arguments
    ld.add_action(DeclareLaunchArgument('apriltags', default_value='false', description='Enable AprilTag detection'))
    ld.add_action(DeclareLaunchArgument('servos', default_value='false', description='Enable servo control'))
    ld.add_action(DeclareLaunchArgument('gpio', default_value='false', description='Enable GPIO control'))
    ld.add_action(DeclareLaunchArgument('offboard', default_value='false', description='Enable offboard control'))
    ld.add_action(DeclareLaunchArgument('keyboard_control', default_value='false', description='Enable keyboard teleop control'))
    ld.add_action(DeclareLaunchArgument('rosbridge', default_value='true', description='Enable ROS bridge'))
    ld.add_action(DeclareLaunchArgument('camera', default_value='true', description='Enable camera'))
    ld.add_action(DeclareLaunchArgument('yolo', default_value='false', description='Enable YOLO detection'))
    ld.add_action(DeclareLaunchArgument('oak_flow', default_value='false', description='Enable OAK-D optical flow node'))

    apriltags = LaunchConfiguration('apriltags')
    servos = LaunchConfiguration('servos')
    gpio = LaunchConfiguration('gpio')
    offboard = LaunchConfiguration('offboard')
    keyboard_control = LaunchConfiguration('keyboard_control')
    rosbridge = LaunchConfiguration('rosbridge')
    camera = LaunchConfiguration('camera')
    yolo = LaunchConfiguration('yolo')
    oak_flow = LaunchConfiguration('oak_flow')

    # Create micro_ros_agent node
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyAMA3', '-b', '921600']
    )
    ld.add_action(micro_ros_agent)

    # Create rosbridge websocket node
    rosbridge_websocket = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        parameters=[{
            'port': 9090,
            'address': '',
            'ssl': False,
            'certfile': '',
            'keyfile': '',
            'authenticate': False,
        }],
        condition=IfCondition(rosbridge)
    )
    ld.add_action(rosbridge_websocket)

    # Create rosapi node
    rosapi = Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        condition=IfCondition(rosbridge)
    )
    ld.add_action(rosapi)

    # Include Pi5 LED service launch file
    led_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_led'), 'launch', 'led_service_pi5.launch.py')
        ])
    )
    ld.add_action(led_launch)

    # Camera launch file for Pi5 using dexi_camera (UVC camera)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_camera'), 'camera.launch.py')
        ]),
        condition=IfCondition(camera)
    )
    ld.add_action(camera_launch)

    # AprilTag node
    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag_node',
        remappings=[
            ('image_rect/compressed', '/cam0/image_raw/compressed_2hz'),
            ('camera_info', '/cam0/camera_info'),
            ('detections', '/apriltag_detections')
        ],
        parameters=[{
            'image_transport': 'compressed',
            'tag_family': '36h11',  # Standard AprilTag family
            'tag_size': 0.1,  # Size of the tag in meters
        }],
        condition=IfCondition(apriltags)
    )
    ld.add_action(apriltag_node)

    # Image throttle node for 2fps raw images - for YOLO detection
    image_throttle_raw_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle_raw_node',
        arguments=['messages', '/cam0/image_raw', '2.0', '/cam0/image_raw/raw_2hz'],
        condition=IfCondition(camera)
    )
    ld.add_action(image_throttle_raw_node)

    # Separate throttle streams for YOLO and AprilTag to eliminate redundant throttling

    # YOLO throttle: 2 FPS for object detection
    image_throttle_yolo_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle_yolo_node',
        arguments=['messages', '/cam0/image_raw/compressed', '2.0', '/cam0/image_raw/compressed_2hz_yolo'],
        condition=IfCondition(camera)
    )
    ld.add_action(image_throttle_yolo_node)

    # AprilTag throttle: 2 FPS for tag detection
    image_throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='image_throttle_node',
        arguments=['messages', '/cam0/image_raw/compressed', '2.0', '/cam0/image_raw/compressed_2hz'],
        condition=IfCondition(camera)
    )
    ld.add_action(image_throttle_node)

    # DEXI servo controller launch file
    servo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_cpp'), 'launch', 'servo_controller.launch.py')
        ]),
        condition=IfCondition(servos)
    )
    ld.add_action(servo_launch)

    # GPIO launch file
    gpio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dexi_cpp'), 'launch', 'tca9555_controller.launch.py')
        ]),
        condition=IfCondition(gpio)
    )
    ld.add_action(gpio_launch)

    # YOLO node - optimized for Pi5
    yolo_node = Node(
        package='dexi_yolo',
        executable='dexi_yolo_node_onnx.py',
        name='dexi_yolo_node',
        remappings=[
            ('/cam0/image_raw/compressed', '/cam0/image_raw/compressed_2hz_yolo')
        ],
        parameters=[{
            'input_size': 320,           # Model trained at 320x320
            'num_threads': 1,            # Single thread to avoid CPU contention
            'detection_frequency': 2.0,  # Process 2 frames per second (matches throttle rate)
            'use_letterbox': True,       # Enable to match training preprocessing (rect=False)
            'confidence_threshold': 0.5, # Lowered from 0.65 (sigmoid fix allows proper filtering)
            'nms_threshold': 0.4,
            'verbose_logging': False,    # Disable verbose logging to save CPU
            'max_detections': 10,        # Limit max detections to reduce processing
        }],
        condition=IfCondition(yolo)
    )
    ld.add_action(yolo_node)

    # Include offboard control nodes
    offboard_manager_node = Node(
        package='dexi_offboard',
        executable='px4_offboard_manager',
        name='px4_offboard_manager',
        namespace='dexi',
        output='screen',
        parameters=[{
            'keyboard_control_enabled': keyboard_control
        }],
        condition=IfCondition(offboard)
    )
    ld.add_action(offboard_manager_node)

    keyboard_teleop_node = Node(
        package='dexi_offboard',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        namespace='dexi',
        output='screen',
        prefix='xterm -e',
        condition=IfCondition(keyboard_control)
    )
    ld.add_action(keyboard_teleop_node)

    # OAK-D optical flow node - runs from virtual environment
    # Publishes /oak/flow/twist for visual odometry
    oak_flow_node = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'source /home/dexi/oak_venv/bin/activate && python3 /home/dexi/oak_flow_node.py'
        ],
        name='oak_flow_node',
        output='screen',
        condition=IfCondition(oak_flow)
    )
    ld.add_action(oak_flow_node)

    return ld
