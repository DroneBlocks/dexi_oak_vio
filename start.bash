#!/bin/bash

source /home/dexi/ros2_jazzy/install/setup.bash
source /home/dexi/dexi_ws/install/setup.bash

# Function to read YAML config values
get_config_value() {
    local node_name="$1"
    local param_name="$2"
    local default="$3"
    local config_file="/home/dexi/.dexi-config.yaml"

    if [ -f "$config_file" ]; then
        # Simple YAML parsing: find "nodes:" section, then find node, then get parameter value
        value=$(awk "
            /^nodes:/ { in_nodes=1; next }
            /^[a-zA-Z]/ && in_nodes { in_nodes=0 }
            in_nodes && /^  ${node_name}:/ { in_node=1; next }
            in_nodes && /^  [a-zA-Z]/ && in_node { in_node=0 }
            in_node && /^    ${param_name}:/ {
                gsub(/^    ${param_name}: */, \"\")
                gsub(/ *$/, \"\")
                gsub(/\"/, \"\")
                print
                exit
            }
        " "$config_file")
        echo "${value:-$default}"
    else
        echo "$default"
    fi
}

# Read configuration values from ~/.dexi-config.yaml
YOLO_ENABLED=$(get_config_value "yolo" "enabled" "false")
APRILTAG_ENABLED=$(get_config_value "apriltag" "enabled" "false")
CAMERA_ENABLED=$(get_config_value "camera" "enabled" "true")
CAMERA_WIDTH=$(get_config_value "camera" "width" "640")
CAMERA_HEIGHT=$(get_config_value "camera" "height" "480")
CAMERA_FORMAT=$(get_config_value "camera" "format" "XRGB8888")
CAMERA_JPEG_QUALITY=$(get_config_value "camera" "jpeg_quality" "60")
SERVO_ENABLED=$(get_config_value "servo" "enabled" "true")
GPIO_ENABLED=$(get_config_value "gpio" "enabled" "true")
OFFBOARD_ENABLED=$(get_config_value "offboard" "enabled" "false")
KEYBOARD_CONTROL_ENABLED=$(get_config_value "offboard" "keyboard_control" "false")
ROSBRIDGE_ENABLED=$(get_config_value "rosbridge" "enabled" "true")
OAK_FLOW_ENABLED=$(get_config_value "oak_flow" "enabled" "false")

# Detect hardware type from device tree model
HARDWARE_MODEL=$(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo "unknown")

echo "Configuration loaded: yolo=$YOLO_ENABLED, apriltags=$APRILTAG_ENABLED, camera=$CAMERA_ENABLED (${CAMERA_WIDTH}x${CAMERA_HEIGHT}, $CAMERA_FORMAT, q$CAMERA_JPEG_QUALITY), servos=$SERVO_ENABLED, gpio=$GPIO_ENABLED, offboard=$OFFBOARD_ENABLED, keyboard_control=$KEYBOARD_CONTROL_ENABLED, rosbridge=$ROSBRIDGE_ENABLED, oak_flow=$OAK_FLOW_ENABLED"

if [[ $HARDWARE_MODEL == *"Raspberry Pi Compute Module 4"* ]]; then
    echo "Detected CM4 hardware, launching dexi_bringup_ark_cm4.launch.py"
    ros2 launch dexi_bringup dexi_bringup_ark_cm4.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED camera_width:=$CAMERA_WIDTH camera_height:=$CAMERA_HEIGHT camera_format:=$CAMERA_FORMAT camera_jpeg_quality:=$CAMERA_JPEG_QUALITY offboard:=$OFFBOARD_ENABLED keyboard_control:=$KEYBOARD_CONTROL_ENABLED rosbridge:=$ROSBRIDGE_ENABLED
elif [[ $HARDWARE_MODEL == *"Raspberry Pi Compute Module 5"* ]]; then
    echo "Detected CM5 hardware, launching dexi_bringup_cm5.launch.py"
    ros2 launch dexi_bringup dexi_bringup_cm5.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED camera_width:=$CAMERA_WIDTH camera_height:=$CAMERA_HEIGHT camera_format:=$CAMERA_FORMAT camera_jpeg_quality:=$CAMERA_JPEG_QUALITY gpio:=$GPIO_ENABLED servos:=$SERVO_ENABLED offboard:=$OFFBOARD_ENABLED keyboard_control:=$KEYBOARD_CONTROL_ENABLED rosbridge:=$ROSBRIDGE_ENABLED
elif [[ $HARDWARE_MODEL == *"Raspberry Pi 5"* ]]; then
    echo "Detected Pi5 hardware, launching dexi_bringup_pi5.launch.py"
    ros2 launch dexi_bringup dexi_bringup_pi5.launch.py yolo:=$YOLO_ENABLED apriltags:=$APRILTAG_ENABLED camera:=$CAMERA_ENABLED camera_width:=$CAMERA_WIDTH camera_height:=$CAMERA_HEIGHT camera_format:=$CAMERA_FORMAT camera_jpeg_quality:=$CAMERA_JPEG_QUALITY gpio:=$GPIO_ENABLED servos:=$SERVO_ENABLED offboard:=$OFFBOARD_ENABLED keyboard_control:=$KEYBOARD_CONTROL_ENABLED rosbridge:=$ROSBRIDGE_ENABLED oak_flow:=$OAK_FLOW_ENABLED
else
    echo "Unknown hardware: $HARDWARE_MODEL - no launch file specified for this platform"
fi
