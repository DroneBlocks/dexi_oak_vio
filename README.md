# DEXI Oak VIO

OAK-D Lite visual-inertial odometry for PX4 position hold on Raspberry Pi 5.

## Overview

This package provides optical flow velocity estimation using the OAK-D Lite camera's hardware feature tracker. It's designed to run on Raspberry Pi 5 alongside the DEXI platform, publishing velocity data that can be used by PX4 for improved position hold.

**Target Platform:** Raspberry Pi 5 (Debian Bookworm, ROS2 Jazzy)

## Features

- **Hardware-accelerated feature tracking** - Runs on OAK-D's Myriad X VPU, minimal CPU load (~1-2%)
- **IMU rotation compensation** - Gyroscope data removes rotation artifacts from optical flow
- **ROS2 integration** - Publishes standard `TwistWithCovarianceStamped` messages
- **Standalone node** - Direct camera connection, no depthai_ros dependency required
- **PX4 bridge** - Optional bridge to send velocity to PX4 via uXRCE-DDS

## Quick Start (Raspberry Pi 5)

### Prerequisites

```bash
# Create virtual environment for depthai
python3 -m venv ~/oak_venv
source ~/oak_venv/bin/activate
pip install depthai numpy

# Set up udev rules for OAK camera
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Running the Node

```bash
# Activate virtual environment and ROS2
source ~/oak_venv/bin/activate
source ~/dexi_ws/install/setup.bash

# Run the optical flow node
python3 ~/dexi_oak_vio/dexi_oak_vio/oak_flow_node.py
```

The node publishes to `/oak/flow/twist` (TwistWithCovarianceStamped).

### Integration with DEXI Launch System

Add to `~/.dexi-config.yaml`:

```yaml
nodes:
  oak_flow:
    enabled: true
```

The DEXI bringup launch file will automatically start the oak_flow_node when `oak_flow:=true`.

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                      OAK-D Lite Flow Node                           │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  OAK-D Lite ──┬── Mono Camera (640x400 @ 30fps)                    │
│               ├── Feature Tracker (hardware, on VPU)                │
│               └── IMU (BMI270, 200Hz gyro)                          │
│                         │                                           │
│                         ▼                                           │
│              ┌─────────────────────┐                                │
│              │   oak_flow_node.py  │                                │
│              │  - Track features   │                                │
│              │  - Compute velocity │                                │
│              │  - IMU compensation │                                │
│              └──────────┬──────────┘                                │
│                         │                                           │
│                         ▼                                           │
│              /oak/flow/twist (ROS2)                                 │
│              TwistWithCovarianceStamped                             │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

## Testing on Desktop (Mac/PC)

Test the camera and feature tracking before deploying to Pi:

```bash
cd scripts
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Basic feature tracker test
python test_feature_tracker.py

# Visual test with OpenCV display
python test_feature_visual.py

# Simulate full ROS2 pipeline (no ROS2 needed)
python test_ros2_sim.py
```

## ROS2 Package (Optional)

For full ROS2 integration with launch files:

### Installation

```bash
cd ~/dexi_ws/src
# Copy this package to your workspace

# Build
cd ~/dexi_ws
colcon build --packages-select dexi_oak_vio
source install/setup.bash
```

### Launch Files

```bash
# Feature tracker mode (recommended)
ros2 launch dexi_oak_vio feature_tracker.launch.py

# With robot_localization fusion
ros2 launch dexi_oak_vio feature_tracker.launch.py use_robot_localization:=true

# Full VIO mode with RTAB-Map (heavier compute)
ros2 launch dexi_oak_vio vio_px4.launch.py
```

## PX4 Integration

### Parameters for External Vision

| Parameter | Value | Description |
|-----------|-------|-------------|
| `EKF2_EV_CTRL` | 7 | Fuse velocity (1+2+4) |
| `EKF2_EVV_NOISE` | 0.1 | Velocity noise |
| `EKF2_EV_DELAY` | 50 | Vision delay (ms) |

### Keep ARK Flow Enabled

| Parameter | Value | Description |
|-----------|-------|-------------|
| `EKF2_OF_CTRL` | 1 | Keep optical flow on |
| `EKF2_OF_QMIN` | 1 | Quality threshold |

## Configuration

### oak_flow_node.py Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `min_features` | 10 | Minimum features for velocity estimate |
| `camera_fx/fy` | 400.0 | Focal length (adjust for calibration) |
| `flow_scale` | 1.0 | Velocity output scale |

### robot_localization.yaml

For ROS2 sensor fusion mode, edit `config/robot_localization.yaml`:
- `twist0_rejection_threshold`: Outlier rejection for Oak-D
- `twist1_rejection_threshold`: Outlier rejection for ARK Flow

## File Structure

```
dexi_oak_vio/
├── config/
│   ├── oak_d_lite.yaml          # Camera config (full VIO mode)
│   ├── rtabmap_odom.yaml        # RTAB-Map config
│   └── robot_localization.yaml  # EKF fusion config
├── dexi_oak_vio/
│   ├── __init__.py
│   ├── feature_tracker_flow.py  # Features → velocity (ROS2 node)
│   ├── oak_flow_node.py         # Standalone flow node (recommended)
│   └── px4_dds_bridge.py        # ROS2 ↔ PX4 bridge
├── launch/
│   ├── feature_tracker.launch.py
│   └── vio_px4.launch.py
├── scripts/                     # Desktop test scripts
│   ├── test_feature_tracker.py
│   ├── test_feature_visual.py
│   ├── test_ros2_sim.py
│   └── requirements.txt
├── resource/
├── package.xml
├── setup.py
└── README.md
```

## Troubleshooting

### No features detected
- Point camera at a textured surface (bookshelf, poster, carpet)
- Check USB connection: `lsusb | grep 03e7`

### Camera already in use
```bash
pkill -f oak_flow
pkill -f depthai
```

### Velocity spikes during rotation
- Verify IMU compensation is working
- Check camera mount orientation matches expected coordinate frame

### USB 2.0 speed (expected on OAK-D Lite)
The OAK-D Lite is a USB 2.0 device by design. This is normal and sufficient for optical flow.

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oak/flow/twist` | `TwistWithCovarianceStamped` | Velocity estimate |

## Hardware

- **Camera:** OAK-D Lite (USB 2.0, 480 Mbps)
- **Processor:** Raspberry Pi 5 (4GB+ recommended)
- **IMU:** BMI270 (built into OAK-D Lite)

## License

MIT
