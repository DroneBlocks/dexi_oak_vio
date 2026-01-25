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

## Installation

### Prerequisites

depthai must be installed system-wide on the Pi:

```bash
pip install depthai numpy
```

Set up udev rules for OAK camera:

```bash
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### Build

```bash
cd ~/dexi_ws/src
git clone <repo-url> dexi_oak_vio

cd ~/dexi_ws
colcon build --packages-select dexi_oak_vio
source install/setup.bash
```

### Enable in DEXI

Edit `~/.dexi-config.yaml`:

```yaml
nodes:
  oak_flow:
    enabled: true
```

Restart the service:

```bash
sudo systemctl restart dexi
```

## Flight Day Checklist

1. **Verify node is running:**
   ```bash
   pgrep -af oak_flow_node
   ```

2. **Check topic is publishing (~30 Hz):**
   ```bash
   ros2 topic hz /oak/flow/twist
   ```

3. **View in Foxglove** (from laptop):
   - Connection: **Rosbridge (ROS 1 & 2)** at `ws://<pi-ip>:9090`
   - Plot: `/oak/flow/twist.twist.twist.linear.x`

4. **Hand test:** Move camera, verify velocity direction is correct.

### Troubleshooting

```bash
# Camera in use
pkill -f oak_flow && pkill -f depthai

# Restart
sudo systemctl restart dexi

# Check USB
lsusb | grep 03e7
```

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

## Configuration

### oak_flow_node Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_fx` | 400.0 | Focal length X (pixels) |
| `camera_fy` | 400.0 | Focal length Y (pixels) |
| `min_features` | 10 | Minimum features for valid estimate |
| `enable_imu_compensation` | true | Subtract rotation from flow |

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

## Testing on Desktop (Mac/PC)

Test the camera before deploying:

```bash
cd scripts
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

python test_feature_tracker.py      # Basic test
python test_feature_visual.py       # With OpenCV display
```

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oak/flow/twist` | `TwistWithCovarianceStamped` | Velocity estimate |

## File Structure

```
dexi_oak_vio/
├── config/
│   ├── oak_d_lite.yaml          # Camera config (full VIO mode)
│   ├── rtabmap_odom.yaml        # RTAB-Map config
│   └── robot_localization.yaml  # EKF fusion config
├── dexi_oak_vio/
│   ├── __init__.py
│   ├── feature_tracker_flow.py  # Requires depthai_ros
│   ├── oak_flow_node.py         # Standalone (recommended)
│   └── px4_dds_bridge.py        # ROS2 ↔ PX4 bridge
├── launch/
│   ├── feature_tracker.launch.py
│   └── vio_px4.launch.py
├── scripts/                     # Desktop test scripts
├── resource/
├── package.xml
├── setup.py
└── README.md
```

## Hardware

- **Camera:** OAK-D Lite (USB 2.0)
- **Processor:** Raspberry Pi 5
- **IMU:** BMI270 (built into OAK-D Lite)

## License

MIT
