# DEXI Oak VIO

OAK-D Lite stereo visual-inertial odometry for PX4 EKF2 position fusion on Raspberry Pi 5.

## Overview

This package provides **stereo VIO** (Visual-Inertial Odometry) using the OAK-D Lite's stereo cameras + BMI270 IMU + hardware feature tracker. It publishes position estimates directly to PX4 EKF2 to reduce horizontal drift when flying indoors with optical flow.

It is designed to **complement** the ARK Flow sensor, not replace it:
- **ARK Flow** → velocity + altitude (high rate, low latency)
- **OAK VIO** → position (corrects drift that pure velocity integration can't)

**Target Platform:** Raspberry Pi 5 (Debian Bookworm, ROS2 Jazzy)

## Where VIO Fits: Indoor Positioning Options

| System | Type | Drift | PX4 category |
|--------|------|-------|-------------|
| ARK Flow | Relative | Fast | Optical flow |
| **OAK VIO (this package)** | **Relative** | **Slow** | **External vision** |
| AprilTags | Absolute (when visible) | None | External vision |
| UWB | Absolute | None | External vision |
| OptiTrack / Vicon | Absolute | None | External vision |
| GPS | Global | None | Global position |

VIO reduces drift compared to optical flow alone, but is still relative — position is tracked from startup, not from a fixed room reference. For drift-free absolute positioning, VIO can be combined with AprilTags or another absolute source (future work).

## VIO vs SLAM

VIO and SLAM are related but do different jobs. This package does VIO. SLAM is a potential future direction.

| | VIO (what this package does) | SLAM |
|---|---|---|
| **Purpose** | "How far have I moved?" | "Where am I on a map, and build the map as I go?" |
| **State tracked** | Current pose (6DOF) | Current pose + a map of landmarks/features |
| **Memory** | Just the last frame or two | The full map of the environment |
| **Loop closure** | No — revisiting a spot doesn't fix drift | Yes — recognizing a previously-mapped area corrects accumulated drift |
| **Drift** | Accumulates over time | Bounded when loop closures happen |
| **Compute cost** | Low (a few hundred features, one PnP solve) | High (map storage, feature matching across map, bundle adjustment) |
| **Good for** | Real-time control inputs to a flight controller | Mapping an environment, long-duration navigation |

**VIO in this package**: Matches 3D features between consecutive frames to compute how the camera moved. Simple, fast, fits in ~50% of one Pi 5 core with most work on the Myriad X VPU. Drifts linearly with distance traveled.

**SLAM on the same hardware**: Would build up a persistent map of features in the room. When the drone flies back over a previously-seen area, SLAM recognizes it and snaps the position back, correcting accumulated drift. That's called **loop closure** — and it's the thing VIO can't do.

### Could we do SLAM on OAK-D Lite + Pi 5?

Yes, it's feasible. A few open-source options exist that target exactly this kind of hardware:

- **ORB-SLAM3** — Mature C++ library, supports stereo + IMU. Known to run on Pi 5, but tuning and integration are non-trivial. Uses the CPU fairly heavily.
- **RTAB-Map** — ROS2-native, stereo + IMU mode, includes loop closure and occupancy grid mapping. Runs on Pi 4 in published benchmarks, so Pi 5 has headroom. Heavier than VIO but manageable.
- **Kimera-VIO** — Research-grade, lightweight, real-time focused. Less mature tooling.

**The tradeoff of going SLAM**:
- ✅ Drift-free position hold in a familiar room (after loop closure)
- ✅ A usable map for path planning and obstacle avoidance later
- ✅ Could reduce or eliminate the need for AprilTags
- ❌ Noticeably more CPU / RAM
- ❌ First-time-through-a-space has the same drift as VIO (no map yet)
- ❌ Harder to debug when things go wrong
- ❌ Map quality depends on texture and lighting — blank walls break it

**Recommended progression**:
1. **Phase 1 (now)** — Stereo VIO, what this package does
2. **Phase 2** — Add AprilTags for absolute corrections
3. **Phase 3 (future exploration)** — RTAB-Map or ORB-SLAM3 for map-based localization, potentially replacing tags

Each phase is a meaningful improvement and lets us validate the stack incrementally.

## Features

- **Hardware-accelerated feature tracking** - Runs on OAK-D's Myriad X VPU, minimal Pi CPU load
- **Stereo depth + PnP** - 3D features give scale, PnP solves camera motion
- **IMU rotation constraint** - Gyro provides rotation directly, PnP solves translation only
- **Stationary detection** - Zero drift when camera is not moving (accel variance test)
- **Direct PX4 publishing** - Publishes `VehicleOdometry` to `/fmu/in/vehicle_visual_odometry`
- **Origin alignment** - Locks VIO frame to EKF2 frame on first position fix

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
