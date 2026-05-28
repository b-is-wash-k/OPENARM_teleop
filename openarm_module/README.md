# OpenArm + LeRobot Integration

Complete integration of OpenArm bimanual robots with HuggingFace LeRobot for robotic learning and teleoperation.

This repository contains a modified fork of LeRobot with OpenArm-specific enhancements for bimanual manipulation, gamepad teleoperation, and hardware calibration synchronization.

## Features

- **Bimanual OpenArm Support**: Control two OpenArm follower robots simultaneously
- **Gamepad Teleoperation**: PlayStation/Xbox controller support for intuitive joint control
- **Leader Arm Teleoperation**: Bimanual leader arm control for high-quality demonstration collection
- **Hardware Calibration Sync**: Proper integration with OpenArm's hardware calibration system
- **Velocity & Torque Support**: Full action space including position, velocity, and torque
- **Recording & Replay**: Compatible with LeRobot's dataset recording and policy training


## 🛠️ Installation

### Prerequisites

- Ubuntu 22.04 or 24.04
- Python 3.10+
- Two OpenArm follower robots with CAN bus connectivity
- PlayStation DualSense or Xbox controller
- Root access for CAN bus configuration

### Step 1: Install LeRobot Fork

This module requires the AD-SDL LeRobot fork with OpenArm support:

```bash
# In your project directory clone the LeRobot fork
git clone https://github.com/AD-SDL/lerobot.git
cd lerobot

# Install with PDM (recommended - uses lock file)
pdm install

# OR install with pip
pip install -e .

cd ..
```

### Step 2: Install System Dependencies

```bash
# Clone this repository
git clone https://github.com/AD-SDL/openarm_module.git
cd openarm_module

# Run system setup (installs OpenArm packages and configures auto CAN setup)
bash setup_system.sh
```

This script will:
- Install OpenArm system packages (`openarm-can-utils`, CLI tools)
- Configure udev rules for automatic CAN configuration on USB connection
- Set up CAN interfaces for current session

### Step 3: Install Python Dependencies

```bash
# Install with PDM
pdm install

# OR install with pip
pip install -e .
```

## Hardware Calibration

### Check Current Calibration

**IMPORTANT**: The robots are already calibrated at the factory. Only recalibrate if you suspect the zero positions are incorrect (e.g., arms don't return to proper zero position).

**To verify calibration:**
```bash
# Move arms to zero to check if calibration is correct
python scripts/move_to_zero.py
```

If the arms move to a centered, symmetric position (hanging down naturally), calibration is good. **Skip to Sync Calibration below.**

If the arms move to awkward or asymmetric positions, proceed with recalibration.

### Recalibrate Hardware (Only if needed)

**WARNING**: This will overwrite the existing calibration. Only do this if verification above failed.

```bash
# Calibrate follower right arm (can0)
openarm-can-zero-position-calibration --canport can0 --arm-side right_arm

# Calibrate follower left arm (can1)
openarm-can-zero-position-calibration --canport can1 --arm-side left_arm

# Calibrate leader right arm (can2)
openarm-can-zero-position-calibration --canport can2 --arm-side right_arm

# Calibrate leader left arm (can3)
openarm-can-zero-position-calibration --canport can3 --arm-side left_arm
```

### Sync Calibration with LeRobot

**REQUIRED**: After verifying or recalibrating hardware, sync LeRobot's calibration files:

```bash
# Delete old LeRobot calibration files (if any)
rm -rf ~/.cache/huggingface/lerobot/calibration/

# Calibrate follower arms
lerobot-calibrate \
    --robot.type=openarm_follower \
    --robot.port=can0 \
    --robot.side=right \
    --robot.id=my_openarm_follower_right

lerobot-calibrate \
    --robot.type=openarm_follower \
    --robot.port=can1 \
    --robot.side=left \
    --robot.id=my_openarm_follower_left

# Calibrate leader arms
lerobot-calibrate \
    --teleop.type=openarm_leader \
    --teleop.port=can2 \
    --teleop.id=my_openarm_leader_right

lerobot-calibrate \
    --teleop.type=openarm_leader \
    --teleop.port=can3 \
    --teleop.id=my_openarm_leader_left
```

### Verify CAN Interfaces

After plugging in USB-CAN adapters, verify they're configured:

```bash
ip link show can0 can1 can2 can3
```

All four interfaces should show as UP. If not, unplug and replug the USB-CAN adapters.

**CAN port mapping:**
- `can0` — follower right arm
- `can1` — follower left arm
- `can2` — leader right arm
- `can3` — leader left arm

### Camera Setup

USB wrist cameras are identified by physical USB port using udev symlinks. After plugging in cameras:

```bash
# Verify symlinks are created
ls -la /dev/video-wrist-*
# Expected:
# /dev/video-wrist-left -> videoX
# /dev/video-wrist-right -> videoX
```

If symlinks are missing, check the udev rules file at `/etc/udev/rules.d/99-openarm-cameras.rules`.

Get the RealSense chest camera serial number:

```bash
source ~/humanoids/lerobot_env/bin/activate
python3 -c "import pyrealsense2 as rs; ctx = rs.context(); print([d.get_info(rs.camera_info.serial_number) for d in ctx.devices])"
```

### Manual CAN Configuration (if needed)

If auto-configuration doesn't work:

```bash
openarm-can-configure-socketcan can0 -fd -b 1000000 -d 5000000
openarm-can-configure-socketcan can1 -fd -b 1000000 -d 5000000
```

## Quick Start

After installation and calibration:

```bash
# Test teleoperation
lerobot-teleoperate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --teleop.type=openarm_bi_gamepad_joints \
    --teleop.joint_velocity_scale=60.0
```

Use the gamepad to control the robots. Press Square/X to toggle between left/right arm!

## 🎮 Teleoperation

### Start Gamepad Teleoperation

```bash
lerobot-teleoperate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --teleop.type=openarm_bi_gamepad_joints \
    --teleop.joint_velocity_scale=60.0
```

### Gamepad Controls

**Important:** Use `--teleop.type=openarm_bi_gamepad_joints` for bimanual control (two arms), or `--teleop.type=openarm_gamepad_joints` for single arm control.

- **Left Stick**: Control joint 1-2
- **Right Stick**: Control joint 3-4
- **D-Pad**: Control joint 5-7
- **L1/R1**: Open/close gripper
- **PS/Xbox Button**: Return active arm to zero position
- **Square/X**: Toggle between left/right arm (bimanual mode)
- **Triangle/Y**: Print current arm position
- **Circle/B**: Exit teleoperation

### Leader Arm Teleoperation

Leader arm teleoperation produces significantly smoother demonstrations than gamepad control and is recommended for high-quality data collection.

**CAN port mapping for leader arm setup:**
- `can0/can1` — follower right/left arms
- `can2/can3` — leader right/left arms

**Set camera formats before starting:**
```bash
v4l2-ctl --device=/dev/video-wrist-left --set-fmt-video=width=640,height=480,pixelformat=MJPG
v4l2-ctl --device=/dev/video-wrist-right --set-fmt-video=width=640,height=480,pixelformat=MJPG
```

**Teleoperate with cameras:**
```bash
lerobot-teleoperate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.left_arm_config.cameras="{ \
        chest: {type: intelrealsense, serial_number_or_name: YOUR_SERIAL, width: 848, height: 480, fps: 30}, \
        wrist_left: {type: opencv, index_or_path: /dev/video-wrist-left, width: 640, height: 480, fps: 30, fourcc: MJPG} \
    }" \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --robot.right_arm_config.cameras="{ \
        wrist_right: {type: opencv, index_or_path: /dev/video-wrist-right, width: 640, height: 480, fps: 30, fourcc: MJPG} \
    }" \
    --robot.id=my_bimanual_follower \
    --teleop.type=bi_openarm_leader \
    --teleop.left_arm_config.port=can3 \
    --teleop.right_arm_config.port=can2 \
    --teleop.id=my_bimanual_leader \
    --teleop.left_arm_config.position_kp="[120,120,60,20,12,15,12,2]" \
    --teleop.left_arm_config.position_kd="[2,2,1.0,0.5,0.1,0.1,0.1,0.02]" \
    --teleop.right_arm_config.position_kp="[120,120,60,20,12,15,12,2]" \
    --teleop.right_arm_config.position_kd="[2,2,1.0,0.5,0.1,0.1,0.1,0.02]" \
    --robot.left_arm_config.position_kp="[240,240,120,40,24,31,25,5]" \
    --robot.left_arm_config.position_kd="[5,5,1.5,0.3,0.3,0.3,0.3,0.05]" \
    --robot.right_arm_config.position_kp="[240,240,120,40,24,31,25,5]" \
    --robot.right_arm_config.position_kd="[5,5,1.5,0.3,0.3,0.3,0.3,0.05]" \
    --display_data=true
```

**Tuning notes:**
- `teleop` kp/kd values control leader arm stiffness — lower values make the leader easier to backdrive
- `robot` kp/kd values control follower arm responsiveness

### Common Issues

**Controller not responding:**
- Disconnect and reconnect the controller
- Check controller is detected: `ls /dev/input/js*`
- Restart the teleoperation script

**Arms move too fast/slow:**
- Adjust `--teleop.joint_velocity_scale` (default: 60.0)
- Lower values = slower movement
- Higher values = faster movement

## Recording Demonstrations

### Gamepad Recording

```bash
export HF_HUB_OFFLINE=1

lerobot-record \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --teleop.type=openarm_bi_gamepad_joints \
    --teleop.joint_velocity_scale=60.0 \
    --dataset.repo_id=local/my_task \
    --dataset.single_task="Task description" \
    --dataset.fps=30 \
    --dataset.num_episodes=50 \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=10 \
    --dataset.push_to_hub=false
```

### Leader Arm Recording

```bash
export HF_HUB_OFFLINE=1

v4l2-ctl --device=/dev/video-wrist-left --set-fmt-video=width=640,height=480,pixelformat=MJPG
v4l2-ctl --device=/dev/video-wrist-right --set-fmt-video=width=640,height=480,pixelformat=MJPG

lerobot-record \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.left_arm_config.cameras="{ \
        chest: {type: intelrealsense, serial_number_or_name: YOUR_SERIAL, width: 848, height: 480, fps: 30, use_depth: true}, \
        wrist_left: {type: opencv, index_or_path: /dev/video-wrist-left, width: 640, height: 480, fps: 30, fourcc: MJPG} \
    }" \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --robot.right_arm_config.cameras="{ \
        wrist_right: {type: opencv, index_or_path: /dev/video-wrist-right, width: 640, height: 480, fps: 30, fourcc: MJPG} \
    }" \
    --robot.id=my_bimanual_follower \
    --teleop.type=bi_openarm_leader \
    --teleop.left_arm_config.port=can3 \
    --teleop.right_arm_config.port=can2 \
    --teleop.id=my_bimanual_leader \
    --teleop.left_arm_config.position_kp="[120,120,60,20,12,15,12,2]" \
    --teleop.left_arm_config.position_kd="[2,2,1.0,0.5,0.1,0.1,0.1,0.02]" \
    --teleop.right_arm_config.position_kp="[120,120,60,20,12,15,12,2]" \
    --teleop.right_arm_config.position_kd="[2,2,1.0,0.5,0.1,0.1,0.1,0.02]" \
    --dataset.repo_id=local/my_task \
    --dataset.single_task="Task description" \
    --dataset.fps=30 \
    --dataset.num_episodes=100 \
    --dataset.episode_time_s=40 \
    --dataset.reset_time_s=10 \
    --dataset.push_to_hub=false \
    --display_data=false
```

To resume adding episodes to an existing dataset add `--resume=true` to the command.

**Recording Parameters:**
- `--dataset.fps`: Recording framerate (30 fps recommended for leader arm)
- `--dataset.num_episodes`: Number of demonstrations to collect
- `--dataset.episode_time_s`: Maximum episode duration in seconds
- `--dataset.reset_time_s`: Time between episodes for scene reset
- `--dataset.push_to_hub`: Set to false for local-only storage
- `--resume`: Set to true to append episodes to an existing dataset

**Dataset location:** `~/.cache/huggingface/lerobot/local/`

## Replay Demonstrations

```bash
lerobot-replay \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --dataset.fps=60 \
    --dataset.repo_id=local/my_task \
    --dataset.episode=0
```

## Training Policies

Train an ACT policy on recorded demonstrations:

```bash
python lerobot/scripts/train.py \
    policy=act \
    env=bi_openarm_follower \
    dataset_repo_id=local/my_task \
    training.offline_steps=100000 \
    training.batch_size=8 \
    training.eval_freq=10000
```

## Key Modifications in LeRobot Fork

This module depends on the `lerobot` fork which contains the following modifications from upstream LeRobot:

### 1. OpenArm Follower Robot (`lerobot/src/lerobot/robots/openarm_follower/`)

**openarm_follower.py:**
- Removed `set_zero_position()` call from `connect()` to preserve hardware calibration
- Added full action space support (position, velocity, torque)
- Optimized `get_observation()` to read all motor states in one CAN refresh cycle

**config_openarm_follower.py:**
- Configured motor IDs and types for 7-DOF + gripper
- Set appropriate kp/kd gains per joint
- Defined joint limits for left/right arm configurations

### 2. Bimanual OpenArm (`src/lerobot/robots/bi_openarm_follower/`)

- Created bimanual robot wrapper for dual OpenArm control
- Synchronized action/observation spaces across both arms
- Added proper torque enable/disable on connect/disconnect

### 3. Gamepad Teleoperation (`src/lerobot/teleoperators/openarm_gamepad/`)

**openarm_teleop_gamepad.py:**
- Direct joint control mode for OpenArm
- PlayStation controller button mapping
- Return-to-zero functionality

**openarm_bi_teleop_gamepad.py:**
- Bimanual control with arm toggle
- Independent left/right arm control
- Synchronized gripper control

**gamepad_utils.py modifications:**
- Added `get_button()` method to `GamepadController`
- Enhanced button state tracking
- Improved analog stick dead zone handling

### 4. Recording System (`src/lerobot/teleoperators/gamepad/`)

**teleop_bi_gamepad_joints.py:**
- Added velocity and torque fields to action dictionary
- Fixed action registration for bimanual recording
- Proper integration with LeRobot's dataset format

## Repository Structure

```
openarm_module/
├── README.md
├── pyproject.toml                 # Python package configuration
├── scripts/
│   ├── setup_system.sh            # System setup (OpenArm packages + CAN)
│   ├── sync_calibration.py        # Sync hardware calibration with LeRobot
│   └── move_to_zero.py           # Utility to move arms to zero
├── src/
│   └── openarm_module/            # Main module code
│       ├── __init__.py
│       └── ...                    # TODO
└── tests/
```

**Note**: This module depends on the `lerobot` fork, which should be installed separately (see Installation).

## Troubleshooting

### CAN Bus Issues

```bash
# Check CAN interfaces are up
ip link show can0
ip link show can1

# Restart CAN interfaces
sudo ip link set down can0
sudo ip link set down can1
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set up can0
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set up can1

# Check for CAN errors
candump can0
candump can1
```

### Motor Communication Errors

```bash
# Test motor communication
openarm-can-motor-check --canport can0
openarm-can-motor-check --canport can1

# Check motor IDs match configuration
openarm-can-diagnosis --canport can0
```

### Calibration Issues

If arms don't return to correct zero position:

1. **Re-run hardware calibration:**
   ```bash
   openarm-can-zero-position-calibration --canport can0 --arm-side right_arm
   openarm-can-zero-position-calibration --canport can1 --arm-side left_arm
   ```

2. **Delete LeRobot calibration and re-sync:**
   ```bash
   rm -rf ~/.cache/huggingface/lerobot/calibration/
   python scripts/sync_calibration.py
   ```

3. **Verify zero position:**
   ```bash
   python scripts/move_to_zero.py
   ```

### Dataset Recording Issues

**Dataset not saving:**
- Check disk space: `df -h`
- Verify HF_HUB_OFFLINE is set: `echo $HF_HUB_OFFLINE`
- Check dataset path exists: `ls ~/.cache/huggingface/lerobot/`

**Missing velocity/torque in observations:**
- This was a bug in earlier versions
- Update to latest version from this repository
- Observations should include `.pos`, `.vel`, and `.torque` for all joints

## Documentation

- [LeRobot Documentation](https://huggingface.co/docs/lerobot)
- [OpenArm CAN Documentation](https://docs.openarm.dev/software/can)

## Contributing

Contributions are welcome! Please:

1. Fork this repository
2. Create a feature branch: `git checkout -b feature-name`
3. Make your changes and test thoroughly
4. Submit a pull request with a clear description

## License

This project inherits the Apache 2.0 license from HuggingFace LeRobot.

## Acknowledgments

- **HuggingFace LeRobot Team** for the excellent robotic learning framework
- **Enactic/OpenArm** for the robot hardware and CAN library
- **AD-SDL/Argonne National Laboratory** for supporting this integration

## Contact

For questions and support:
- Open an issue in this repository: [AD-SDL/openarm_module](https://github.com/AD-SDL/openarm_module/issues)
- Contact: Rapid Prototyping Laboratory, Argonne National Laboratory

## Related Projects

- [lerobot-rpl](https://github.com/AD-SDL/lerobot) - LeRobot fork with OpenArm support
- [LeRobot](https://github.com/huggingface/lerobot) - Original LeRobot framework
- [OpenArm CAN](https://github.com/enactic/openarm_can) - OpenArm CAN library

---

**Note**: This is a research project. Use at your own risk and always ensure safety when operating robotic systems.