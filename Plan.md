# OpenARM Bimanual VR Teleoperation Plan
## Meta Quest → Bimanual OpenARM (No Leader/Follower Hardware)

**Date:** 2026-03-29
**Setup:** Bimanual OpenARM (two arms, CAN bus), Meta Quest headset, no physical leader/follower system
**Goal:** Teleoperate both arms in real-time using Meta Quest hand tracking / controller poses

---

## Overview

Since you do NOT have a leader/follower hardware system, the VR headset becomes the "virtual leader." The Meta Quest tracks your hands (or controllers) in 3D space, and those poses are mapped → IK → joint angles → sent to both OpenARM arms via CAN bus.

**Architecture:**

```
Meta Quest (hand tracking / 6DOF poses)
        │  OpenXR protocol
        ▼
  PC (Ubuntu, Oculus Link / Air Link)
        │  OpenXR runtime
        ▼
  IsaacLab OpenXR device  ──OR──  pyopenxr bridge
        │  Retargeted joint angles (IK solved)
        ▼
  CAN Bus (openarm Python library)
        │  left: can0   right: can1
        ▼
  Bimanual OpenARM hardware
```

Two implementation paths are described below. **Path A (IsaacLab + OpenXR)** is recommended because the OpenXR infrastructure and arm retargeters already exist in your IsaacLab install.

---

## Phase 0: Prerequisites & Inventory

### 0.1 Hardware Requirements
- [x] Bimanual OpenARM (two arms assembled)
- [x] Meta Quest (Quest 2 / 3 / Pro — any OpenXR-capable model)
- [x] Ubuntu workstation with GPU (NVIDIA for IsaacLab)
- [x] Two USB-CAN adapters (Canable-style, Candlelight firmware) — one per arm
- [ ] USB-C cable or Wi-Fi 6 router for Oculus Link / Air Link

### 0.2 Software Already on Machine
- [x] `/home/air-lab-ncsu/OPEN_ARM/openarm` — Python CAN control library
- [x] `/home/air-lab-ncsu/IsaacLab/` — Isaac Lab with OpenXR support
- [ ] `openarm_description` — URDF/xacro (needs cloning)
- [ ] `openarm_ros2` — ROS2 nodes (needed for Path B)
- [ ] `openarm_teleop` — Bilateral teleop package (needed for Path B)
- [ ] `openarm_isaac_lab` — Isaac Lab extension for OpenARM (recommended)

### 0.3 Repositories to Clone

```bash
cd ~/OPEN_ARM

# OpenARM URDF description (needed for IK and simulation)
git clone https://github.com/enactic/openarm_description.git

# Isaac Lab extension for OpenARM (simulation tasks + environments)
git clone https://github.com/enactic/openarm_isaac_lab.git

# CAN library (C++ with Python bindings — optional, Python openarm already works)
git clone https://github.com/enactic/openarm_can.git

# ROS2 integration (needed only for Path B / ROS2 approach)
git clone https://github.com/enactic/openarm_ros2.git

# Teleoperation package (C++ unilateral/bilateral — needed for Path B)
git clone https://github.com/enactic/openarm_teleop.git
```

---

## Phase 1: CAN Bus Verification (Both Arms)

Before anything VR-related, confirm both arms are controllable.

### 1.1 Set Up Persistent CAN Device Names (udev)

Create udev rules so left arm is always `can0` and right arm always `can1`:

```bash
# Find device serial numbers
udevadm info /dev/ttyACM0 | grep SERIAL

# Create rule file
sudo nano /etc/udev/rules.d/99-openarm.rules
```

Add:
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="<LEFT_SERIAL>",  SYMLINK+="robot_l"
SUBSYSTEM=="usb", ATTRS{idVendor}=="1d50", ATTRS{idProduct}=="606f", ATTRS{serial}=="<RIGHT_SERIAL>", SYMLINK+="robot_r"
```

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

### 1.2 Bring Up CAN Interfaces

```bash
# Bring up both CAN buses
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# Verify both arms respond
cd ~/OPEN_ARM/openarm
python -m openarm.damiao.monitor --port robot_l:left --port robot_r:right
```

### 1.3 Gravity Compensation Test

```bash
# Enable gravity compensation on both arms (they float freely)
python -m openarm.damiao.monitor --port robot_l:left --port robot_r:right --gravity
```

Both arms should resist gravity smoothly. If they do, motor communication is verified.

---

## Phase 2: Meta Quest Setup (PC Link)

### 2.1 Install Oculus PC App (for Oculus Link / Air Link)

```bash
# On Windows side or via Steam on Linux
# Install: https://www.meta.com/help/quest/articles/headsets-and-accessories/oculus-link/connect-link-with-quest-2/
```

**Recommended: Air Link** (no cable needed, Wi-Fi 6 router required)

On Quest headset:
1. Settings → Experimental Features → Air Link → Enable
2. Connect to same Wi-Fi as PC
3. Select your PC from the Air Link list

### 2.2 Install OpenXR Runtime on PC

```bash
# NVIDIA + Linux: use SteamVR as OpenXR runtime
# Or use Monado (open-source OpenXR runtime)

# Option A: SteamVR (easier, requires Steam)
sudo apt install steam
# Launch SteamVR, set as default OpenXR runtime

# Option B: Monado (open-source)
sudo apt install monado-service
sudo systemctl enable monado
```

### 2.3 Verify OpenXR Works

```bash
# Test with hello_xr sample
git clone https://github.com/KhronosGroup/OpenXR-SDK-Source
cd OpenXR-SDK-Source && mkdir build && cd build
cmake .. && make
./src/tests/hello_xr/hello_xr -g Vulkan
```

You should see a simple scene rendered in the Quest.

---

## Phase 3A: IsaacLab OpenXR Path (Recommended)

This uses the existing OpenXR infrastructure in IsaacLab + OpenARM Isaac Lab extension.

### 3A.1 Install openarm_isaac_lab Extension

```bash
cd ~/OPEN_ARM/openarm_isaac_lab

# Install as Isaac Lab extension
python -m pip install -e .

# Or install via Isaac Lab extension manager
cd ~/IsaacLab
python source/standalone/tools/install_standalone_extension.py \
    ~/OPEN_ARM/openarm_isaac_lab
```

### 3A.2 Configure Bimanual OpenARM Scene

The openarm_isaac_lab likely has single-arm environments. You need a bimanual scene.

Check what environments exist:
```bash
ls ~/OPEN_ARM/openarm_isaac_lab/
grep -r "bimanual\|dual_arm\|two_arm" ~/OPEN_ARM/openarm_isaac_lab/ --include="*.py" -l
```

If no bimanual env exists, create `bimanual_env_cfg.py`:

```python
# ~/OPEN_ARM/openarm_isaac_lab/openarm_isaac_lab/envs/bimanual_teleop_env.py
import isaaclab.sim as sim_utils
from isaaclab.assets import ArticulationCfg
from isaaclab.envs import DirectRLEnvCfg
from isaaclab.scene import InteractiveSceneCfg

# Load the URDF for both arms
LEFT_ARM_CFG = ArticulationCfg(
    prim_path="/World/left_arm",
    spawn=sim_utils.UrdfFileCfg(
        asset_path="~/OPEN_ARM/openarm_description/urdf/openarm_left.urdf",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.3, 0.0),    # left arm position in world
    ),
)

RIGHT_ARM_CFG = ArticulationCfg(
    prim_path="/World/right_arm",
    spawn=sim_utils.UrdfFileCfg(
        asset_path="~/OPEN_ARM/openarm_description/urdf/openarm_right.urdf",
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, -0.3, 0.0),   # right arm position in world
    ),
)
```

### 3A.3 Launch VR Teleoperation with Hand Tracking

```bash
cd ~/IsaacLab

# Launch with OpenXR kit (this enables VR rendering)
python scripts/environments/teleoperation/teleop_se3_agent.py \
    --task OpenARM-Bimanual-IK-Abs-v0 \
    --teleop_device handtracking \
    --kit_file apps/isaaclab.python.xr.openxr.kit \
    --num_envs 1
```

**What this does:**
- IsaacLab renders the simulation into the Quest headset
- OpenXR reads hand joint positions (26 joints per hand)
- Retargeter maps wrist + finger poses → robot end-effector targets
- IK solver computes 8 joint angles per arm
- Simulation executes the motion

### 3A.4 Add Custom OpenARM Retargeter

The existing retargeters are for Unitree/Fourier robots. You need one for OpenARM's 8-DOF kinematics.

Create `~/OPEN_ARM/openarm_isaac_lab/openarm_isaac_lab/devices/openarm_retargeter.py`:

```python
from isaaclab.devices.openxr.retargeters import RetargeterBase
import numpy as np
from ikpy.chain import Chain

class OpenARMBimanualRetargeter(RetargeterBase):
    """Maps Meta Quest hand tracking → OpenARM bimanual joint angles via IK."""

    def __init__(self, urdf_left: str, urdf_right: str):
        # Load IK chains from URDF
        self.chain_left  = Chain.from_urdf_file(urdf_left,  base_elements=["base_link"])
        self.chain_right = Chain.from_urdf_file(urdf_right, base_elements=["base_link"])

        # Workspace scaling: Quest space → robot workspace
        self.scale = 1.0          # 1:1 scale (meters)
        self.workspace_offset_left  = np.array([0.0,  0.3, 0.0])
        self.workspace_offset_right = np.array([0.0, -0.3, 0.0])

    def retarget(self, hand_data: dict) -> dict:
        """
        hand_data: {
            "left":  {"wrist": [x,y,z,qw,qx,qy,qz], ...},
            "right": {"wrist": [x,y,z,qw,qx,qy,qz], ...},
        }
        Returns: {"left": [q0..q7], "right": [q0..q7]}
        """
        left_wrist  = hand_data["left"]["wrist"][:3]
        right_wrist = hand_data["right"]["wrist"][:3]

        # Solve IK for each arm
        q_left  = self.chain_left.inverse_kinematics(
            left_wrist + self.workspace_offset_left)
        q_right = self.chain_right.inverse_kinematics(
            right_wrist + self.workspace_offset_right)

        # Apply joint limits from motor specs (radians)
        q_left  = self._clip_joints(q_left)
        q_right = self._clip_joints(q_right)

        return {"left": q_left[1:-1], "right": q_right[1:-1]}  # strip base/tip

    def _clip_joints(self, q: np.ndarray) -> np.ndarray:
        # OpenARM joint limits (approximate, verify from hardware docs)
        limits = [
            (-3.14, 3.14),   # J1 shoulder rotation
            (-1.57, 1.57),   # J2 shoulder pitch
            (-3.14, 3.14),   # J3 elbow
            (-1.57, 1.57),   # J4 forearm
            (-1.57, 1.57),   # J5 wrist pitch
            (-1.57, 1.57),   # J6 wrist roll
            (-1.57, 1.57),   # J7 wrist yaw
            (-0.05, 0.08),   # J8 gripper
        ]
        for i, (lo, hi) in enumerate(limits):
            q[i+1] = np.clip(q[i+1], lo, hi)
        return q
```

### 3A.5 Sim-to-Real Bridge

After verifying motion in simulation, add real robot execution:

```python
# ~/OPEN_ARM/openarm_isaac_lab/scripts/vr_teleop_real.py
"""
Runs IsaacLab simulation with VR input, then mirrors joint angles
to the real OpenARM hardware via CAN bus.
"""
import asyncio
import threading
from openarm.damiao.arm import Arm

# Initialize real robot arms
arm_left  = Arm(port="robot_l", side="left")
arm_right = Arm(port="robot_r", side="right")

arm_left.enable()
arm_right.enable()

def send_to_robot(q_left, q_right, velocity=3.0):
    """Send joint angle targets to both real arms."""
    for i, angle in enumerate(q_left):
        arm_left.motors[i].set_position(angle, velocity=velocity)
    for i, angle in enumerate(q_right):
        arm_right.motors[i].set_position(angle, velocity=velocity)

# Hook this into the IsaacLab step callback
# Call send_to_robot() every simulation step with the retargeted angles
```

---

## Phase 3B: Direct OpenXR → CAN Path (Lightweight, No Isaac)

If you want to skip IsaacLab simulation and go directly from Quest to robot:

### 3B.1 Install pyopenxr

```bash
pip install pyopenxr
```

### 3B.2 Create Direct VR-to-Robot Node

```python
# ~/OPEN_ARM/openarm/examples/vr_teleop_direct.py
"""
Direct Meta Quest hand tracking → OpenARM bimanual control.
No simulation. Pure OpenXR → IK → CAN.
"""
import xr          # pyopenxr
import numpy as np
from ikpy.chain import Chain
from openarm.damiao.arm import Arm

# Robot setup
arm_left  = Arm(port="robot_l", side="left")
arm_right = Arm(port="robot_r", side="right")
arm_left.enable()
arm_right.enable()

# IK chains
chain_left  = Chain.from_urdf_file("~/OPEN_ARM/openarm_description/urdf/openarm.urdf")
chain_right = Chain.from_urdf_file("~/OPEN_ARM/openarm_description/urdf/openarm.urdf")

# OpenXR context
instance = xr.create_instance(
    xr.InstanceCreateInfo(
        enabled_extension_names=[
            xr.EXT_HAND_TRACKING_EXTENSION_NAME,
            xr.KHR_OPENGL_ENABLE_EXTENSION_NAME,
        ]
    )
)

def get_hand_pose(session, hand: str) -> np.ndarray:
    """Returns wrist position [x, y, z] from OpenXR hand tracking."""
    # ... (OpenXR hand tracking API calls)
    # Returns 3D wrist position in meters
    pass

def main_loop():
    # ... OpenXR session setup ...
    while running:
        # 1. Get wrist poses from Quest
        left_pos  = get_hand_pose(session, "left")
        right_pos = get_hand_pose(session, "right")

        # 2. Solve IK
        q_left  = chain_left.inverse_kinematics(left_pos)
        q_right = chain_right.inverse_kinematics(right_pos)

        # 3. Send to robot
        for i in range(8):
            arm_left.motors[i].set_position(q_left[i+1],  velocity=5.0)
            arm_right.motors[i].set_position(q_right[i+1], velocity=5.0)

        time.sleep(0.02)  # 50 Hz control loop
```

---

## Phase 4: Safety & Control Layer

**CRITICAL — implement before running on real hardware.**

### 4.1 Emergency Stop

```python
def emergency_stop():
    """Immediately disable all motors."""
    arm_left.disable()
    arm_right.disable()

# Map to Quest controller button (e.g., A button = E-stop)
# Map to keyboard: press 'Q' at any time
import signal
signal.signal(signal.SIGINT, lambda s, f: emergency_stop())
```

### 4.2 Velocity Limiting

```python
MAX_JOINT_VELOCITY = 3.0  # rad/s — keep low initially, increase gradually
MAX_DELTA_PER_STEP = 0.05  # radians — max change per control step

def safe_position_command(current_q, target_q):
    """Clamp step size to avoid jerky motion."""
    delta = np.clip(target_q - current_q, -MAX_DELTA_PER_STEP, MAX_DELTA_PER_STEP)
    return current_q + delta
```

### 4.3 Workspace Bounding Box

```python
# Define safe Cartesian workspace (meters, robot base frame)
WORKSPACE = {
    "left":  {"x": (0.1, 0.7), "y": (0.0, 0.6), "z": (-0.3, 0.5)},
    "right": {"x": (0.1, 0.7), "y": (-0.6, 0.0), "z": (-0.3, 0.5)},
}

def in_workspace(pos: np.ndarray, side: str) -> bool:
    ws = WORKSPACE[side]
    return (ws["x"][0] <= pos[0] <= ws["x"][1] and
            ws["y"][0] <= pos[1] <= ws["y"][1] and
            ws["z"][0] <= pos[2] <= ws["z"][1])
```

### 4.4 Gripper Mapping

```python
def map_pinch_to_gripper(hand_data: dict, side: str) -> float:
    """
    Map thumb-index pinch distance to gripper opening.
    Returns: 0.0 = fully closed, 0.08 = fully open (meters)
    """
    thumb_tip = hand_data[side]["thumb_3"][:3]
    index_tip = hand_data[side]["index_3"][:3]
    pinch_dist = np.linalg.norm(thumb_tip - index_tip)
    # Pinch range: ~0.01m (closed) to ~0.10m (open)
    return np.clip((pinch_dist - 0.01) / 0.09, 0.0, 1.0) * 0.08
```

---

## Phase 5: Calibration

### 5.1 Reference Pose Calibration

Before teleoperation, perform a T-pose or home calibration:

```
Procedure:
1. Move both robot arms to home position (all joints = 0)
2. Put on Quest, hold hands in front of you in T-pose
3. Press calibration button (e.g., Quest menu button)
4. This establishes the mapping: your hand positions → robot home
```

### 5.2 Workspace Scaling

Test with conservative 1:1 mapping first. If the robot can't reach your full range of motion, apply a scaling factor:

```python
# If robot workspace is smaller than your arm reach:
SCALE = 0.7  # 70% of your motion = 100% of robot workspace
```

### 5.3 Latency Check

```bash
# Measure end-to-end latency (VR pose → robot motion)
# Target: < 50ms for comfortable teleoperation
# Method: wave hand rapidly and observe robot response delay
python ~/OPEN_ARM/openarm/examples/latency_test.py
```

---

## Phase 6: Testing Sequence

**Follow this order — do NOT skip steps.**

| Step | Test | Pass Condition |
|------|------|----------------|
| 1 | CAN bus + both arms detected | Monitor shows all 16 motors |
| 2 | Gravity compensation on both arms | Arms float freely, no drift |
| 3 | OpenXR hand tracking in simulation | Hands visible in IsaacLab viewer |
| 4 | IK solving in simulation | Sim arms follow hand motion |
| 5 | Single arm real robot (left, slow speed) | Physical arm follows sim |
| 6 | Single arm full speed | Smooth motion, no jitter |
| 7 | Bimanual (both arms, slow speed) | Both arms respond simultaneously |
| 8 | Gripper open/close via pinch | Gripper actuates correctly |
| 9 | E-stop verification | Both arms halt instantly |
| 10 | Full bimanual manipulation task | Pick and place with both hands |

---

## Recommended File Structure

```
~/OPEN_ARM/
├── README.md
├── Plan.md                          ← this file
├── openarm/                         ← Python CAN library (existing)
├── openarm_description/             ← URDF files (clone)
├── openarm_isaac_lab/               ← Isaac Lab extension (clone)
├── openarm_ros2/                    ← ROS2 packages (clone, Path B only)
├── openarm_teleop/                  ← Teleop C++ packages (clone, Path B only)
└── scripts/
    ├── vr_teleop_isaaclab.sh        ← launch script for Path A
    ├── vr_teleop_direct.sh          ← launch script for Path B
    └── setup_can.sh                 ← CAN bus bringup script
```

---

## Quick Start (TL;DR — after all setup)

```bash
# 1. Bring up CAN buses
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can1 up type can bitrate 1000000

# 2. Connect Meta Quest via Air Link (or USB cable)

# 3. Launch VR teleoperation (Path A - IsaacLab)
cd ~/IsaacLab
python scripts/environments/teleoperation/teleop_se3_agent.py \
    --task OpenARM-Bimanual-Teleop-v0 \
    --teleop_device handtracking \
    --kit_file apps/isaaclab.python.xr.openxr.kit

# 4. Perform calibration T-pose when prompted

# 5. Teleoperate! E-stop: press A button on right Quest controller
```

---

## Known Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| Arms collide with each other | Implement inter-arm collision avoidance in IK |
| IK singularity causes jerky motion | Add null-space damping; velocity limiting |
| VR latency causes oscillation | Reduce control frequency; add low-pass filter |
| CAN message overflow | Use async CAN with priority queuing |
| Quest tracking lost = arm freezes | Timeout handler: hold last position on tracking loss |
| Motor overheating | Add current monitoring; thermal shutdown |

---

## References

- [openarm_teleop docs](https://docs.openarm.dev/teleop/)
- [openarm_ros2 docs](https://docs.openarm.dev/software/ros2/install)
- [IsaacLab OpenXR docs](https://isaac-lab.github.io)
- [openarm_isaac_lab](https://docs.openarm.dev/simulation/isaac-lab)
- [pyopenxr library](https://github.com/cmbruns/pyopenxr)
- [ikpy inverse kinematics](https://github.com/Phylliade/ikpy)
- [OpenXR hand tracking spec](https://www.khronos.org/openxr/)
- IsaacLab OpenXR device: `~/IsaacLab/source/isaaclab/isaaclab/devices/openxr/openxr_device.py`
- OPEN_ARM CAN monitor: `~/OPEN_ARM/openarm/openarm/damiao/monitor.py`
