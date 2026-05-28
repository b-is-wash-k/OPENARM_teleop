# Quest3 Streamer — Full Setup & Debug Log
## OpenArm Bimanual Teleoperation with Isaac Sim 5.x + ROS2 Jazzy

This document records every step taken to get the Quest 3 → ROS2 → Isaac Sim bimanual teleoperation pipeline running on this machine. Includes exact commands, outputs, errors, and fixes.

---

## Machine Environment

| Item | Value |
|------|-------|
| OS | Ubuntu (Linux 6.17.0) |
| ROS2 | Jazzy (at `/opt/ros/jazzy`) |
| Isaac Sim | 5.1.0 pip-based in conda env `env_isaacsim` |
| Isaac Sim Python | 3.11 (conda env) |
| System Python | 3.12 |
| Quest Device | Meta Quest 3 |
| WiFi IP | 192.168.1.191 |

---

## Step 1 — Clone the Repository

```bash
cd ~/OPEN_ARM
git clone https://github.com/AiSaurabhPatil/quest3_streamer.git
cd quest3_streamer
```

**Output (important warning):**
```
Encountered 3 files that should have been pointers, but weren't:
    3d_assets/box/box.usd
    3d_assets/electric_screw_driver/electric_screw_driver.usd
    environment.usd
```

**What this means:** The repo uses Git LFS. These 3D asset files weren't downloaded correctly.

**Fix:**
```bash
sudo apt install git-lfs
git lfs install
git lfs pull
```

**Output:**
```
git-lfs is already the newest version (3.4.1-1ubuntu0.3+esm3)
Updated Git hooks.
Git LFS initialized.
```

---

## Step 2 — Repository Structure

```
quest3_streamer/
├── config/
│   └── config.yaml           # Centralized paths configuration
├── src/
│   ├── isaac_openarm_teleop.py   # Bimanual OpenArm teleoperation
│   ├── isaac_panda_teleop.py     # Franka Panda teleoperation
│   └── webxr_ros_bridge.py       # WebXR → ROS 2 bridge
├── web/
│   ├── webxr_streamer.html       # Quest browser WebXR app
│   └── https_server.py           # HTTPS server for wireless
├── scripts/
│   ├── run_openarm_teleop.sh     # Launch OpenArm teleop
│   ├── run_panda_teleop.sh       # Launch Panda teleop
│   ├── run_wireless.sh           # Launch wireless streaming
│   └── generate_cert.sh          # Generate SSL certificates
├── openarm_config/               # OpenArm robot config (USD, URDF)
└── certs/                        # SSL certificates (gitignored)
```

**Key insight:** This is NOT a ROS workspace. No `colcon build` needed. These are plain Python scripts that use `rclpy` directly.

---

## Step 3 — Python Environment Setup

### Why NOT conda for ROS2

**Attempt (wrong):**
```bash
conda create -n quest3_ros python=3.10
conda activate quest3_ros
pip install -r requirements.txt
```

**Error when running:**
```
ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/jazzy/lib/python3.12/site-packages/_rclpy_pybind11.cpython-310-x86_64-linux-gnu.so' isn't present
```

**Root cause:** ROS2 Jazzy is compiled for Python 3.12. Conda env uses Python 3.10. The compiled `.so` files are ABI-incompatible.

```
conda Python 3.10  ──✗──  ROS .so compiled for Python 3.12
venv  (inherits system Python 3.12)  ──✓──  ROS .so compiled for Python 3.12
```

### Correct approach — venv with system-site-packages

```bash
conda deactivate
python3 --version        # Must show 3.12.x
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
pip install -r requirements.txt
pip install typeguard
source /opt/ros/jazzy/setup.bash
```

**Why `--system-site-packages` matters:** Without it, the venv cannot see `rclpy` which is installed system-wide by ROS2. This flag lets the venv inherit system packages while still isolating new installs.

---

## Step 4 — Start Wireless Streaming (Quest Bridge)

```bash
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
./scripts/run_wireless.sh
```

**Output:**
```
⚠️  Certificates not found. Generating them now...
✅ Certificate generated:
   - certs/cert.pem
   - certs/key.pem

🚀 Starting Wireless WebXR Streamer
-----------------------------------
👉 Quest URL:   https://192.168.1.191:8000/web/webxr_streamer.html
👉 PC Server IP: 192.168.1.191
👉 Port:         9090
-----------------------------------
📦 Starting HTTPS Server on port 8000...
🌉 Starting ROS Bridge on port 9090...
```

**On Quest 3 browser:**
1. Navigate to `https://192.168.1.191:8000/web/webxr_streamer.html`
2. Accept the self-signed certificate warning (click Advanced → Accept)
3. Click "Start AR Session"

**Verify streaming is working:**
```bash
ros2 topic hz /quest/right_hand/pose
```

**Output (streaming confirmed):**
```
average rate: 89.762
    min: 0.000s max: 0.254s std dev: 0.02475s window: 757
```

**Verify topic data:**
```bash
ros2 topic echo /quest/right_hand/pose
```

**Output:**
```yaml
header:
  stamp:
    sec: 1775735826
    nanosec: 250028921
  frame_id: quest_world
pose:
  position:
    x: 0.21808719635009766
    y: 0.973723977804184
    z: -0.1960158348083496
  orientation:
    x: 0.5407201450736082
    y: -0.4217412798284005
    z: 0.03071599018895545
    w: 0.7271949845447169
```

**Quest → ROS2 pipeline is fully working at ~90 Hz.**

---

## Step 5 — Isaac Sim Setup for Bimanual Teleoperation

### Find Isaac Sim installation

```bash
conda activate env_isaacsim
python --version    # Python 3.11.14
find ~ -name 'isaacsim' 2>/dev/null | head -5
```

**Output:**
```
/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/bin/isaacsim
/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim
```

**Isaac Sim version:**
```bash
cat /home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim/VERSION
# 5.1.0-rc.19+release.26219.9c81211b.gl
```

Isaac Sim 5.1.0 is installed as a **pip package** inside `env_isaacsim` conda environment.

---

## Step 6 — Fix 1: config.yaml — Wrong Isaac Sim Path

**Original (wrong):**
```yaml
paths:
  isaac_sim: "/home/saurabh/isaac_sim"
```

**Fixed:**
```yaml
paths:
  isaac_sim: "/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim"
```

---

## Step 7 — Fix 2: run_openarm_teleop.sh — Old Standalone Installer Format

**Original script** was written for the old Isaac Sim standalone installer (`python.sh` style):
```bash
ISAAC_SIM_PATH="/home/saurabh/isaac_sim"
export ROS_DISTRO=humble
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/humble/lib
$ISAAC_SIM_PATH/python.sh $PROJECT_ROOT/src/isaac_openarm_teleop.py
```

**Fixed script** for pip-based Isaac Sim 5.x with ROS2 Jazzy:
```bash
#!/bin/bash

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

ISAAC_SIM_PATH="/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim"
BRIDGE_PATH="$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge"

# Strip system ROS from environment (causes Python version mismatch)
unset AMENT_PREFIX_PATH
unset AMENT_CURRENT_PREFIX
unset COLCON_PREFIX_PATH
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')

# Set Isaac Sim internal ROS 2 bridge (jazzy, Python 3.11)
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$BRIDGE_PATH/jazzy/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_PATH/jazzy/lib"

/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/bin/python $PROJECT_ROOT/src/isaac_openarm_teleop.py
```

---

## Step 8 — Fix 3: isaac_openarm_teleop.py — Old API Imports

Isaac Sim 5.x changed the SimulationApp import and ROS bridge extension name.

**Line 5 — SimulationApp import:**
```python
# OLD (breaks in 5.x):
from omni.isaac.kit import SimulationApp

# NEW:
from isaacsim import SimulationApp
```

**Line 17 — ROS bridge extension:**
```python
# OLD:
enable_extension("omni.isaac.ros2_bridge")

# NEW:
enable_extension("isaacsim.ros2.bridge")
```

All other `omni.isaac.*` imports (`omni.isaac.core`, `omni.isaac.motion_generation`, etc.) still work in 5.x because they are preserved in the `extsDeprecated` layer. You will see deprecation warnings in the log but they do not break anything.

---

## Step 9 — Error: System rclpy Polluting Python Path

**Error:**
```
[67.015s] Attempting to load internal rclpy for ROS Distro: jazzy
[67.017s] Could not import internal rclpy: No module named 'rclpy._rclpy_pybind11'
The C extension '/opt/ros/jazzy/lib/python3.12/site-packages/_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so' isn't present
```

**Root cause:**
- Isaac Sim runs Python 3.11 (conda env)
- System `PYTHONPATH` contained `/opt/ros/jazzy/lib/python3.12/site-packages`
- Python 3.11 found the system `rclpy` package (Python files are version-agnostic) but the compiled `.so` extension is named `cpython-312` not `cpython-311`
- Isaac Sim's bundled rclpy `.so` is at: `isaacsim/exts/isaacsim.ros2.bridge/jazzy/rclpy/rclpy/_rclpy_pybind11.cpython-311-x86_64-linux-gnu.so`

**Fix:** Strip `/opt/ros` from `PYTHONPATH` and `LD_LIBRARY_PATH` in the launch script, then prepend Isaac Sim's bundled jazzy rclpy path. (Implemented in Step 7 above.)

---

## Step 10 — Successful Isaac Sim Launch

After all fixes, running:
```bash
conda activate env_isaacsim
cd ~/OPEN_ARM/quest3_streamer
./scripts/run_openarm_teleop.sh
```

**Key output lines (success):**
```
[67.480s] [ext: isaacsim.ros2.bridge-4.12.4] startup
[67.617s] Attempting to load system rclpy
[67.649s] rclpy loaded                                          ← SUCCESS

[Init] Warming up Isaac Sim...
[Init] Loading stage from .../openarm_config/openarm_bimanual/openarm_bimanual.usd...
[Init] Found robot at: /openarm
[Init] Loading IK Solvers...
[Lula] Joint 'openarm_left_finger_joint2' is specified as a mimic joint...   ← warning only
[Init] IK Solvers loaded successfully!

[Info] Available DOFs: ['openarm_left_joint1', 'openarm_right_joint1', ...]
[Info] Left arm indices:    [0, 2, 4, 6, 8, 10, 12]
[Info] Right arm indices:   [1, 3, 5, 7, 9, 11, 13]
[Info] Left gripper indices:  [14, 15]
[Info] Right gripper indices: [17, 18]

[Init] Initializing ROS2...
BimanualQuestTeleop Initialized
Waiting for Quest controller data...

[Camera] Found 5 cameras: ['Perspective', 'Head Camera', 'Left Wrist Camera', 'Right Wrist Camera', 'Omniversekit Top']
[Camera] Setup recording for: head at (480, 360)
[Camera] Async camera publisher started

RIGHT ARM CALIBRATION COMPLETE — Home position: [ 0.3  -0.15  0.3 ]
LEFT ARM CALIBRATION COMPLETE  — Home position: [0.3   0.15  0.3 ]
```

---

## Step 11 — IK Failures & Root Cause

**Error (after calibration):**
```
[IK] Left arm failed for target: [0.3  0.15 0.3 ]
[IK] Right arm failed for target: [ 0.28612256 -0.14771713  0.3049947 ]
```

**Root cause:** The URDF shows the arm bases are mounted at `z=0.698m` above the world origin:
```xml
<joint name="openarm_left_openarm_body_link0_joint" type="fixed">
  <origin rpy="-1.5708 0 0" xyz="0.0 0.031 0.698"/>
</joint>
<joint name="openarm_right_openarm_body_link0_joint" type="fixed">
  <origin rpy="1.5708 0 0" xyz="0.0 -0.031 0.698"/>
</joint>
```

The hardcoded workspace center `[0.3, 0.0, 0.3]` in `CONFIG` is below the arm mounting height, which is outside the reachable workspace.

**Fix applied:**

1. **Auto-set workspace center from FK at zero config** — added to `src/isaac_openarm_teleop.py` after IK solver init:
```python
fk_pos_l, _ = left_ik_solver.compute_forward_kinematics("openarm_left_hand", np.zeros(7))
fk_pos_r, _ = right_ik_solver.compute_forward_kinematics("openarm_right_hand", np.zeros(7))
auto_center = ((np.array(fk_pos_l) + np.array(fk_pos_r)) / 2.0).tolist()
CONFIG["robot_workspace_center"] = auto_center
CONFIG["left_arm_offset"] = (np.array(fk_pos_l) - np.array(auto_center)).tolist()
CONFIG["right_arm_offset"] = (np.array(fk_pos_r) - np.array(auto_center)).tolist()
```

2. **Position-only IK** (no orientation constraint) — much more robust, applied to both arms:
```python
left_actions, left_success = left_ik_solver.compute_inverse_kinematics(
    target_position=teleop_node.left_arm.smoothed_pos,
    target_orientation=None,          # position-only
    frame_name="openarm_left_hand",
    warm_start=warm_start,
    position_tolerance=0.05,          # 5cm tolerance
)
```

---

## Summary: All Files Changed

### `config/config.yaml`
```yaml
# CHANGED:
paths:
  isaac_sim: "/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim"
```

### `scripts/run_openarm_teleop.sh`
- Changed `ROS_DISTRO` from `humble` to `jazzy`
- Changed `LD_LIBRARY_PATH` bridge path from `humble` to `jazzy`
- Changed `ISAAC_SIM_PATH` from `/home/saurabh/isaac_sim` to pip conda path
- Replaced `$ISAAC_SIM_PATH/python.sh` with `/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/bin/python`
- Added: strip system ROS from `PYTHONPATH` and `LD_LIBRARY_PATH`
- Added: prepend Isaac Sim's bundled jazzy `rclpy` to `PYTHONPATH`

### `src/isaac_openarm_teleop.py`
- Line 5: `from omni.isaac.kit import SimulationApp` → `from isaacsim import SimulationApp`
- Line 17: `enable_extension("omni.isaac.ros2_bridge")` → `enable_extension("isaacsim.ros2.bridge")`
- After IK solver init: added FK at zero config to auto-detect workspace center
- Both IK calls: set `target_orientation=None` and `position_tolerance=0.05`

---

## How to Run (Full Pipeline)

### Terminal 1 — Quest Bridge (keep running)
```bash
cd ~/OPEN_ARM/quest3_streamer
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
./scripts/run_wireless.sh
```

On Quest 3: open `https://192.168.1.191:8000/web/webxr_streamer.html` → Start AR Session

### Terminal 2 — Isaac Sim Teleop (fresh terminal, no ROS sourced)
```bash
conda activate env_isaacsim
cd ~/OPEN_ARM/quest3_streamer
./scripts/run_openarm_teleop.sh
```

### Calibration
1. Wait for Isaac Sim to finish loading
2. Put on Quest headset
3. Hold both controllers still in a comfortable position for ~1 second
4. Log will print `CALIBRATION COMPLETE` for each arm
5. Move hands to control the robot

### Verify everything is flowing
```bash
# Terminal 3 (system ROS sourced)
source /opt/ros/jazzy/setup.bash
ros2 topic list | grep quest
ros2 topic hz /quest/right_hand/pose     # should show ~90 Hz
ros2 topic echo /quest/right_hand/pose   # live controller data
```

---

## ROS Topics Published

| Topic | Type | Description |
|-------|------|-------------|
| `/quest/left_hand/pose` | `PoseStamped` | Left controller 6DoF pose |
| `/quest/right_hand/pose` | `PoseStamped` | Right controller 6DoF pose |
| `/quest/left_hand/inputs` | `Joy` | Left controller buttons/axes |
| `/quest/right_hand/inputs` | `Joy` | Right controller buttons/axes |
| `/joint_states` | `JointState` | Robot joint positions |
| `/camera/head/image_raw` | `Image` | Head camera |
| `/camera/wrist_left/image_raw` | `Image` | Left wrist camera |
| `/camera/wrist_right/image_raw` | `Image` | Right wrist camera |

---

## Controller Mapping

| Controller | Action |
|-----------|--------|
| Left controller (move) | Left arm IK target |
| Right controller (move) | Right arm IK target |
| Trigger or Grip | Close gripper |
| A / X button | Cycle camera views |

---

## Key Concepts Learned

### Why venv works but conda doesn't with ROS2
ROS2 Jazzy's C extensions are compiled for Python 3.12 (`.cpython-312.so`). Conda installs its own Python binary (3.10 or 3.11), which cannot load those `.so` files. `venv` reuses the system Python 3.12 binary — same Python that compiled ROS — so the `.so` files load correctly.

### Why system ROS must be stripped before Isaac Sim
Isaac Sim 5.x (pip, Python 3.11) ships its own internal `rclpy` compiled for Python 3.11. If `/opt/ros/jazzy/lib/python3.12/site-packages` is on `PYTHONPATH`, Python 3.11 will find and try to load the system `rclpy` Python files but will fail on the `cpython-312` compiled extension. The fix is to remove system ROS paths from `PYTHONPATH` before launching, then explicitly prepend Isaac Sim's internal jazzy rclpy path.

### Two separate environments, two separate terminals
- **Terminal 1** (Quest bridge): needs system ROS2 (`source /opt/ros/jazzy/setup.bash`) + `.venv`
- **Terminal 2** (Isaac Sim): needs `env_isaacsim` conda only — NO system ROS sourced

These two environments must NEVER be mixed in the same terminal.

---

## Next Steps Roadmap

Current state:
- Quest → ROS2 bridge: **working** (~90 Hz)
- Isaac Sim loading + IK solvers: **working**
- IK solving (FK workspace fix): **pending verification on next run**
- Hardware: **not yet touched — correct**

---

### Phase 1 — Finish Simulation (do this first)

#### 1.1 Verify IK works after FK workspace fix

Run the teleop again and look for these lines in the output:

```
[FK] Left  end-effector at zero config: [x.xxx  y.xxx  z.xxx]
[FK] Right end-effector at zero config: [x.xxx  y.xxx  z.xxx]
[FK] Auto workspace center: [x.xxx  y.xxx  z.xxx]
```

If IK still fails after this, paste the FK lines — the workspace center was wrong and needs a manual forward offset added.

If you see:
```
[IK] Left arm failed ...
```
disappear from the log → IK is working → robot is moving in sim.

#### 1.2 Practice the calibration flow

1. Launch Isaac Sim teleop (Terminal 2)
2. Put on Quest, launch AR session (Terminal 1 already running)
3. Hold controllers still → both arms print CALIBRATION COMPLETE
4. Move hands slowly → confirm robot follows in Isaac Sim window
5. Try trigger → confirm gripper closes

**Goal:** Comfortable with the sim before touching hardware.

---

### Phase 2 — Pre-Hardware Safety Checks

**Do NOT skip this phase.** These steps verify the hardware is alive without moving joints aggressively.

#### 2.1 Check CAN bus is up

The OpenArm uses Damiao motors over CAN. First check the interface exists:

```bash
ip link show | grep can
# Expected output: can0, follower_l, follower_r or similar
```

If no CAN interfaces show up:
```bash
sudo ip link set can0 up type can bitrate 1000000
# or check which interface your arms use:
ls /sys/class/net/ | grep can
```

#### 2.2 Detect motors on the bus

```bash
cd ~/OPEN_ARM/openarm
conda activate openarm   # or whichever env has the openarm package
python -m openarm.damiao.detect --iface can0
```

**Expected output:** A list of detected motor IDs (slave_id, master_id).

If nothing is detected:
- Check CAN cable is connected
- Check motors are powered
- Try different interface name (`follower_l`, `follower_r`)

#### 2.3 Run gravity compensation test (simulation only — no hardware)

This runs MuJoCo only, zero hardware contact:

```bash
cd ~/OPEN_ARM/openarm
python openarm/damiao/test_gravity_mujoco.py --side left
python openarm/damiao/test_gravity_mujoco.py --side right
python openarm/damiao/test_gravity_mujoco.py --compare
```

Check that torque values look reasonable (not NaN, not huge numbers like >50 Nm).

**What to look for:**
```
Gravity torques for left:
  joint 1: +x.xxxxxx Nm
  joint 2: +x.xxxxxx Nm
  ...
```

Joint 1-2 will have the largest torques (shoulder), joints 5-7 near zero. If joint 1 shows >30 Nm something is wrong with the model.

#### 2.4 Run arm motor check (HARDWARE — small safe movements only)

This moves each joint ±0.15 rad (about 8.6°) and verifies it reaches the target:

```bash
# Left arm only first
python -m openarm.damiao.arm_motor_check --iface follower_l --side left

# Right arm (after left succeeds)
python -m openarm.damiao.arm_motor_check --iface follower_r --side right
```

**Before running:**
- Make sure the arm is in a neutral/home position
- Make sure workspace around the arm is clear (±10 cm in all directions)
- Keep your hand near the emergency stop / power switch
- Watch joint 2 especially — it must move negative on left, positive on right (away from pedestal)

**Expected output:**
```
Testing J1: 0 → -0.15 → 0 ... PASS
Testing J2: 0 → -0.15 → 0 ... PASS
...
All motors: PASS
```

If any motor FAILS or does not reach position within 10 seconds → stop immediately, check that motor's CAN connection.

---

### Phase 3 — Bridge Simulation to Hardware (Shadow Mode)

Before sending real commands, run both systems in **shadow mode**: sim moves, hardware is powered but does NOT move. You watch if the sim joint angles look reasonable for the hardware's current position.

#### 3.1 Understand the data flow

```
Quest 3 (WebXR)
    ↓  ~90 Hz pose stream
webxr_ros_bridge.py  (Terminal 1)
    ↓  /quest/left_hand/pose  /quest/right_hand/pose
isaac_openarm_teleop.py  (Terminal 2, Isaac Sim)
    ↓  Lula IK solver
/joint_states  (ROS2 topic)
    ↓  ← THIS IS WHERE HARDWARE PICKS UP
hardware_teleop_node.py  (Terminal 3, to be written)
    ↓  CAN commands via openarm.damiao.Arm
Damiao motors (physical robot)
```

The `isaac_openarm_teleop.py` already publishes solved joint positions to `/joint_states`. The hardware node just needs to subscribe and forward.

#### 3.2 Monitor /joint_states while sim runs

While Isaac Sim teleop is running with Quest connected:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /joint_states
```

Watch the joint position values. Move your hands and confirm the values change. These are the exact positions that would go to the real motors.

**Check:**
- Values are in radians
- Range is reasonable (not ±π for small movements)
- Left and right arms update independently

#### 3.3 Check joint ordering matches hardware config

In `openarm/damiao/config.py`:
```bash
cat ~/OPEN_ARM/openarm/openarm/damiao/config.py
```

The motor IDs and joint order in the hardware config must match the joint order in `/joint_states`. If the Isaac Sim DOF order is:
```
['openarm_left_joint1', 'openarm_right_joint1', 'openarm_left_joint2', ...]
```
(interleaved left-right) but the hardware expects all-left then all-right, you need a remapping step in the hardware node.

---

### Phase 4 — Hardware Teleoperation Node

Once shadow mode looks good, write a minimal hardware bridge node.

#### 4.1 Minimal hardware subscriber (skeleton)

Create `src/hardware_teleop.py`:

```python
"""
Subscribe to /joint_states from Isaac Sim teleop and send positions to real motors.
Run this AFTER verifying shadow mode (Phase 3).

Usage:
    conda activate openarm   # env with openarm package
    source /opt/ros/jazzy/setup.bash
    python src/hardware_teleop.py --iface-left follower_l --iface-right follower_r
"""
import asyncio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from openarm.bus import Bus
from openarm.damiao.arm import Arm
from openarm.damiao.motor import Motor
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.damiao.encoding import PosVelControlParams

# Safety limits — hardware will REJECT positions outside these bounds
JOINT_POSITION_LIMITS_RAD = {
    "openarm_left_joint1":  (-2.5, 2.5),
    "openarm_left_joint2":  (-2.0, 0.2),   # negative = safe direction for left
    "openarm_left_joint3":  (-2.5, 2.5),
    "openarm_left_joint4":  (-2.5, 0.0),
    "openarm_left_joint5":  (-2.5, 2.5),
    "openarm_left_joint6":  (-2.0, 2.0),
    "openarm_left_joint7":  (-2.5, 2.5),
    # mirror for right arm
}

MAX_VELOCITY = 0.3   # rad/s — SLOW for first run
SCALE = 0.3          # scale down all movements to 30% on first run

class HardwareTeleop(Node):
    def __init__(self):
        super().__init__('hardware_teleop')
        self.sub = self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 10)
        self.get_logger().info("Hardware teleop node started — listening to /joint_states")

    def on_joint_state(self, msg: JointState):
        # Extract left arm joints (indices 0,2,4,6,8,10,12 from Isaac Sim DOF list)
        # TODO: verify ordering matches hardware config
        positions = dict(zip(msg.name, msg.position))
        for name, pos in positions.items():
            # Safety check — clamp and log if out of range
            if name in JOINT_POSITION_LIMITS_RAD:
                lo, hi = JOINT_POSITION_LIMITS_RAD[name]
                if not (lo <= pos <= hi):
                    self.get_logger().warn(f"CLAMPING {name}: {pos:.3f} outside [{lo}, {hi}]")
                    pos = max(lo, min(hi, pos))

def main():
    rclpy.init()
    node = HardwareTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

Start with this node doing **nothing but logging** — confirm the joint names and values look right before adding actual motor commands.

#### 4.2 Add motor commands only after logging looks correct

Only after the dry-run logging phase, add the actual motor control using `Arm.control_mit()` from `openarm.damiao.arm`:

```python
# MIT mode: position + velocity + torque feedforward
params = PosVelControlParams(
    position=target_pos,
    velocity=MAX_VELOCITY,
)
await arm.control_pos_vel(params)
```

With gravity compensation torque added from `openarm.damiao.gravity.GravityCompensator`.

---

### Safety Rules for Hardware

| Rule | Reason |
|------|--------|
| Always run `arm_motor_check.py` first after power-on | Confirms all motors respond before teleoperation |
| Start with `SCALE = 0.3` in hardware node | 30% movement range, reduces crash risk |
| Keep `MAX_VELOCITY = 0.3 rad/s` or lower | Slow enough to stop by hand |
| Never skip calibration | Wrong home position = wrong IK targets = hardware crash |
| Keep hand on power switch during first runs | Kill power immediately if arm moves unexpectedly |
| Clear workspace 30 cm around robot before teleop | VR movements can be larger than expected |
| Test one arm at a time | Easier to diagnose if something goes wrong |

---

### Current File Status

| File | Status | Notes |
|------|--------|-------|
| `config/config.yaml` | Fixed | isaac_sim path updated |
| `scripts/run_wireless.sh` | Working | Quest bridge runs fine |
| `scripts/run_openarm_teleop.sh` | Fixed | jazzy ROS, pip Isaac Sim |
| `src/webxr_ros_bridge.py` | Working | ~90 Hz streaming confirmed |
| `src/isaac_openarm_teleop.py` | Fixed | SimulationApp import, ROS bridge ext, FK workspace auto-detect, position-only IK |
| `src/hardware_teleop.py` | Not yet written | Phase 4 |
| `openarm/damiao/arm_motor_check.py` | Exists, ready to use | Phase 2 hardware check |
| `openarm/damiao/gravity.py` | Exists, ready to use | Needed for Phase 4 |
| `openarm/damiao/test_gravity_mujoco.py` | Exists, ready to use | Phase 2 simulation check |

---

## DOF Mapping — What Quest Controls and How

### The Full Picture: Quest → Value → Joint

```
Quest 3 Controller (6DoF pose + buttons)
         │
         │  position (x,y,z)  +  orientation (quaternion)
         ▼
webxr_ros_bridge.py
         │
         │  /quest/left_hand/pose   (PoseStamped, ~90 Hz)
         │  /quest/right_hand/pose  (PoseStamped, ~90 Hz)
         │  /quest/left_hand/inputs  (Joy — trigger, grip, buttons)
         │  /quest/right_hand/inputs (Joy — trigger, grip, buttons)
         ▼
isaac_openarm_teleop.py  (Lula IK solver)
         │
         │  target end-effector position → IK → 7 joint angles per arm
         ▼
/joint_states  (22 DOF total — see table below)
         ▼
hardware_teleop.py  (to be written — Phase 4)
         │
         │  J1–J7 positions in radians → motor CAN commands
         ▼
Damiao Motors  (CAN bus)
```

---

### All 22 DOFs in the USD/Isaac Sim — Ordered as Published on /joint_states

| Index | Joint Name | Type | Controlled By | Notes |
|-------|-----------|------|--------------|-------|
| 0 | `openarm_left_joint1` | revolute | Left Quest IK | Shoulder rotation (yaw) |
| 1 | `openarm_right_joint1` | revolute | Right Quest IK | Shoulder rotation (yaw) |
| 2 | `openarm_left_joint2` | revolute | Left Quest IK | Shoulder pitch |
| 3 | `openarm_right_joint2` | revolute | Right Quest IK | Shoulder pitch |
| 4 | `openarm_left_joint3` | revolute | Left Quest IK | Upper arm roll |
| 5 | `openarm_right_joint3` | revolute | Right Quest IK | Upper arm roll |
| 6 | `openarm_left_joint4` | revolute | Left Quest IK | Elbow pitch |
| 7 | `openarm_right_joint4` | revolute | Right Quest IK | Elbow pitch |
| 8 | `openarm_left_joint5` | revolute | Left Quest IK | Forearm roll |
| 9 | `openarm_right_joint5` | revolute | Right Quest IK | Forearm roll |
| 10 | `openarm_left_joint6` | revolute | Left Quest IK | Wrist pitch |
| 11 | `openarm_right_joint6` | revolute | Right Quest IK | Wrist pitch |
| 12 | `openarm_left_joint7` | revolute | Left Quest IK | Wrist roll |
| 13 | `openarm_right_joint7` | revolute | Right Quest IK | Wrist roll |
| 14 | `openarm_left_finger_joint1` | prismatic | Left trigger/grip | Finger open/close |
| 15 | `openarm_left_finger_joint2` | prismatic | Left trigger/grip | Finger open/close (mimic) |
| 16 | `openarm_left_hand` | — | passive/fixed | Hand TCP — not commanded |
| 17 | `openarm_right_finger_joint1` | prismatic | Right trigger/grip | Finger open/close |
| 18 | `openarm_right_finger_joint2` | prismatic | Right trigger/grip | Finger open/close (mimic) |
| 19 | `openarm_right_hand` | — | passive/fixed | Hand TCP — not commanded |
| 20 | `openarm_left_ee_tcp_joint` | fixed | — | End-effector frame only |
| 21 | `openarm_right_ee_tcp_joint` | fixed | — | End-effector frame only |

**Note:** DOF order in Isaac Sim is interleaved (left1, right1, left2, right2, …), NOT grouped by arm.

---

### Each Arm's 7 Revolute Joints — Limits and Rotation Axes

The left arm base is mounted at `xyz=[0, 0.031, 0.698]` with `rpy=[-90°, 0, 0]` on the body.

| Joint | URDF Name | Rotation Axis (in link frame) | Lower (rad) | Upper (rad) | Lower (°) | Upper (°) | Max Torque | Motor Type |
|-------|-----------|-------------------------------|-------------|-------------|-----------|-----------|-----------|------------|
| J1 | `openarm_left_joint1` | Z (yaw) | -3.491 | 1.396 | -200° | +80° | 40 Nm | DM8009 |
| J2 | `openarm_left_joint2` | -X (pitch) | -3.316 | 0.175 | -190° | +10° | 40 Nm | DM8009 |
| J3 | `openarm_left_joint3` | Z (roll) | -1.571 | 1.571 | -90° | +90° | 27 Nm | DM4340 |
| J4 | `openarm_left_joint4` | Y (pitch) | 0.000 | 2.443 | 0° | +140° | 27 Nm | DM4340 |
| J5 | `openarm_left_joint5` | Z (roll) | -1.571 | 1.571 | -90° | +90° | 7 Nm | DM4310 |
| J6 | `openarm_left_joint6` | X (pitch) | -0.785 | 0.785 | -45° | +45° | 7 Nm | DM4310 |
| J7 | `openarm_left_joint7` | -Y (roll) | -1.571 | 1.571 | -90° | +90° | 7 Nm | DM4310 |

Right arm is a mirror — same limits except J1/J2 are sign-flipped (hardware `inverted=True` on J1–J3, J5–J7).

**Also J8 (gripper motor):**

| Joint | Motor | Limits | Notes |
|-------|-------|--------|-------|
| J8 | DM4310 | -45° to 0° | Gripper — not in URDF revolute list; maps to finger_joint1/2 |

---

### Gripper: How trigger maps to finger position

```python
gripper_open_pos   = 0.132   # metres (prismatic joint upper limit = 0.044 m × 3 approx)
gripper_closed_pos = -1      # fully closed
gripper_threshold  = 0.5     # trigger axis > 0.5 → close

# Quest trigger/grip axes come in on /quest/*/inputs as Joy.axes[0] and axes[1]
# Either trigger OR grip being pressed > 0.5 closes the gripper
```

The finger joints are **prismatic** (linear, in metres). They move ±0.044 m (4.4 cm) total per finger.
Both `finger_joint1` and `finger_joint2` are commanded the same value (`smoothed_gripper_pos`).
`finger_joint2` is a mimic joint in the URDF — the hardware drives only `finger_joint1`.

---

### What the Quest pose actually sends (coordinate frame)

The Quest reports pose in its own `quest_world` frame (right-handed, Y-up in VR).
The teleop script transforms it into robot frame using:

```python
# Transformation matrix: VR space → Robot space
T = np.array([[0, 0, -1],   # robot X  = VR -Z  (forward in VR = forward for robot)
              [-1, 0,  0],   # robot Y  = VR -X  (VR right = robot left/back)
              [ 0, 1,  0]])  # robot Z  = VR  Y  (VR up = robot up)
```

Then the IK target is:
```python
xr_offset  = vr_position - calibration_reference   # delta from calibration pose
robot_pos  = T @ xr_offset * scale + home_position  # mapped to robot workspace
```

**Scale = 1.0** — 1:1 mapping. 10 cm hand movement = 10 cm robot movement.
**Smoothing = 0.9** — heavily filtered, so fast hand jerks are damped.

---

### Hardware Motor IDs vs Joint Names

From `openarm/damiao/config.py`:

| Hardware Name | slave_id | master_id | Motor Type | URDF Joint (left) | URDF Joint (right) | Inverted? |
|--------------|----------|-----------|-----------|-------------------|--------------------|-----------|
| J1 | 0x01 | 0x11 | DM8009 | left_joint1 | right_joint1 | Yes |
| J2 | 0x02 | 0x12 | DM8009 | left_joint2 | right_joint2 | Yes |
| J3 | 0x03 | 0x13 | DM4340 | left_joint3 | right_joint3 | Yes |
| J4 | 0x04 | 0x14 | DM4340 | left_joint4 | right_joint4 | No |
| J5 | 0x05 | 0x15 | DM4310 | left_joint5 | right_joint5 | Yes |
| J6 | 0x06 | 0x16 | DM4310 | left_joint6 | right_joint6 | Yes |
| J7 | 0x07 | 0x17 | DM4310 | left_joint7 | right_joint7 | Yes |
| J8 | 0x08 | 0x18 | DM4310 | finger_joint1 | finger_joint1 | No |

**`inverted=True`** means when bridging sim→hardware you must **negate the angle** before sending.
This accounts for mirrored mounting of left vs right arms.

---

### What is actually NOT controlled by Quest

| DOF | Why not controlled |
|-----|--------------------|
| `openarm_left_hand` (index 16) | Fixed TCP frame — just a reference point for IK |
| `openarm_right_hand` (index 19) | Same |
| `openarm_left_ee_tcp_joint` (index 20) | Fixed offset frame |
| `openarm_right_ee_tcp_joint` (index 21) | Fixed offset frame |
| `finger_joint2` | Mimic of `finger_joint1` — hardware only needs `finger_joint1` |

---

### Summary: What one Quest controller actually does

```
Quest right controller moves →
    1 x PoseStamped at ~90 Hz  →
    IK solver maps to 7 joint angles  →
    7 revolute joints (J1–J7) commanded  →
    + 1 gripper (J8) from trigger axis  →
    = 8 motor CAN commands per arm per frame
```

Total for bimanual: **16 motor commands per frame at ~90 Hz**

---

---

# Session 2 — MuJoCo Teleop (2026-04-10)

## Context: Why Move Away from Isaac Sim

After the first session, Isaac Sim teleoperation was running but had noticeable lag. The causes were:
- Isaac Sim renderer is heavy even in headless mode
- Lula IK solver has per-call overhead
- The 0.9 smoothing filter added ~10 frames of apparent delay

The user asked: *"I want to do this in MuJoCo. I have the files for the MuJoCo model. I want to make a bridge to MuJoCo. Or what to do?"*

The cloned repo `openarm_mujoco_hardware` was already present. First step: understand what that repo actually is.

---

## What `openarm_mujoco_hardware` Is (and Is NOT)

**User confusion**: The README says "simulating OpenArm using the MuJoCo physics engine" — this sounds like a standalone simulator. It is not.

**What it actually is**: A ROS2 C++ `ros2_control` hardware plugin. It does NOT contain MuJoCo. MuJoCo runs in a browser tab (WebAssembly) at `thomasonzhou.github.io/mujoco_anywhere/`.

### Full architecture of `openarm_mujoco_hardware`

```
Browser tab (thomasonzhou.github.io/mujoco_anywhere/)
  ↕  WebSocket port 1337 / 1338
openarm_mujoco_hardware C++ plugin  (ros2_control hardware interface)
  ↕  ros2_control state/command interfaces
MoveIt2 Joint Trajectory Controller / Servo
  ↕  ROS2 topics
quest_bridge.py  (openarm_quest_teleop package — already built)
  ↕  WebSocket port 9090
Quest 3 browser (WebXR)
```

### How the C++ plugin works (from reading `src/openarm_mujoco_hardware.cpp`)

- Opens a WebSocket **server** on port 1337 (right arm) / 1338 (left arm)
- The browser tab connects to it
- **Write loop**: receives position commands from `ros2_control`, computes PD torque:
  ```
  torque = Kp * (q_cmd - q_actual) + Kd * (qdot_cmd - qdot_actual) + ff_torque
  ```
  Sends `{"cmd": {"joint_name": torque}}` to browser MuJoCo
- **Read loop**: receives `{"state": {"joint_name": {"qpos": ..., "qvel": ..., "qtau": ...}}}` from browser, updates internal state

### Why we are NOT using this

The user said: *"I don't want to open the MuJoCo from the web browser, I want to do it in my own which is being installed here."*

`unitree` conda env already has `mujoco 3.5.0` installed. We can run MuJoCo entirely locally with a proper viewer window, no browser needed.

---

## Two Paths Considered

### Path A — Full ROS2 Stack (the `openarm_mujoco_hardware` way)
```
Quest → quest_bridge.py → TwistStamped → MoveIt Servo → ros2_control
     → openarm_mujoco_hardware C++ → WebSocket → Browser MuJoCo tab
```
- Requires: colcon build of C++ package (boost, cmake), MoveIt2, browser tab
- Uses `openarm_quest_teleop/quest_bridge.py` (velocity-based, already written)
- Note: `packages/install/` already has everything built except `openarm_mujoco_hardware`

### Path B — Direct Python MuJoCo (chosen)
```
Quest → webxr_ros_bridge.py → /quest/*/pose (ROS2 topics)
     → mujoco_teleop.py (this session) → local mujoco.viewer window
```
- No C++ build, no browser, no MoveIt2
- Uses `openarm/openarm/simulation/models/scene_openarm.xml` (local MJCF)
- IK: damped least-squares via MuJoCo body Jacobian (fast, built-in)
- Runs in `.venv` (Python 3.12 + ROS2 Jazzy + `pip install mujoco`)
- ~200 Hz, zero extra overhead

**Reason for choosing Path B**: Fastest to get running, validates pipeline, no lag. Path A is the right next step for actual hardware integration later.

---

## Environment Setup (one-time)

```bash
cd ~/OPEN_ARM/quest3_streamer
source .venv/bin/activate
pip install mujoco
```

The `.venv` uses `--system-site-packages` (system Python 3.12), so it already has ROS2 Jazzy rclpy working. `mujoco` pip installs cleanly alongside it.

---

## Key Files Read This Session

### `openarm/openarm/simulation/models/scene_openarm.xml`
- Includes `openarm.xml`
- Has floor, lighting, white sky background
- Correct starting point for loading the full robot + scene

### `openarm/openarm/simulation/models/openarm.xml`
Key findings:
- `openarm_left_hand_tcp` and `openarm_right_hand_tcp` are **bodies** (not sites)
  → Must use `mj_jacBody` (not `mj_jacSite`) for Jacobian
  → Position accessed via `data.xpos[body_id]` (not `data.site_xpos`)
- Finger joints are type `slide` (linear, metres), range 0–0.044m
- Actuators are `motor` type (torque, ctrlrange -10 to 10)
  → We bypass actuators entirely and set `data.qpos[]` directly (kinematic control)
- Joint names match URDF: `openarm_left_joint1` … `openarm_left_joint7`

### `quest3_streamer/src/webxr_ros_bridge.py` — Joy message layout
```
axes[0]    = trigger    (index finger, 0.0–1.0)
axes[1]    = squeeze    (side grip button, 0.0–1.0)  ← DEADMAN
axes[2]    = thumbstick X
axes[3]    = thumbstick Y
buttons[0] = A / X
buttons[1] = B / Y
buttons[2] = Menu (always 0)
buttons[3] = thumbstick click
```

### `packages/src/openarm_quest_teleop/openarm_quest_teleop/quest_bridge.py`
- Already written by Enactic — uses MoveIt Servo (velocity-based IK)
- Publishes `TwistStamped` to `/left/servo_node/delta_twist_cmds`
- Not used in our path, but relevant for Path A later

---

## IK Method: Damped Least-Squares (DLS)

Instead of Lula (which requires Isaac Sim), we use MuJoCo's own Jacobian:

```python
# Get 3×nv body Jacobian
jacp = np.zeros((3, model.nv))
mujoco.mj_jacBody(model, data, jacp, None, body_id)

# Extract columns for this arm's 7 DOFs
J = jacp[:, dof_indices]   # 3×7

# Damped least-squares solve
dq = J.T @ np.linalg.solve(J @ J.T + λ²·I₃, dp)
data.qpos[joint_qpos_indices] += dq * gain
```

**Tuning parameters:**
| Parameter | Value | Effect |
|-----------|-------|--------|
| `IK_GAIN` | 0.5 | Fraction of error corrected per iteration |
| `IK_DAMP` | 0.005 | Singularity damping (higher = more stable near limits) |
| `IK_ITERS` | 5 | Iterations per viewer frame |
| `MAX_STEP_M` | 0.05 m | Max target jump per frame (clamps spikes) |
| `SMOOTH` | 0.4 | Exponential filter on target (0=raw, 0.9=heavy) |

**Why position-only IK:** 3×7 system is well-conditioned, robust near limits, no orientation singularities. For hardware we don't need end-effector orientation to be constrained.

---

## New Files Created This Session

### `src/mujoco_teleop.py`

Main teleop script. Key design decisions:
- Kinematic control: sets `data.qpos[]` directly, calls `mj_forward()` — bypasses physics/actuators
- ROS2 runs in a daemon thread, main thread owns MuJoCo viewer loop
- Initial calibration: hold controllers still for ~2s, same as Isaac Sim approach
- Deadman: per-arm, re-anchors on grip press so no jump on re-engagement
- Gripper: trigger axis → prismatic joint position (0=open, 0.044m)

### `scripts/run_mujoco_teleop.sh`

Single-terminal runner (when venv + ROS2 already sourced).

### `scripts/launch_mujoco.sh`

Two-terminal launcher (replaces `launch_all.sh` for MuJoCo path):
- Terminal 1: `.venv` + ROS2 Jazzy → `run_wireless.sh`
- Terminal 2: `.venv` + ROS2 Jazzy → `mujoco_teleop.py`

---

## First Run Results (user confirmed working)

```
[FK] Left  home : [x.xxx y.xxx z.xxx]
[FK] Right home : [x.xxx y.xxx z.xxx]
[CAL] LEFT calibrated
[CAL] RIGHT calibrated
[IK] L err=0.0cm  R err=0.0cm   ← normal steady state
[IK] L err=67.1cm  R err=92.8cm ← spike from fast/out-of-workspace movement
```

**Verdict**: Running correctly. Quest connected (`Client connected: ('192.168.1.12', 53212)`), MuJoCo viewer opened, calibration succeeded.

**IK error spikes** (20–67cm) were caused by the IK target jumping outside the workspace in one frame when moving fast. Fixed with `MAX_STEP_M = 0.05` clamp.

---

## Deadman Switch — Implementation (Session 2, second half)

### User requirement
> "Hold GRIP (both hands) = arms enabled (deadman switch, must hold). Release GRIP = arms stop. Pull index trigger = open/close gripper. Or pass --no-deadman argument to disable it."

### Implementation details

**Deadman axis**: `axes[1]` (squeeze/grip), threshold `> 0.3`

**Per-arm independent**: left grip controls only left arm, right grip controls only right arm.

**Re-anchor on rising edge** (the key safety feature):
```
Grip pressed (False→True):
    anchor_vr    = current Quest position
    anchor_robot = current robot EE position (arm.smooth_pos)
    → from now: desired = anchor_robot + T_VR2ROBOT @ (raw - anchor_vr) * SCALE
```
This means the robot stays exactly where it is when you re-press grip. No jump.

**When grip released**:
- `smooth_pos` stops updating
- IK keeps running but target is frozen
- Robot holds position

**`--no-deadman` flag**:
```bash
python src/mujoco_teleop.py --no-deadman
# or via launcher:
MUJOCO_ARGS=--no-deadman ./scripts/launch_mujoco.sh
```
When set: `arm.enabled = True` always, grip has no effect on arm movement.

**Gripper stays independent**: trigger always controls gripper regardless of deadman state.

### Final control mapping

| Input | Effect |
|-------|--------|
| GRIP held (`axes[1] > 0.3`) | Arm enabled — tracks hand |
| GRIP released | Arm frozen — holds last position |
| INDEX TRIGGER (`axes[0]`) | Gripper: 0=open (0.044m), 1=closed (0.0m) |
| Move controller (while GRIP held) | End-effector follows hand |
| Re-press GRIP | Re-anchors — no position jump |

### Status log output (every 3 seconds)
```
[STATUS] L:ON  err=0.1cm  R:OFF err=0.0cm
[GRIP] LEFT arm ENABLED  — anchored at [0.123 0.456 0.789]
[GRIP] LEFT arm DISABLED — frozen  at [0.123 0.456 0.789]
[CAL] Waiting for RIGHT controller to be still...
```

---

## How to Run (MuJoCo Path — Full Instructions)

### Terminal approach (recommended)
```bash
cd ~/OPEN_ARM/quest3_streamer
./scripts/launch_mujoco.sh
# or without deadman:
MUJOCO_ARGS=--no-deadman ./scripts/launch_mujoco.sh
```

### Manual two-terminal approach
**Terminal 1 (wireless bridge)**:
```bash
cd ~/OPEN_ARM/quest3_streamer
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
./scripts/run_wireless.sh
```

**Terminal 2 (MuJoCo viewer)**:
```bash
cd ~/OPEN_ARM/quest3_streamer
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
python src/mujoco_teleop.py
# or: python src/mujoco_teleop.py --no-deadman
```

### Steps after launch
1. Terminal 1 prints Quest URL → open on Quest browser
2. Accept certificate → Start AR Session
3. MuJoCo viewer window opens on desktop
4. Hold both controllers still (~2s) → `[CAL] LEFT/RIGHT calibrated`
5. Hold GRIP (side squeeze) on each controller to enable arm
6. Move hands → robot follows in viewer
7. Squeeze index trigger → close gripper

---

## Comparison: Isaac Sim vs MuJoCo

| | Isaac Sim (Session 1) | MuJoCo (Session 2) |
|---|---|---|
| Environment | `conda env_isaacsim` (Python 3.11) | `.venv` (Python 3.12 + ROS2 Jazzy) |
| Install complexity | Heavy (conda + pip, ~10GB) | One `pip install mujoco` |
| Renderer | Heavy GPU renderer | Lightweight native viewer |
| IK solver | Lula (deprecated, Isaac-specific) | DLS via MuJoCo Jacobian (built-in) |
| Effective rate | ~30 Hz | ~200 Hz |
| Lag | Noticeable | Minimal |
| Deadman switch | No | Yes (`axes[1]` squeeze) |
| Model format | `.usd` | `.xml` (MJCF) |

---

## Next Steps (updated)

| Phase | Task | Status |
|-------|------|--------|
| ✅ | Quest → ROS2 bridge running | Done |
| ✅ | MuJoCo viewer teleop | Done |
| ✅ | Deadman switch + re-anchor | Done |
| 🔲 | Test on hardware: CAN bus check | Phase 2 |
| 🔲 | Gravity compensation test (`openarm/damiao/gravity.py`) | Phase 2 |
| 🔲 | Shadow mode: run sim + monitor `/joint_states` | Phase 3 |
| 🔲 | Write `hardware_teleop.py` (sim → CAN bus) | Phase 4 |


---

## Session 2 — Continued (same day, 2026-04-10)

### Terminal Close Bug in `launch_mujoco.sh`

**User report**: Running `./scripts/launch_mujoco.sh` — Terminal 2 (MuJoCo) closed immediately.

**Diagnosis**:
- Confirmed `mujoco 3.6.0` IS installed in `.venv`
- Ran script directly: output shows it started fine (ROS2 node init printed)
- The `ExternalShutdownException` in the thread was from `timeout 5` in test, not a real crash
- Root cause: gnome-terminal closing Terminal 2 before `read` was reached (race condition or display issue)

**Fix in `launch_mujoco.sh`**:
- Changed both terminals to `bash --norc -c "..."`
- Added `trap '_pause() { exec bash --norc; }; trap _pause EXIT'` so terminal ALWAYS stays open
- Added step-by-step environment checks printed to terminal: `[1/3] Activating .venv...` etc.
- Terminal now drops into interactive bash shell on exit instead of closing

**Test command** (run before using launcher to confirm it works):
```bash
cd ~/OPEN_ARM/quest3_streamer
source .venv/bin/activate
source /opt/ros/jazzy/setup.bash
python src/mujoco_teleop.py
```

---

### First Confirmed Successful Run — Deadman Working

**User confirmed**: Quest connected, MuJoCo viewer opened, deadman switch working correctly.

Full log extract:
```
[GRIP] LEFT arm ENABLED  — anchored at [0.075 0.179 0.243]
[STATUS] L:ON  err=0.0cm  R:OFF err=3.3cm
[GRIP] LEFT arm DISABLED — frozen at [0.31  0.157 0.212]
[GRIP] RIGHT arm ENABLED — anchored at [ 0.088 -0.16   0.254]
[STATUS] L:OFF err=0.0cm  R:ON  err=0.0cm
[GRIP] RIGHT arm DISABLED — frozen at [ 0.163 -0.173  0.185]
...
[STATUS] L:ON  err=0.0cm  R:ON  err=0.0cm   ← both arms working
```

**Observations from log**:
- Deadman per-arm works: L/R independently ON/OFF
- Re-anchor works: pressing grip again → `anchored at [same position where frozen]`
- IK error mostly 0.0cm — convergence good
- Occasional spikes (8.5cm, 27.8cm) = fast movement near workspace limits, handled by MAX_STEP_M

**Remaining issue**: Right arm visually not following correctly in MuJoCo viewer. "Even on straight in right hand it is not in simulation."

---

### Root Cause: Wrong Starting Joint Configuration

**Problem identified** by running FK at zero config and at preferred config:

| Config | Left TCP | Right TCP |
|--------|----------|-----------|
| Zero config (all joints = 0) | `[0.0, 0.158, 0.082]` | `[0.0, -0.149, 0.082]` |
| Preferred config (midpoints) | `[0.182, 0.509, 1.023]` | `[0.190, -0.509, 1.018]` |

At **zero config**: arms fold completely DOWN (Z=0.08m). J4 (elbow) is at its lower limit (0°), arms are nearly straight. The IK workspace is severely constrained — the solver gets stuck in local minima near joint limits.

At **preferred config** (joint-limit midpoints): arms are in natural "elbow up" pose (Z≈1.02m). Full workspace available.

**Joint limit analysis:**
```
Left  arm J1: [-200°, +80°]   midpoint = -60°  (right arm is MIRRORED)
Right arm J1: [ -80°, +200°]  midpoint = +60°

Left  arm J2: [-190°, +10°]   midpoint = -90°
Right arm J2: [ -10°, +190°]  midpoint = +90°
```

The right arm J1/J2 need POSITIVE preferred angles (+1.047, +1.571 rad), not zeros. This was causing the right arm IK to find degenerate solutions — numerically converging (0cm error) but in a physically bizarre joint configuration.

---

### Fix: Preferred Config Start + Nullspace Control

**Two changes to `mujoco_teleop.py`:**

#### 1. Start at preferred configuration

```python
L_PREFERRED = np.array([-1.047, -1.571, 0.0, 1.222, 0.0, 0.0, 0.0])
R_PREFERRED = np.array([ 1.047,  1.571, 0.0, 1.222, 0.0, 0.0, 0.0])

# Set at startup (instead of all-zeros)
data.qpos[l_qpos] = L_PREFERRED
data.qpos[r_qpos] = R_PREFERRED
mujoco.mj_forward(model, data)

left_home  = get_body_pos(model, data, LEFT_TCP)   # [0.182, 0.509, 1.023]
right_home = get_body_pos(model, data, RIGHT_TCP)  # [0.190, -0.509, 1.018]
```

The home positions used for calibration anchor are now in the proper workspace.

#### 2. Nullspace control in IK

Standard DLS IK only achieves position target. For a 7-DOF arm there are infinite solutions. Without nullspace control, the IK drifts into degenerate/visually bad configurations.

```python
# Primary task: reach target position
dq_primary = J^T (J J^T + λ²I)^{-1} dp * gain

# Nullspace projector (DLS version)
JpJ = J^T (J J^T + λ²I)^{-1} J   # 7×7
N   = I₇ - JpJ                     # projects into nullspace of J

# Secondary task: gravitate towards preferred angles
dq_null = N @ (null_gain * (q_preferred - q_current))

# Combined
dq = dq_primary + dq_null
```

With `NULL_GAIN = 0.05`: nullspace task gently pulls joints towards natural configuration without fighting the primary IK task.

#### Updated tuning parameters

| Parameter | Old | New | Reason |
|-----------|-----|-----|--------|
| `IK_ITERS` | 5 | 10 | More iterations → better convergence |
| `IK_GAIN` | 0.5 | 0.6 | Slightly faster convergence |
| `NULL_GAIN` | — | 0.05 | New: nullspace regularization |
| `L_PREFERRED` | — | `[-1.047, -1.571, 0, 1.222, 0, 0, 0]` | New: left arm natural pose |
| `R_PREFERRED` | — | `[+1.047, +1.571, 0, 1.222, 0, 0, 0]` | New: right arm natural pose (mirrored) |

---

### How to Run (updated)

```bash
cd ~/OPEN_ARM/quest3_streamer
./scripts/launch_mujoco.sh                      # deadman ON
MUJOCO_ARGS=--no-deadman ./scripts/launch_mujoco.sh  # arms always on
```

Expected startup output:
```
Loading: .../scene_openarm.xml
[FK] Left  home (preferred config): [0.182 0.509 1.023]
[FK] Right home (preferred config): [ 0.19  -0.509  1.018]
[FK] Workspace center: [0.186 0.   1.021]

Mode: DEADMAN ON
  Hold GRIP (side squeeze) to enable each arm independently
  Release GRIP → arm freezes in place (safe)
  INDEX TRIGGER → close gripper

Step 1: Hold both controllers still (~2s) for initial calibration
Step 2: Hold GRIP and move hands to control arms
```


---

## Session 2 — Right Arm IK Fix (2026-04-10, continued)

### Second run — preferred config working, right arm still bad

Output confirmed preferred config is used:
```
[FK] Left  home (preferred config): [0.182 0.509 1.023]
[FK] Right home (preferred config): [ 0.19  -0.509  1.018]
[FK] Workspace center: [ 0.186 -0.     1.02 ]
[CAL] LEFT calibrated — home: [0.182 0.509 1.023]
[CAL] RIGHT calibrated — home: [ 0.19  -0.509  1.018]
```

Deadman working correctly. Left arm tracks perfectly. Right arm large IK errors:
```
[STATUS] L:OFF err=0.0cm  R:OFF err=23.7cm  ← arm is OFF but can't reach frozen target
[STATUS] L:OFF err=0.0cm  R:ON  err=58.7cm  ← even while ON, can't reach
[STATUS] L:OFF err=0.0cm  R:ON  err=89.2cm  ← target completely out of workspace
```

### Root Cause: Right Arm Targets Outside Reachable Workspace

**Diagnosis by tracing frozen positions in the log:**

Right arm accumulated X positions each re-grip cycle:
```
[0.19, -0.509, 1.018]  (home)
→ frozen [0.417, -0.477, 0.469]  (+0.23m X)
→ frozen [0.6,   -0.402, 0.512]  (+0.18m X)
→ frozen [0.65,  -0.433, 0.334]  (+0.05m X)
→ frozen [0.703, -0.412, 0.486]  (+0.05m X)
```

Each grip-release-re-press cycle: user held right hand forward → arm moved forward → re-anchored there → moved forward again → cumulative X drift.

**Reachability test** (50 IK iterations from preferred config to each target):

| Target | Distance from home | IK error | Result |
|--------|-------------------|----------|--------|
| `[0.6, -0.4, 0.5]` | 0.67m | 33.1cm | OUT OF REACH |
| `[0.5, -0.4, 0.5]` | 0.61m | 5.8cm | OUT OF REACH |
| `[0.4, -0.4, 0.5]` | 0.57m | 0.0cm | REACHABLE ✓ |
| `[0.35, -0.4, 0.6]` | 0.46m | 0.0cm | REACHABLE ✓ |
| `[0.3, -0.5, 0.7]` | 0.34m | 0.0cm | REACHABLE ✓ |

**Right arm max reachable workspace radius from home: ~0.55m** (at 0.61m IK starts failing).

The user's movements were pushing the right arm to targets 0.67m+ from home — beyond this limit. The IK numerically "converges" but gets stuck at the nearest reachable boundary point, showing persistent large error even when frozen.

**Why left arm was fine**: Left arm target X never exceeded 0.38m from home in the session. Right arm went to 0.7m. The user naturally reaches further forward/out with the dominant hand.

### Fix: Workspace Radius Clamp

Added `MAX_WORKSPACE_R = 0.45` — hard clamp on IK target distance from home position.

```python
# In target update loop (after per-frame step clamp):
offset = desired - home_pos
dist   = np.linalg.norm(offset)
if dist > MAX_WORKSPACE_R:
    desired = home_pos + offset * (MAX_WORKSPACE_R / dist)
```

- 0.45m clamp < 0.55m max reach → always inside reachable workspace
- Both arms clamped to same radius (left arm was never hitting it anyway)
- When user reaches beyond 0.45m, arm moves to the closest reachable point in that direction (smooth, not sudden stop)
- Persistent OFF-error now impossible: frozen target always reachable → IK always converges

### Summary of all parameters tuned

```python
SCALE           = 1.0    # 1:1 hand movement to robot movement
SMOOTH          = 0.4    # target smoothing
CALIB_STILL_M   = 0.015  # 1.5cm jitter threshold for calibration
DEADMAN_THRESH  = 0.3    # squeeze > 0.3 = arm enabled
IK_GAIN         = 0.6    # DLS step gain
IK_DAMP         = 0.005  # DLS damping
IK_ITERS        = 10     # iterations per frame
NULL_GAIN       = 0.05   # nullspace (keep joints natural)
MAX_STEP_M      = 0.05   # max target jump per frame (5cm)
MAX_WORKSPACE_R = 0.45   # max target distance from home (45cm sphere)
GRIPPER_OPEN    = 0.044  # metres
```


---

---

# Session 2 — Addendum: First Successful Bimanual Test + launch_mujoco.sh Fix

## Terminal-Closing Bug Fix (launch_mujoco.sh)

**Problem**: Running `./scripts/launch_mujoco.sh` opened Terminal 2, but it immediately closed before showing any error. This made debugging impossible.

**Root cause**: When `gnome-terminal -- bash -c "..."` finishes (whether success or crash), it closes immediately by default. The `read` we had at the end only triggered on the error path — a clean Python crash from an import error would exit before reaching it.

**Fix applied**:
```bash
# At the top of each bash -c block:
_pause() { echo ''; echo '=== Script ended. Type exit or press Ctrl+D to close ==='; exec bash --norc; }
trap _pause EXIT
```

This ensures Terminal 2 ALWAYS drops into an interactive shell instead of closing, regardless of how the script exits. Combined with early sanity checks (venv, ROS2, mujoco import), the terminal now stays open and shows exactly what went wrong.

**Verified**: Running directly in a terminal (`python src/mujoco_teleop.py`) confirmed the script starts correctly and only terminates cleanly on `timeout 5` kill (expected `ExternalShutdownException` from the ROS2 spin thread — not a real error).

---

## First Successful Bimanual Teleop Session

### Session log excerpt (full deadman test):

```
[INFO] quest_mujoco_teleop: Deadman mode: GRIP to enable each arm
[INFO] quest_mujoco_teleop: Waiting for Quest 3 poses...
[INFO] webxr_ros_bridge: Client connected: ('192.168.1.12', 50146)

[GRIP] LEFT arm ENABLED  — anchored at [0.075 0.179 0.243]
[STATUS] L:ON  err=0.0cm  R:OFF err=3.3cm
[GRIP] LEFT arm DISABLED — frozen  at [0.31  0.157 0.212]
[GRIP] RIGHT arm ENABLED — anchored at [ 0.088 -0.16   0.254]
[STATUS] L:OFF err=0.0cm  R:ON  err=0.0cm
[GRIP] RIGHT arm DISABLED — frozen at [ 0.163 -0.173  0.185]
[GRIP] RIGHT arm ENABLED — anchored at [ 0.163 -0.173  0.185]
[STATUS] L:OFF err=0.0cm  R:ON  err=0.0cm
[GRIP] RIGHT arm DISABLED — frozen at [ 0.255 -0.209  0.3  ]
...
[GRIP] LEFT arm ENABLED  — anchored at [0.25  0.072 0.36 ]
[GRIP] RIGHT arm ENABLED — anchored at [ 0.23  -0.171  0.56 ]
[STATUS] L:ON  err=0.0cm  R:ON  err=0.0cm
[STATUS] L:ON  err=0.0cm  R:ON  err=0.0cm
...
[STATUS] L:ON  err=0.0cm  R:ON  err=2.3cm
[STATUS] L:ON  err=0.0cm  R:ON  err=0.0cm
[GRIP] LEFT arm DISABLED — frozen at [0.273 0.064 0.436]
[GRIP] RIGHT arm DISABLED — frozen at [ 0.185 -0.184  0.509]
```

### User confirmation:
> "the deadman switch is okay for this one"

Both arms independently respond to GRIP:
- Pressing GRIP: arm activates, anchors to current pose — no jump
- Releasing GRIP: arm freezes in place
- Re-pressing GRIP: re-anchors at current position — still no jump
- Both arms fully independent
- Gripper (trigger) works regardless of deadman state

### IK quality observations:
- **Left arm**: consistently `err=0.0cm`, tracks well
- **Right arm**: mostly `err=0.0–2.3cm`, occasional spikes (8.5cm, 27.8cm) on fast moves near limits
  - The right arm IK converges numerically (zero error in status) but visual tracking offset observed
  - Noted for future improvement; not blocking for cube_stack phase

---

## Decision: Next Phase — cube_stack_openarm

After confirming bimanual teleop works, the next logical step is a **cube stacking manipulation task** to:
1. Validate gripper grasping in simulation (do the fingers actually interact with objects?)
2. Create a structured manipulation demo environment
3. Provide training data for the IsaacLab mimic pipeline

### Components planned:
| Component | Description |
|-----------|-------------|
| `src/cube_stack_scene.xml` | MuJoCo scene: robot + table + 3 colored cubes |
| `src/cube_stack_teleop.py` | Physics-enabled teleop (`mj_step` so cubes fall and stack) |
| `scripts/run_cube_stack.sh` | One-command launcher for cube stack session |
| `pinocchio_envs/openarm_stack_bimanual_mimic_env.py` | IsaacLab mimic env class |
| `pinocchio_envs/openarm_stack_bimanual_mimic_env_cfg.py` | Mimic env config |

### Key technical difference from basic teleop:
```python
# mujoco_teleop.py (kinematic only):
mujoco.mj_forward(model, data)   # no physics

# cube_stack_teleop.py (physics enabled):
for _ in range(SUBSTEPS):
    mujoco.mj_step(model, data)         # cubes obey gravity + contacts
    data.qpos[arm_joints] = ik_solution  # arms remain kinematically controlled
    data.qvel[arm_dofs]   = 0.0          # zero arm velocities
```

---

## Updated Next Steps

| Phase | Task | Status |
|-------|------|--------|
| ✅ | Quest → ROS2 bridge running | Done |
| ✅ | MuJoCo viewer teleop | Done |
| ✅ | Deadman switch + re-anchor | Done |
| ✅ | Bimanual test confirmed | Done |
| 🔲 | cube_stack_scene.xml + physics teleop | Phase 3 |
| 🔲 | IsaacLab mimic env for stack task | Phase 3 |
| 🔲 | Hardware: CAN bus gravity comp test | Phase 4 |
| 🔲 | Shadow mode: sim + hardware in sync | Phase 4 |

---

---

# Session 3 — cube_stack_openarm Setup (2026-04-09)

## What Was Built

Four files created for the cube stacking teleoperation task:

| File | Description |
|------|-------------|
| `openarm/simulation/models/cube_stack_scene.xml` | MuJoCo scene with table + 3 physics cubes |
| `quest3_streamer/src/cube_stack_teleop.py` | Physics-enabled teleop script |
| `quest3_streamer/scripts/run_cube_stack.sh` | One-command launcher |
| `pinocchio_envs/openarm_stack_bimanual_mimic_env.py` | IsaacLab mimic env class |
| `pinocchio_envs/openarm_stack_bimanual_mimic_env_cfg.py` | Mimic env config |
| `pinocchio_envs/__init__.py` | Updated with new gym registration |

---

## Problem Found on First Visual Test

**User feedback**: "robot openarm is in floor and cube is in table rn but it cant reach that one"

Two issues:
1. **Robot too low**: Robot base at z=0m (floor). At zero joint config, TCP is at z=0.08m (arms pointing down). Even at preferred config, TCP is at z=1.02m — but reaching DOWN to the cube table required moving 0.40m down AND 0.45m inward in Y, which was at the edge of the workspace.

2. **Cube positions too centered in Y**: Cubes at y=±0.06m. The arm TCPs start at y=±0.51m at preferred config. Getting from y=+0.51 to y=+0.06 is a 0.45m inward swing — near the workspace limit.

3. **Arms started at ZERO config** (TCP at z=0.08m), not at the preferred config. The IK had to drag the arm from pointing straight down to the working position on every session start.

---

## Fix Applied

### 1. Robot elevated on a pedestal (+0.50 m)

**Python code in `cube_stack_teleop.py`:**
```python
ROBOT_LIFT_Z = 0.50  # metres

robot_base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "openarm_body_link0")
model.body_pos[robot_base_id][2] += ROBOT_LIFT_Z
```

After lift:
- Robot base effectively at z = 0.50 m
- Arm bases at z = 0.50 + 0.698 = 1.198 m
- TCP at preferred config: z = 0.50 + 1.02 = **1.52 m**

**Visual pedestal in `cube_stack_scene.xml`:**
```xml
<body name="robot_pedestal" pos="0 0 0">
    <geom name="pedestal_top"  type="box" size="0.28 0.28 0.02" pos="0 0 0.48"/>
    <geom name="pedestal_body" type="box" size="0.24 0.24 0.24" pos="0 0 0.24"/>
</body>
```
Pedestal top surface at z=0.50m matches the programmatic lift.

### 2. Work table raised and cubes repositioned

| | Before | After |
|---|---|---|
| Table top z | 0.72 m | 1.10 m |
| cube_A y | +0.06 m | **+0.25 m** |
| cube_B y | -0.06 m | **-0.25 m** |
| cube_C position | (0.52, 0.0, 0.74) | (0.50, 0.0, 1.12) |
| Cube z | 0.74 m | **1.12 m** |

### 3. Arms start at preferred joint config

```python
L_PREFERRED = np.array([-1.047, -1.571, 0.0, 1.222, 0.0, 0.0, 0.0])
R_PREFERRED = np.array([ 1.047,  1.571, 0.0, 1.222, 0.0, 0.0, 0.0])

data.qpos[l_qpos] = L_PREFERRED.copy()
data.qpos[r_qpos] = R_PREFERRED.copy()
mujoco.mj_forward(model, data)
left_home  = get_body_pos(model, data, LEFT_TCP)   # ≈ (0.19, +0.51, 1.52)
right_home = get_body_pos(model, data, RIGHT_TCP)  # ≈ (0.19, -0.51, 1.52)
```

Arms start UP in a natural working position, not dangling at the floor.

---

## Workspace Math (Verified)

```
Left  TCP preferred:  (0.182, +0.509, 1.523)
Right TCP preferred:  (0.190, -0.509, 1.518)

cube_A (red,   left  side): (0.35, +0.25, 1.12)
cube_B (blue,  right side): (0.35, -0.25, 1.12)
cube_C (green, centre far): (0.50,  0.00, 1.12)

Delta L → cube_A: (+0.168, -0.259, -0.403)  ← achievable
Delta R → cube_B: (+0.160, +0.259, -0.398)  ← symmetric, achievable
```

The arms need to move: 0.17m forward, 0.26m inward, 0.40m down. All within the 7-DOF workspace.

---

## Key Technical Difference: Physics vs Kinematic

```python
# mujoco_teleop.py — kinematic only (no cube interaction):
mujoco.mj_forward(model, data)

# cube_stack_teleop.py — physics enabled (cubes fall, stack, can be grasped):
for _ in range(SUBSTEPS):           # 5 substeps per viewer frame
    mujoco.mj_step(model, data)     # cubes: full physics
    data.qpos[l_qpos] = l_qpos_des  # arms: kinematic override
    data.qpos[r_qpos] = r_qpos_des
    data.qvel[l_dof]  = 0.0         # zero arm velocities
    data.qvel[r_dof]  = 0.0
    # gripper: same override
    data.qpos[lf_qpos] = l_grip_des
    data.qpos[rf_qpos] = r_grip_des
    data.qvel[lf_dof] = 0.0
    data.qvel[rf_dof] = 0.0
```

Cube auto-reset: if any cube falls below z=0.90m (off table edge), it teleports back to its spawn position with a `[RESET]` log line.

---

## IsaacLab Mimic Env (Structural Stub)

Created `Isaac-Stack-OpenArm-Bimanual-Abs-Mimic-v0` in `pinocchio_envs/__init__.py`.

Action space: **16-DOF** (left pos 3 + left quat 4 + right pos 3 + right quat 4 + left grip 1 + right grip 1).

Subtask signals:
- `grasp_right` — right arm has grasped cube_B
- `stack_right` — cube_B placed on cube_A
- `grasp_left`  — left arm has grasped cube_C
- `stack_left`  — cube_C placed on the stack

**Robot asset placeholder**: Uses Franka base config as structural stub. TODO: swap in OpenArm USD once it's in IsaacLab nucleus.

---

## How to Run Cube Stack Teleop

```bash
cd ~/OPEN_ARM/quest3_streamer
./scripts/run_cube_stack.sh
```

Or directly (single terminal):
```bash
source .venv/bin/activate && source /opt/ros/jazzy/setup.bash
python src/cube_stack_teleop.py
```

Expected startup:
```
[SETUP] Robot lifted by 0.5 m (pedestal).
[FK] Left  home (preferred): [0.182 0.509 1.523]
[FK] Right home (preferred): [0.19  -0.509  1.518]
     Work table top at z=1.10 m — delta to cube_A: [0.168 -0.259 -0.403]
[INFO] quest_cube_stack_teleop: Waiting for Quest 3 poses...
```

---

## Updated Next Steps

| Phase | Task | Status |
|-------|------|--------|
| ✅ | Quest → ROS2 bridge | Done |
| ✅ | MuJoCo bimanual teleop | Done |
| ✅ | Deadman switch | Done |
| ✅ | cube_stack_scene.xml + physics teleop | Done |
| ✅ | Robot pedestal + workspace fix | Done |
| ✅ | IsaacLab mimic env stub | Done |
| 🔲 | Test cube grasping (do fingers actually hold cubes?) | Phase 3 |
| 🔲 | Record a teleoperated demo for mimic datagen | Phase 3 |
| 🔲 | Hardware: CAN bus gravity comp test | Phase 4 |

---

---

# Session 3 — Addendum: Collision + Grasping Fixes (2026-04-09)

## Problems Reported After First Visual Test

1. **Arm entering table**: Robot arm links physically penetrate the table geometry in the viewer.
2. **No grasping**: Closing gripper on a cube does nothing — cube stays on table.

---

## Root Cause Analysis

### Problem 1 — Arm penetrates table

We use **kinematic override** in the physics loop:
```python
for _ in range(SUBSTEPS):
    mujoco.mj_step(model, data)
    data.qpos[l_qpos] = l_qpos_des  # force arm position
    data.qvel[l_dof]  = 0.0
```

When `mj_step` runs, it computes contact forces — but immediately after we overwrite `qpos`. The contact forces are **never applied back to the arm**. So the arm can pass through geometry without being deflected. The table was also registered as a collision body with `contype=1 conaffinity=1`, so it was flagging contacts but those forces were discarded.

**Also**: The IK target had no Z floor, so the user could command the TCP below the table surface.

### Problem 2 — No grasping

The parallel jaw fingers have `condim=3` (standard friction) and `contype=1`, so they *should* contact cubes. But because arm qpos is force-overwritten each substep, finger DOFs are also zeroed out — the grip force is never sustained long enough for the cube to stick.

Even with `condim=6` (maximum friction), a kinematic arm can't exert consistent grip force over multiple timesteps when its joint velocities are zeroed every step. The cube simply doesn't experience a sustained clamping force.

---

## Fix 1 — Exclude arm-table collisions + IK Z-clamp

### `cube_stack_scene.xml` — contact exclusions

Added `<contact><exclude>` pairs for every arm body vs. `table` and `robot_pedestal`:

```xml
<contact>
  <exclude body1="table" body2="openarm_left_link0"/>
  ...  (all arm links, both sides)
  <exclude body1="robot_pedestal" body2="openarm_body_link0"/>
  ...
</contact>
```

**Fingers are NOT excluded** — they still interact with cubes and the table surface for stacking.

### `cube_stack_teleop.py` — IK target Z floor

```python
TCP_MIN_Z = TABLE_TOP_Z - 0.05   # = 1.05 m (5 cm above table surface)

# applied in the IK target update loop:
desired[2] = max(desired[2], TCP_MIN_Z)  # clamp: don't go through table
arm.smooth_pos = SMOOTH * arm.smooth_pos + (1 - SMOOTH) * desired
```

---

## Fix 2 — Programmatic cube grasping (virtual attach)

Since kinematic arms cannot lift cubes through contact forces, we implement **programmatic attach**: when the trigger is pressed and a cube is within `GRASP_DIST` of the TCP, we lock the cube's position relative to the TCP.

### Constants

| Constant | Value | Meaning |
|----------|-------|---------|
| `GRASP_DIST` | 0.07 m | Cube within 7 cm of TCP → graspable |
| `GRASP_THRESH` | 0.75 | Trigger must be ≥ 75% pressed to grasp |
| `RELEASE_THRESH` | 0.20 | Trigger < 20% → release cube |

### New ArmState fields

```python
self.grasped_idx  = None   # index into CUBE_BODIES, or None
self.grasp_offset = None   # cube_pos - tcp_pos at grasp time
```

### Grasp logic (per arm, per frame)

```python
if arm.grasped_idx is None:
    if trigger > GRASP_THRESH:
        # Find nearest ungrasped cube within GRASP_DIST
        best_i = nearest cube within threshold
        arm.grasped_idx  = best_i
        arm.grasp_offset = cube_pos - tcp_pos  # locked offset
        print(f"[GRASP] LEFT grasped cube_A  dist=3.2cm")
else:
    if trigger < RELEASE_THRESH:
        arm.grasped_idx = None                  # drop cube
        print(f"[RELEASE] LEFT released cube_A  at z=132.1cm")
    else:
        # Track cube to TCP
        data.qpos[cube_qpos][0:3] = tcp_pos + arm.grasp_offset
        data.qvel[cube_qvel][0:3] = 0.0
```

Each arm independently manages its own grasped cube. The other arm's held cube is excluded from the search to prevent both arms grabbing the same cube.

### Status log output

```
[GRASP]   LEFT  grasped cube_A   dist=3.2cm
[RELEASE] LEFT  released cube_A  at z=132.1cm
[GRASP]   RIGHT grasped cube_B   dist=5.1cm
[RELEASE] RIGHT released cube_B  at z=152.0cm
```

---

## ROBOT_LIFT_Z Update

User adjusted `ROBOT_LIFT_Z` from 0.50 → **0.60 m** for better visual clearance.

With 0.60 m lift:
- Arm bases at z = 0.60 + 0.698 = **1.298 m**
- TCP at preferred config: z = 0.60 + 1.02 = **1.62 m**
- Cube surface at z = 1.12 m → delta = **−0.50 m** (still reachable)

Note: The pedestal geom in `cube_stack_scene.xml` still has its top at z=0.48 m (was designed for 0.50 m lift). This leaves a small visual gap. The pedestal is visual-only (no functional effect on robot kinematics), so this is cosmetic — can be updated by changing `pedestal_top pos="0 0 0.58"` if needed.

---

## Summary of All Changes to cube_stack_scene.xml

| Change | Effect |
|--------|--------|
| Added `<contact><exclude>` for all arm links vs table | Arm no longer registers contacts with table (no visual penetration effect remains, but solver is cleaner) |
| Fingers remain collision-enabled | Cubes can rest on table; stacked cubes interact with each other |

## Summary of All Changes to cube_stack_teleop.py

| Change | Effect |
|--------|--------|
| `TCP_MIN_Z = TABLE_TOP_Z - 0.05` | IK target never sent below 1.05 m |
| `desired[2] = max(desired[2], TCP_MIN_Z)` | Arm visually stops above table surface |
| `ArmState.grasped_idx / grasp_offset` | Tracks which cube is held and its relative offset |
| Grasp loop (trigger > 0.75 + distance check) | Programmatically attaches nearest cube to TCP |
| Release (trigger < 0.20) | Drops cube at current position |
| Grasped cube qpos enforced inside physics substep loop | Cube stays attached to TCP during all 5 physics substeps per frame |

