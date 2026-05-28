# OpenArm VR Teleoperation — How-To Guide

Step-by-step record of everything done to get IsaacSim VR teleoperation working
with robot joint feedback, camera streaming, gripper control, and a cube-stacking scene.

---

## Directory Layout (on this machine)

```
~/OPEN_ARM/
├── teleop_xr/                        ← this repo
│   ├── scripts/
│   │   ├── openarm_vr_isaacsim_feedback.py        ← IsaacSim + feedback driver
│   │   ├── openarm_vr_cube_stack_isaacsim.py      ← cube-stacking scene driver
│   │   ├── run_openarm_vr_isaacsim_feedback.sh    ← launcher (IsaacSim env)
│   │   ├── run_openarm_vr_cube_stack_isaacsim.sh  ← launcher (cube stack)
│   │   ├── run_openarm_vr_ros2_feedback.sh        ← launcher (TeleopXR IK + cameras)
│   │   └── run_openarm_vr_ros2.sh                 ← launcher (TeleopXR IK, no cameras)
│   └── teleop_xr/ros2/__main__.py   ← TeleopXR ROS2 bridge (gripper fix applied here)
│
├── quest3_streamer/
│   └── openarm_config/openarm_bimanual/
│       └── openarm_bimanual.usd      ← preferred USD (has camera prims baked in)
│
└── openarm_isaac_lab/source/openarm/openarm/tasks/manager_based/
    openarm_manipulation/usds/openarm_bimanual/
    └── openarm_bimanual.usd          ← fallback USD (no camera prims, uses fallback cameras)
```

---

## Variables to Change for a Different Machine

Every path that is machine-specific is listed here.
Search for these strings in the shell scripts and update them.

| Variable / String | Where it appears | What to change it to |
|---|---|---|
| `/home/air-lab-ncsu` | All shell scripts | Your home directory |
| `anaconda3/envs/env_isaacsim` | `run_openarm_vr_isaacsim_feedback.sh`, `run_openarm_vr_cube_stack_isaacsim.sh` | Path to your IsaacSim conda environment |
| `anaconda3/envs/teleop_xr` | `run_openarm_vr_ros2_feedback.sh`, `run_openarm_vr_ros2.sh` | Path to your teleop_xr conda environment |
| `/opt/ros/jazzy` | `run_openarm_vr_ros2_feedback.sh`, `run_openarm_vr_ros2.sh` | Your ROS2 distro path (e.g. `/opt/ros/humble`) |
| `ROS_DISTRO=jazzy` | IsaacSim launcher scripts | Your ROS2 distro name |
| `quest3_streamer/openarm_config/openarm_bimanual/openarm_bimanual.usd` | `openarm_vr_isaacsim_feedback.py`, `openarm_vr_cube_stack_isaacsim.py` | Path to your USD with camera prims |
| `openarm_isaac_lab/source/...` | Same Python files | Path to your fallback USD |
| `192.168.1.191` | Printed at startup (auto-detected) | Detected automatically — no manual change needed |
| Port `4443` | TeleopXR default | Change with `--port` flag if needed |

---

## Conda Environments

Two separate conda environments are required.

| Environment | Python | Purpose |
|---|---|---|
| `env_isaacsim` | 3.11 | Runs IsaacSim + its bundled ROS2 bridge |
| `teleop_xr` | 3.12 | Runs TeleopXR IK server + system ROS2 jazzy |

**They must NOT be mixed in the same terminal.**

---

## Problem 1 — Black Camera Panels in VR Headset

### Root Cause

The IsaacLab USD (`openarm_bimanual.usd`) had no robot-mounted camera prims.
IsaacSim only found its own editor cameras (`/OmniverseKit_Persp`, etc.), which were
ignored. TeleopXR opened the camera panels but had nothing to stream into them.

### Fix A — Use the Quest Streamer USD

The quest3_streamer USD already had three camera prims baked in:

```
/openarm/openarm_body_link/head_camera
/openarm/openarm_left_link7/left_wrist_camera
/openarm/openarm_right_link7/right_wrist_camera
```

The IsaacSim script now automatically prefers this USD if it exists:

```python
# scripts/openarm_vr_isaacsim_feedback.py
_QUEST_STREAMER_USD = "~/OPEN_ARM/quest3_streamer/openarm_config/openarm_bimanual/openarm_bimanual.usd"
_ISAAC_LAB_USD      = "~/OPEN_ARM/openarm_isaac_lab/.../openarm_bimanual.usd"
_DEFAULT_USD = _QUEST_STREAMER_USD if os.path.exists(_QUEST_STREAMER_USD) else _ISAAC_LAB_USD
```

You can override at runtime:

```bash
./scripts/run_openarm_vr_isaacsim_feedback.sh --usd /path/to/your.usd
```

### Fix B — Fallback Camera Creation

If neither USD has camera prims, the script creates them automatically at:

```
/openarm/openarm_body_link/teleop_head_camera
/openarm/openarm_left_link7/teleop_left_wrist_camera
/openarm/openarm_right_link7/teleop_right_wrist_camera
```

Suppress this with `--no-fallback-cameras` or `--no-cameras`.

---

## Problem 2 — cv_bridge / NumPy Crash (Segfault)

### Root Cause

`teleop_xr/ros2/__main__.py` imported `cv_bridge`, which was compiled against NumPy 1.x.
The `teleop_xr` conda env had NumPy 2.x installed, causing a segfault on the first
camera image callback.

### Fix

`cv_bridge` was removed from the ROS2 bridge module entirely.
Camera `sensor_msgs/Image` messages are now decoded with plain NumPy + OpenCV,
both of which are NumPy 2.x compatible.

---

## Problem 3 — Index Trigger Conflict (Robot Moved in VR Instead of Gripper)

### Root Cause

The WebXR frontend used index triggers for "distance grab" of the VR robot ghost.
This meant pressing the trigger moved the virtual robot model rather than
closing the real gripper.

### Fix — Two trigger modes available

**TeleopXR ROS2 bridge** (`teleop_xr/ros2/__main__.py`):
- Added `_apply_gripper_targets()`: maps index trigger value → OpenArm finger joints.
- Added `--no-gripper-trigger` CLI flag: when set, triggers are NOT forwarded to gripper joints, leaving them free for the VR distance-grab feature.

**WebXR frontend** (`webxr/src/xr/robot_system.ts` + `RobotSettingsPanel.tsx`):
- Distance-grab (`DistanceGrabbable` / `Interactable`) is controlled by a **Distance Grab** toggle in the Robot Settings panel (default: off).
- When enabled: point a controller at the robot ghost and pull the trigger to grab and reposition it in VR space.

**Two scripts — pick the one that matches your workflow:**

| Script | Index Trigger does | VR Distance Grab |
|---|---|---|
| `run_openarm_vr_ros2_feedback.sh` | Closes/opens real gripper joints | Available via Settings toggle |
| `run_openarm_vr_ros2_vrgrab.sh` | VR robot grab only (no gripper cmds) | Available via Settings toggle |

**Default control layout (`run_openarm_vr_ros2_feedback.sh`):**

| Input | Action |
|---|---|
| Both squeeze grips (deadman) | Enable IK teleoperation |
| Move controllers (with deadman) | Move robot end-effectors |
| Left index trigger | Close/open left gripper |
| Right index trigger | Close/open right gripper |
| Settings → Distance Grab ON | Additionally grab/move VR robot ghost |

**VR grab layout (`run_openarm_vr_ros2_vrgrab.sh`):**

| Input | Action |
|---|---|
| Both squeeze grips (deadman) | Enable IK teleoperation |
| Move controllers (with deadman) | Move robot end-effectors |
| Settings → Distance Grab ON + trigger | Grab and reposition VR robot ghost |

---

## Step-by-Step: Basic VR Teleop with Feedback + Cameras

### Terminal 1 — IsaacSim (env_isaacsim, do NOT source /opt/ros)

```bash
conda activate env_isaacsim
cd ~/OPEN_ARM/teleop_xr
./scripts/run_openarm_vr_isaacsim_feedback.sh
```

Expected output:

```
[Camera] Camera prims on stage:
  /openarm/openarm_body_link/head_camera
  /openarm/openarm_left_link7/left_wrist_camera
  /openarm/openarm_right_link7/right_wrist_camera
[Camera] Publishing head:       ... -> /camera/head/image_raw
[Camera] Publishing wrist_left: ... -> /camera/wrist_left/image_raw
[Camera] Publishing wrist_right:... -> /camera/wrist_right/image_raw
[Init] Quest browser → https://192.168.x.x:4443
```

If the USD has no cameras, you will see fallback creation instead:

```
[Camera] No robot camera prims found in USD; creating fallback cameras.
[Camera] Created fallback camera: /openarm/openarm_body_link/teleop_head_camera
...
```

### Terminal 2 — TeleopXR IK server (teleop_xr env)

```bash
conda activate teleop_xr
cd ~/OPEN_ARM/teleop_xr
./scripts/run_openarm_vr_ros2_feedback.sh
```

### Quest 3 Headset

1. Open the browser inside the Quest.
2. Navigate to `https://<HOST_IP>:4443` (printed in both terminals).
3. Accept the self-signed certificate warning.
4. Hold **both squeeze grips** simultaneously to start IK teleoperation.
5. Move controllers to move the arms.
6. Use index triggers to open/close grippers.

---

## Step-by-Step: Cube Stacking Scene

Spawns red / green / blue cubes on the table in front of the robot.

### Terminal 1 — IsaacSim

```bash
conda activate env_isaacsim
cd ~/OPEN_ARM/teleop_xr
./scripts/run_openarm_vr_cube_stack_isaacsim.sh
```

Optional arguments:

```bash
./scripts/run_openarm_vr_cube_stack_isaacsim.sh --cube-size 0.08 --cube-spacing 0.12
```

| Argument | Default | Description |
|---|---|---|
| `--cube-size` | `0.10` | Cube side length in metres |
| `--cube-spacing` | `0.10` | Distance between cube centres |
| `--usd` | auto | Override USD path |
| `--no-cameras` | off | Disable camera streaming |
| `--headless` | off | Run without GUI window |

### Terminal 2 — TeleopXR (same as basic mode)

```bash
conda activate teleop_xr
cd ~/OPEN_ARM/teleop_xr
./scripts/run_openarm_vr_ros2_feedback.sh
```

---

## Step-by-Step: VR Robot Grab Mode (Trigger moves robot ghost, not gripper)

Use this when you want to reposition the VR robot ghost by grabbing it with the
index trigger, rather than sending gripper close/open commands.

### Terminal 1 — IsaacSim (same as basic feedback mode)

```bash
conda activate env_isaacsim
cd ~/OPEN_ARM/teleop_xr
./scripts/run_openarm_vr_isaacsim_feedback.sh
```

### Terminal 2 — TeleopXR in VR-grab mode

```bash
conda activate teleop_xr
cd ~/OPEN_ARM/teleop_xr
./scripts/run_openarm_vr_ros2_vrgrab.sh
```

### In the Quest headset

1. Connect to `https://<HOST_IP>:4443`.
2. Open the **Robot Settings** panel (dashboard icon).
3. Enable the **Distance Grab** toggle.
4. Point a controller at the robot ghost and pull the **index trigger** to grab and
   drag the robot model to a new position in VR space.
5. Hold **both squeeze grips** to engage IK teleoperation as normal.

---

## Step-by-Step: Basic Teleop (No Camera Feedback)

Use this if you do not need camera streaming (lighter, faster).

### Terminal 1 — IsaacSim

```bash
conda activate env_isaacsim
cd ~/OPEN_ARM/teleop_xr
./scripts/run_openarm_vr_isaacsim_feedback.sh --no-cameras
```

### Terminal 2 — TeleopXR

```bash
conda activate teleop_xr
cd ~/OPEN_ARM/teleop_xr
./scripts/run_openarm_vr_ros2.sh
```

---

## ROS2 Topics Summary

| Topic | Direction | Message Type | Publisher | Subscriber |
|---|---|---|---|---|
| `/joint_trajectory` | IsaacSim ← TeleopXR | `JointTrajectory` | TeleopXR | IsaacSim |
| `/joint_states` | IsaacSim → TeleopXR | `JointState` | IsaacSim | TeleopXR (ghost robot) |
| `/camera/head/image_raw` | IsaacSim → TeleopXR | `sensor_msgs/Image` | IsaacSim | TeleopXR |
| `/camera/wrist_left/image_raw` | IsaacSim → TeleopXR | `sensor_msgs/Image` | IsaacSim | TeleopXR |
| `/camera/wrist_right/image_raw` | IsaacSim → TeleopXR | `sensor_msgs/Image` | IsaacSim | TeleopXR |

Both sides must use the same RMW: `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
(set automatically by the shell scripts).

---

## Camera Resolution / Frame Rate Tuning

Default: 480 × 360 px, published every 3 simulation steps.

```bash
./scripts/run_openarm_vr_isaacsim_feedback.sh \
    --camera-width 640 \
    --camera-height 480 \
    --camera-interval 1
```

Higher resolution or lower interval = higher GPU load in IsaacSim.

---

## USD Camera Prim Naming Convention

The script recognises camera prims by scanning path and name for these keywords:

| Stream | Keywords matched (case-insensitive) |
|---|---|
| `head` | `head` |
| `wrist_left` | `wrist_left`, `left_wrist`, or `left` + `camera` in prim name |
| `wrist_right` | `wrist_right`, `right_wrist`, or `right` + `camera` in prim name |

Editor cameras (`/OmniverseKit_*`) are always ignored.

---

## Troubleshooting

### Camera panels still black

1. Check IsaacSim terminal for `[Camera]` lines.
2. Confirm the correct USD is loaded (look for the `[Init] Loading USD:` line).
3. If using fallback cameras, the angle may be wrong — tune the `translation` and
   `rotation_xyz_deg` in `_add_fallback_camera_prims()` inside the script.

### Segfault in TeleopXR terminal

Likely a NumPy version mismatch in a C extension (cv_bridge, etc.).
cv_bridge has already been removed. If it recurs, check for other compiled
extensions importing NumPy in `teleop_xr/ros2/__main__.py`.

### "rclpy not available"

The IsaacSim launcher strips system ROS paths and injects the bundled bridge.
If you see this, you ran the script without the launcher (or sourced `/opt/ros` first).
Always use:

```bash
./scripts/run_openarm_vr_isaacsim_feedback.sh
```

Never `conda activate env_isaacsim && python scripts/openarm_vr_isaacsim_feedback.py` directly.

### Robot not moving

- Confirm TeleopXR IK server is running (Terminal 2).
- Hold **both** squeeze grips at the same time — one grip alone does nothing.
- Watch for `[ROS2] received cmd #1` in the IsaacSim terminal.

### IK solved but robot unchanged

The IK server may be re-sending the same solution. The message:

```
IK active, but solved config is unchanged; move controllers while holding both squeeze grips
```

means the controllers are not moving. Move the controllers physically while
holding the grips.

---

## Files Changed in This Session

| File | What changed |
|---|---|
| `scripts/openarm_vr_isaacsim_feedback.py` | Added camera discovery, fallback camera creation, `/joint_states` feedback, Quest-streamer USD preference |
| `scripts/run_openarm_vr_isaacsim_feedback.sh` | New launcher: strips system ROS, injects IsaacSim bridge |
| `scripts/openarm_vr_cube_stack_isaacsim.py` | New file: cube-stacking scene (red/green/blue cubes on table) |
| `scripts/run_openarm_vr_cube_stack_isaacsim.sh` | New launcher for cube-stacking scene |
| `scripts/run_openarm_vr_ros2_feedback.sh` | New launcher: TeleopXR IK with camera topics; triggers = gripper joints |
| `scripts/run_openarm_vr_ros2_vrgrab.sh` | New launcher: same as above but passes `--no-gripper-trigger`; triggers free for VR distance-grab |
| `teleop_xr/ros2/__main__.py` | Removed cv_bridge (NumPy crash fix); added gripper trigger mapping; added `--no-gripper-trigger` flag |
| `teleop_xr/ros2/cli.py` | Added `no_gripper_trigger: bool` field |
| `webxr/src/xr/robot_system.ts` | Restored `DistanceGrabbable` / `Interactable` distance-grab, controlled by Settings toggle |
| `webxr/src/components/dashboard/RobotSettingsPanel.tsx` | Restored Distance Grab toggle switch in Robot Settings panel |
