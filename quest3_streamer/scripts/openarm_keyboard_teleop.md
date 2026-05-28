# OpenArm Keyboard Teleoperation — Dev Log

**Date:** 2026-04-15
**Environment:** Isaac Sim 5.x (env_isaacsim conda), OpenArm Bimanual USD

---

## Prompt

> "I want to run only OpenArm in Isaac Sim and then open it but by using the keyboard to control the end effector — like the `Se3Keyboard` from IsaacLab (`isaaclab.devices.keyboard.se3_keyboard`) which was used with Franka. At first let's only launch OpenArm and then try to run it with the keyboard without any task, just making sure we can do this."

---

## What Was Built

Two new files:

| File | Purpose |
|------|---------|
| `src/isaac_openarm_keyboard_teleop.py` | Main Python script — loads the robot, registers keyboard, runs IK loop |
| `scripts/run_openarm_keyboard.sh` | Shell launcher — strips system ROS, calls the script |

---

## How to Run

```bash
cd /home/air-lab-ncsu/OPEN_ARM/quest3_streamer
bash scripts/run_openarm_keyboard.sh
```

No ROS, no Quest headset required. Just Isaac Sim + keyboard.

---

## Input

### Runtime keyboard bindings (focus must be on the Isaac Sim window)

| Key | Action |
|-----|--------|
| `W` / `S` | Move end-effector forward / backward (X axis) |
| `A` / `D` | Move end-effector left / right (Y axis) |
| `Q` / `E` | Move end-effector up / down (Z axis) |
| `Z` / `X` | Roll end-effector + / − |
| `T` / `G` | Pitch end-effector + / − |
| `C` / `V` | Yaw end-effector + / − |
| `K` | Toggle gripper open / close |
| `L` | Reset active arm to home position |
| `Tab` | Switch which arm the keyboard controls (LEFT ↔ RIGHT) |

### Configuration constants (top of script)

| Constant | Value | Meaning |
|----------|-------|---------|
| `POS_SPEED` | 0.004 m/step | How fast EE moves per sim frame while key held |
| `ROT_SPEED` | 0.008 rad/step | Rotation speed per frame |
| `GRIPPER_SPEED` | 0.05 pos/step | Gripper open/close rate |
| `GRIPPER_OPEN` | 0.132 | Fully-open joint position |
| `GRIPPER_CLOSED` | -1.0 | Fully-closed joint position |

---

## Output (terminal on startup)

```
[Init] Warming up Isaac Sim...
[Init] Loading USD: .../openarm_bimanual/openarm_bimanual.usd
[Init] Robot prim: /openarm
[Init] Loading Lula IK solvers...
[Init] IK solvers ready.
[Init] Keyboard registered.
[Info] DOFs (22): ['openarm_left_joint1', 'openarm_right_joint1', ...]
[Info] Left  arm indices : [0, 2, 4, 6, 8, 10, 12]
[Info] Right arm indices : [1, 3, 5, 7, 9, 11, 13]
[Info] Left  grip indices: [14, 15]
[Info] Right grip indices: [17, 18]
[FK] Left  home EE : [0.    0.153 0.162]
[FK] Right home EE : [ 0.   -0.153  0.162]

============================================================
OpenArm Keyboard Teleop — READY
============================================================
  L         — Reset active arm to home
  Tab       — Switch active arm (left ↔ right)
  K         — Toggle gripper open/close
  W / S     — EE forward / back  (X)
  A / D     — EE left  / right   (Y)
  Q / E     — EE up    / down    (Z)
  Z / X     — Roll  + / -
  T / G     — Pitch + / -
  C / V     — Yaw   + / -

  Active arm: RIGHT
============================================================
```

During use, switch events print:
```
[Switch] Active arm → LEFT
[Reset]  RIGHT arm reset to home.
[KB Debug] First key event: type=KeyboardEventType.KEY_PRESS, raw=<KeyboardInput.W: ...>, name='W'
```

---

## How It Works

### Architecture

```
Keyboard (physical)
        │
        ▼
carb.input.subscribe_to_keyboard_events()   ← registered once at startup
        │
        ▼
_on_keyboard_event(event)                   ← fires on KEY_PRESS / KEY_RELEASE / CHAR
        │  sets kb_state.delta_pos / delta_rot / gripper_closed
        ▼
Main loop  (world.step each frame)
        │
        ├─ delta_pos * POS_SPEED  ──► target_pos  (accumulated)
        │
        ├─ LulaKinematicsSolver.compute_inverse_kinematics(target_pos)
        │        left arm  → 7 joint angles
        │        right arm → 7 joint angles
        │
        ├─ Gripper smooth interpolation
        │
        └─ openarm.apply_action(ArticulationAction(joint_positions))
```

### Step-by-step

1. **SimulationApp** starts Isaac Sim with a GUI window (1920×1080, not headless).
2. **USD load** — opens `openarm_bimanual/openarm_bimanual.usd` which contains the full bimanual robot with physics, drives, and cameras.
3. **Articulation** — the robot prim at `/openarm` is wrapped as an `Articulation` object giving access to DOF names and position control.
4. **Lula IK** — two separate `LulaKinematicsSolver` instances (one per arm) are loaded from the `robot_descriptor.yaml` + URDF files in `openarm_config/left_arm/` and `openarm_config/right_arm/`. These solve inverse kinematics for a 7-DOF arm.
5. **FK home init** — `compute_forward_kinematics("openarm_left_hand", zeros)` and the right equivalent give the exact 3D position of each end-effector at the zero joint config. This becomes the initial `target_pos` for each arm so IK starts from a known reachable pose.
6. **Keyboard registration** — `carb.input.acquire_input_interface().subscribe_to_keyboard_events(keyboard, callback)` registers `_on_keyboard_event`. This uses the Omniverse/carb low-level input system (same underlying API as IsaacLab's `Se3Keyboard` class).
7. **Per-frame integration** — each `world.step()`, `delta_pos * POS_SPEED` is added to `target_pos` for the active arm. Holding a key causes continuous motion; releasing it stops motion (delta_pos resets to zero on KEY_RELEASE).
8. **IK solve + fallback** — Lula solves IK for both arms every frame. If the target moves out of workspace (IK fails), the step is undone (`target_pos -= delta`) so the target stays within reach. The warm-start from the previous frame's joint angles keeps convergence fast.
9. **Gripper** — uses smooth interpolation (`+= clip(target - current, ±speed)`) so the fingers open/close gradually rather than snapping.

---

## Deep Dive — How Lula IK Gets the End-Effector to a Target Point

This is the core of the whole system. When you press `W`, the robot arm actually moves to a new position because of a chain of three concepts working together: **Forward Kinematics**, **Inverse Kinematics**, and **Joint Position Control**.

---

### 1. What is Forward Kinematics (FK)?

FK answers: *"If I set all 7 joints to these angles, where does the hand end up?"*

The OpenArm arm has 7 revolute joints (joint1 → joint7). Each joint rotates around one axis. The combined effect of all rotations stacks up through the robot's links like a chain of coordinate frames, and the final frame position is the end-effector location.

Mathematically, each joint i contributes a **4×4 homogeneous transform matrix** T_i that encodes both the rotation and the offset to the next link (from the URDF). FK multiplies them all together:

```
T_ee = T_base × T_1(q1) × T_2(q2) × T_3(q3) × T_4(q4) × T_5(q5) × T_6(q6) × T_7(q7)
```

The top-left 3×3 of T_ee is the EE orientation, the top-right 3×1 is the EE position.

In code we use FK once at startup to find the home position:
```python
home_pos, home_quat = right_ik.compute_forward_kinematics("openarm_right_hand", np.zeros(7))
# home_pos = [0.0, -0.153, 0.162]  ← where the hand is when all joints = 0
```

That 3D point becomes `right_target_pos`, the EE position we're trying to track.

---

### 2. What is Inverse Kinematics (IK)?

IK answers the opposite question: *"What joint angles do I need to reach this 3D target point?"*

This is hard. FK is a simple multiplication chain — you just compute it. IK is solving that chain **in reverse**, which has no simple closed-form solution for most 6/7-DOF arms. Instead it is solved numerically.

**Every frame**, after `target_pos` is updated by the keyboard:
```python
right_target_pos += kb_state.delta_pos * POS_SPEED
```

We call:
```python
actions, success = right_ik.compute_inverse_kinematics(
    target_position=right_target_pos,   # the 3D point we want the hand at
    target_orientation=None,            # orientation not constrained (position-only)
    frame_name="openarm_right_hand",    # which link to target
    warm_start=last_right_joints,       # starting guess (previous frame's angles)
    position_tolerance=0.02,            # accept solution within 2 cm
)
# actions = 7 joint angles that place the hand at right_target_pos
```

---

### 3. How Lula IK Solves Numerically (Cyclic Coordinate Descent)

Lula (NVIDIA's kinematics library, part of Isaac Sim's Motion Generation) uses a method called **Cyclic Coordinate Descent (CCD)** with gradient refinement. Here is the intuition:

```
Start: current joint angles q = [q1, q2, q3, q4, q5, q6, q7]
Goal:  find q* such that FK(q*).position ≈ target_pos

Repeat until converged:
  For each joint i (from end-effector back to base):
    1. Compute where the EE currently is via FK(q)
    2. Compute how much rotating joint_i alone would move the EE toward target_pos
    3. Adjust q_i by that amount (a small gradient step)
  Check: is ||FK(q).position - target_pos|| < tolerance?
  If yes → done. Return q.
  If no  → repeat.
```

Each iteration improves the configuration a little. The algorithm converges quickly (usually < 50 iterations) because we **warm-start** from the previous frame's solution — the arm barely moved between frames, so the previous angles are an excellent starting guess.

```
Frame N:   target = [0.30, -0.15, 0.20]   →  IK solves from seed  →  q_N
Frame N+1: target = [0.30, -0.15, 0.204]  →  IK solves from q_N   →  q_N+1  (very fast!)
```

Without warm-start, the solver might converge to a different elbow configuration (the "elbow up" vs "elbow down" ambiguity). The seed steers it toward the natural pose:
```python
RIGHT_SEED = np.array([0.0, 1.0, 0.0, 1.2, 0.0, 0.0, 0.0])
# joint2 = +1.0 forces elbow to bend in the natural right-arm direction
```

---

### 4. The Jacobian Relationship (Why It Works)

The reason CCD works is the **Jacobian matrix J**. For a 7-DOF arm, J is a 6×7 matrix where each column j_i says: *"if I rotate joint i by a tiny amount dq_i, the EE moves by J[:,i] × dq_i"*.

The position-only version is a 3×7 matrix:
```
Δx_ee = J(q) × Δq

where:
  Δx_ee = desired EE displacement  (3D vector, metres)
  Δq    = required joint changes    (7D vector, radians)
  J(q)  = Jacobian at current config (3×7)
```

Lula computes J analytically from the URDF geometry and uses the **pseudo-inverse** J† = Jᵀ(JJᵀ)⁻¹ to find:
```
Δq = J†(q) × Δx_ee
```

For a 7-DOF arm targeting 3D position only (3 constraints, 7 unknowns), there are infinitely many solutions — the arm has 4 extra degrees of freedom. The elbow can swing in many ways while the hand stays put. The warm-start + seed configuration picks a consistent elbow posture.

---

### 5. What Happens After IK Returns

Once Lula returns 7 joint angles `q*`:

```python
# Pack the IK solution into the full 22-DOF position array
for i, idx in enumerate(right_arm_idx):   # [1, 3, 5, 7, 9, 11, 13]
    target_positions[idx] = right_joints[i]

# Send to the physics simulation
openarm.apply_action(ArticulationAction(joint_positions=target_positions))
```

`apply_action` writes position targets to the **joint drives** (PD controllers) defined in the USD file. Each drive has a stiffness and damping set in the USD:
- **Stiffness (Kp)** — pulls the joint toward the target angle
- **Damping (Kd)** — resists velocity, prevents oscillation

The physics engine (PhysX, running inside Isaac Sim) then simulates the actual torques applied to the joints, the resulting motion, collisions, gravity, etc. The drives apply `torque = Kp * (q_target - q_actual) + Kd * (dq_target - dq_actual)` every physics sub-step.

So the full chain from keypress to robot movement is:

```
Key W held down
  └─► delta_pos = [1, 0, 0]
        └─► target_pos += [0.004, 0, 0]  each frame
              └─► Lula IK solves q* = FK⁻¹(target_pos)
                    └─► apply_action writes q* to joint drives
                          └─► PhysX torque = Kp*(q*-q_actual) + Kd*(dq)
                                └─► Robot arm physically accelerates toward q*
                                      └─► EE arrives at target_pos  ✓
```

---

### 6. What Happens When IK Fails

If `target_pos` is moved outside the arm's reachable workspace (too far, behind the base, in collision), Lula returns `success=False`. In that case:

```python
if not right_ok:
    right_target_pos -= delta   # undo the keyboard step
    # hold last valid joint config
    for i, idx in enumerate(right_arm_idx):
        target_positions[idx] = last_right_joints[i]
```

The arm freezes at its last valid pose, and `target_pos` is rolled back. This prevents the target "drifting" invisibly into unreachable space — you'll feel the arm stop moving at the workspace boundary.

---

### 7. FK vs IK Summary Table

| | Forward Kinematics (FK) | Inverse Kinematics (IK) |
|---|---|---|
| **Question** | Given joint angles → where is EE? | Given EE position → what joint angles? |
| **Input** | 7 joint angles (radians) | 3D target position (metres) |
| **Output** | 3D EE position + orientation | 7 joint angles |
| **Difficulty** | Easy — just matrix multiply | Hard — solved numerically |
| **Unique?** | Yes, always one answer | No — infinite solutions (7-DOF arm, 3 constraints) |
| **Used when** | Startup: find home position | Every frame: track keyboard target |
| **Speed** | Instant | ~0.1–1 ms with warm-start |

---

### The `carb` CHAR event bug (fixed)

When a key is pressed, carb fires **two** events:
- `KEY_PRESS` — `event.input` is a `KeyboardInput` enum, `.name` gives `"W"`
- `CHAR` — `event.input` is a plain Python `str` (the character, e.g. `"w"`)

The IsaacLab `Se3Keyboard` pattern `event.input.name` crashes on CHAR events because Python `str` has no `.name` attribute. Fixed with:

```python
raw = event.input
name = raw if isinstance(raw, str) else raw.name
```

For CHAR events `name` becomes the lowercase character `"w"`, which is not in our key map (uppercase only), so it silently does nothing. For KEY_PRESS/RELEASE the enum `.name` returns the uppercase key string as expected.

The second error (`KeyError: <class 'NoneType'>` in `omni.kit.manipulator.selector`) is an internal Isaac Sim 5.x UI bug that fires when clicking the viewport with nothing selected. It is cosmetic — the simulation continues running normally.

---

## Files Modified / Created

```
quest3_streamer/
├── src/
│   └── isaac_openarm_keyboard_teleop.py   ← NEW
└── scripts/
    ├── run_openarm_keyboard.sh            ← NEW
    └── openarm_keyboard_teleop.md         ← THIS FILE
```

---

## Known Limitations / Next Steps

- **Position-only IK** — rotation keys (`Z/X/T/G/C/V`) accumulate `delta_rot` but orientation is not yet passed to `compute_inverse_kinematics` (`target_orientation=None`). To enable: convert `delta_rot` to a quaternion and pass it as `target_orientation`.
- **Single gripper toggle** — `K` opens/closes both grippers together. To control per-arm: add `kb_state.left_gripper_closed` / `kb_state.right_gripper_closed` and bind separate keys.
- **No workspace visualisation** — no visual marker for the EE target. Adding a sphere prim at `target_pos` each frame would help.
- **Next goal** — integrate with a task (e.g. pick-and-place) or record demonstrations for LeRobot training.
