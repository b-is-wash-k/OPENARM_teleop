# isaac_openarm_keyboard_teleop.py
# Keyboard-based SE(3) end-effector control for OpenArm in Isaac Sim
# No ROS required - pure Isaac Sim + Lula IK
#
# Key bindings (same as IsaacLab Se3Keyboard):
#   L          - Reset arm to home position
#   K          - Toggle gripper open/close
#   Tab        - Switch active arm (left / right)
#   W / S      - Move end-effector along X axis (forward/back)
#   A / D      - Move end-effector along Y axis (left/right)
#   Q / E      - Move end-effector along Z axis (up/down)
#   Z / X      - Rotate around X axis (roll)
#   T / G      - Rotate around Y axis (pitch)
#   C / V      - Rotate around Z axis (yaw)

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
    "window_width": 1920,
    "window_height": 1080,
})

import os
import sys
import numpy as np
from scipy.spatial.transform import Rotation as R

import carb
import carb.input
import omni
import omni.appwindow
import omni.ui

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from pxr import Usd

# ---------------------------------------------------------------------------
# Paths  (same as the existing teleop script)
# ---------------------------------------------------------------------------
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)

import yaml
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "config.yaml")
with open(CONFIG_PATH) as f:
    PATH_CONFIG = yaml.safe_load(f)

USD_PATH  = os.path.join(PROJECT_ROOT, PATH_CONFIG["paths"]["openarm"]["usd"])
URDF_PATH = os.path.join(PROJECT_ROOT, PATH_CONFIG["paths"]["openarm"]["urdf"])
LEFT_ARM_CFG_DIR  = os.path.join(PROJECT_ROOT, PATH_CONFIG["paths"]["openarm"]["left_arm_config"])
RIGHT_ARM_CFG_DIR = os.path.join(PROJECT_ROOT, PATH_CONFIG["paths"]["openarm"]["right_arm_config"])

# ---------------------------------------------------------------------------
# Tuning knobs
# ---------------------------------------------------------------------------
POS_SPEED     = 0.004   # metres per simulation step while key is held
ROT_SPEED     = 0.008   # radians per simulation step while key is held
GRIPPER_SPEED = 0.05    # gripper position change per step
GRIPPER_OPEN  = 0.132
GRIPPER_CLOSED = -1.0

# OpenArm joint name lists
LEFT_ARM_JOINTS     = [f"openarm_left_joint{i}"  for i in range(1, 8)]
RIGHT_ARM_JOINTS    = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_GRIPPER_JOINTS  = ["openarm_left_finger_joint1",  "openarm_left_finger_joint2"]
RIGHT_GRIPPER_JOINTS = ["openarm_right_finger_joint1", "openarm_right_finger_joint2"]

# IK preferred elbow seeds
LEFT_SEED  = np.array([0.0, -1.0, 0.0, 1.2, 0.0, 0.0, 0.0])
RIGHT_SEED = np.array([0.0,  1.0, 0.0, 1.2, 0.0, 0.0, 0.0])


# ---------------------------------------------------------------------------
# Keyboard state (shared between callback and main loop)
# ---------------------------------------------------------------------------
class KeyboardState:
    def __init__(self):
        self.delta_pos  = np.zeros(3)   # accumulated velocity while keys held
        self.delta_rot  = np.zeros(3)
        self.gripper_closed = False
        self.reset_requested = False
        self.switch_arm_requested = False

kb_state = KeyboardState()

# Key → axis vector mapping (unit vectors, scaled by POS/ROT_SPEED in advance())
_POS_MAP = {
    "W": np.array([ 1.0, 0.0, 0.0]),
    "S": np.array([-1.0, 0.0, 0.0]),
    "A": np.array([ 0.0, 1.0, 0.0]),
    "D": np.array([ 0.0,-1.0, 0.0]),
    "Q": np.array([ 0.0, 0.0, 1.0]),
    "E": np.array([ 0.0, 0.0,-1.0]),
}
_ROT_MAP = {
    "Z": np.array([ 1.0, 0.0, 0.0]),
    "X": np.array([-1.0, 0.0, 0.0]),
    "T": np.array([ 0.0, 1.0, 0.0]),
    "G": np.array([ 0.0,-1.0, 0.0]),
    "C": np.array([ 0.0, 0.0, 1.0]),
    "V": np.array([ 0.0, 0.0,-1.0]),
}


_KB_DEBUG_PRINTED = False

def _on_keyboard_event(event, *args, **kwargs):
    """carb keyboard callback — updates kb_state.

    carb sends three event types on a key press:
      KEY_PRESS  → event.input is a KeyboardInput enum  (name e.g. "W")
      KEY_REPEAT → same enum
      CHAR       → event.input is a str (the character, e.g. "w")
      KEY_RELEASE→ enum again

    The old IsaacLab pattern `event.input.name` crashes on CHAR events because
    str has no .name attribute.  We guard by checking isinstance first.
    """
    global _KB_DEBUG_PRINTED
    raw = event.input
    name = raw if isinstance(raw, str) else raw.name

    # One-time diagnostic so we can confirm the key name format
    if not _KB_DEBUG_PRINTED and event.type == carb.input.KeyboardEventType.KEY_PRESS:
        print(f"[KB Debug] First key event: type={event.type}, raw={repr(raw)}, name={repr(name)}")
        _KB_DEBUG_PRINTED = True

    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if name == "L":
            kb_state.reset_requested = True
        elif name == "K":
            kb_state.gripper_closed = not kb_state.gripper_closed
        elif name == "TAB":
            kb_state.switch_arm_requested = True
        elif name in _POS_MAP:
            kb_state.delta_pos += _POS_MAP[name]
        elif name in _ROT_MAP:
            kb_state.delta_rot += _ROT_MAP[name]

    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        if name in _POS_MAP:
            kb_state.delta_pos -= _POS_MAP[name]
        elif name in _ROT_MAP:
            kb_state.delta_rot -= _ROT_MAP[name]

    return True


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _find_robot_prim(stage):
    candidates = [
        "/World/Robot", "/World/openarm", "/openarm", "/Robot",
        "/Environment/openarm", "/World/openarm_bimanual",
    ]
    for path in candidates:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            return path
    return None


def _map_joints(dof_names, joint_name_lists):
    """Return index list for each group, preserving joint order."""
    indices = []
    for joint_list in joint_name_lists:
        idx = [i for i, n in enumerate(dof_names) if n in joint_list]
        indices.append(idx)
    return indices


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    # ------------------------------------------------------------------
    # 1. Warm up & load stage
    # ------------------------------------------------------------------
    print("[Init] Warming up Isaac Sim...")
    for _ in range(30):
        simulation_app.update()

    print(f"[Init] Loading USD: {USD_PATH}")
    open_stage(USD_PATH)

    for _ in range(50):
        simulation_app.update()

    # ------------------------------------------------------------------
    # 2. World + robot
    # ------------------------------------------------------------------
    world = World(stage_units_in_meters=1.0)
    for _ in range(20):
        simulation_app.update()

    stage = world.stage
    robot_prim_path = _find_robot_prim(stage)
    if robot_prim_path is None:
        print("[ERROR] Cannot find OpenArm prim.  Available root prims:")
        for p in stage.GetPseudoRoot().GetChildren():
            print(f"  {p.GetPath()}")
        simulation_app.close()
        return

    print(f"[Init] Robot prim: {robot_prim_path}")
    openarm = world.scene.add(
        Articulation(prim_path=robot_prim_path, name="openarm")
    )

    # ------------------------------------------------------------------
    # 3. IK solvers (Lula)
    # ------------------------------------------------------------------
    print("[Init] Loading Lula IK solvers...")
    from omni.isaac.motion_generation import LulaKinematicsSolver

    left_ik  = LulaKinematicsSolver(
        robot_description_path=os.path.join(LEFT_ARM_CFG_DIR,  "robot_descriptor.yaml"),
        urdf_path=URDF_PATH,
    )
    right_ik = LulaKinematicsSolver(
        robot_description_path=os.path.join(RIGHT_ARM_CFG_DIR, "robot_descriptor.yaml"),
        urdf_path=URDF_PATH,
    )
    print("[Init] IK solvers ready.")

    # ------------------------------------------------------------------
    # 4. Register keyboard
    # ------------------------------------------------------------------
    appwindow       = omni.appwindow.get_default_app_window()
    input_iface     = carb.input.acquire_input_interface()
    keyboard        = appwindow.get_keyboard()
    _kb_sub         = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    print("[Init] Keyboard registered.")

    # ------------------------------------------------------------------
    # 5. Minimise UI panels
    # ------------------------------------------------------------------
    for win_name in ["Stage", "Layer", "Render Settings", "Content",
                     "Console", "Property", "Properties"]:
        try:
            w = omni.ui.Workspace.get_window(win_name)
            if w:
                w.visible = False
        except Exception:
            pass

    # ------------------------------------------------------------------
    # 6. Reset world, grab DOF info
    # ------------------------------------------------------------------
    world.reset()
    for _ in range(20):
        simulation_app.update()

    dof_names = openarm.dof_names
    print(f"[Info] DOFs ({len(dof_names)}): {dof_names}")

    (left_arm_idx, right_arm_idx,
     left_grip_idx, right_grip_idx) = _map_joints(
        dof_names,
        [LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, LEFT_GRIPPER_JOINTS, RIGHT_GRIPPER_JOINTS],
    )
    print(f"[Info] Left  arm indices : {left_arm_idx}")
    print(f"[Info] Right arm indices : {right_arm_idx}")
    print(f"[Info] Left  grip indices: {left_grip_idx}")
    print(f"[Info] Right grip indices: {right_grip_idx}")

    # ------------------------------------------------------------------
    # 7. Initialise target poses from FK at zero config
    # ------------------------------------------------------------------
    zero7 = np.zeros(7)
    left_home_pos,  left_home_quat  = left_ik.compute_forward_kinematics("openarm_left_hand",  zero7)
    right_home_pos, right_home_quat = right_ik.compute_forward_kinematics("openarm_right_hand", zero7)

    left_home_pos  = np.array(left_home_pos)
    right_home_pos = np.array(right_home_pos)

    print(f"[FK] Left  home EE : {np.round(left_home_pos,  3)}")
    print(f"[FK] Right home EE : {np.round(right_home_pos, 3)}")

    # Mutable target poses (we accumulate delta into these)
    left_target_pos  = left_home_pos.copy()
    right_target_pos = right_home_pos.copy()

    # Warm-start joint positions for IK
    last_left_joints  = LEFT_SEED.copy()
    last_right_joints = RIGHT_SEED.copy()

    # Gripper smooth positions
    left_grip_smooth  = GRIPPER_OPEN
    right_grip_smooth = GRIPPER_OPEN

    # Which arm the keyboard currently controls
    active_arm = "right"

    # ------------------------------------------------------------------
    # 8. Print controls
    # ------------------------------------------------------------------
    print("\n" + "=" * 60)
    print("OpenArm Keyboard Teleop — READY")
    print("=" * 60)
    print("  L         — Reset active arm to home")
    print("  Tab       — Switch active arm (left ↔ right)")
    print("  K         — Toggle gripper open/close")
    print("  W / S     — EE forward / back  (X)")
    print("  A / D     — EE left  / right   (Y)")
    print("  Q / E     — EE up    / down    (Z)")
    print("  Z / X     — Roll  + / -")
    print("  T / G     — Pitch + / -")
    print("  C / V     — Yaw   + / -")
    print(f"\n  Active arm: {active_arm.upper()}")
    print("=" * 60 + "\n")

    # ------------------------------------------------------------------
    # 9. Main loop
    # ------------------------------------------------------------------
    while simulation_app.is_running():

        # --- Handle one-shot keyboard requests ---
        if kb_state.reset_requested:
            kb_state.reset_requested = False
            if active_arm == "left":
                left_target_pos = left_home_pos.copy()
            else:
                right_target_pos = right_home_pos.copy()
            print(f"[Reset] {active_arm.upper()} arm reset to home.")

        if kb_state.switch_arm_requested:
            kb_state.switch_arm_requested = False
            active_arm = "left" if active_arm == "right" else "right"
            print(f"[Switch] Active arm → {active_arm.upper()}")

        # --- Integrate keyboard velocity into target position ---
        delta = kb_state.delta_pos * POS_SPEED
        # delta_rot is for future orientation control; IK currently position-only
        if active_arm == "left":
            left_target_pos  += delta
        else:
            right_target_pos += delta

        # --- Grab current joint positions ---
        current_pos = openarm.get_joint_positions()
        if current_pos is None:
            world.step(render=True)
            continue
        target_positions = current_pos.copy()

        # --- Left arm IK ---
        left_actions, left_ok = left_ik.compute_inverse_kinematics(
            target_position=left_target_pos,
            target_orientation=None,
            frame_name="openarm_left_hand",
            warm_start=last_left_joints,
            position_tolerance=0.02,
        )
        if left_ok:
            left_joints = np.array(left_actions).flatten()[:7]
            last_left_joints = left_joints.copy()
            for i, idx in enumerate(left_arm_idx):
                target_positions[idx] = left_joints[i]
        else:
            # IK failed — hold last known position, nudge target back
            if active_arm == "left":
                left_target_pos -= delta  # undo the step
            for i, idx in enumerate(left_arm_idx):
                target_positions[idx] = last_left_joints[i]

        # --- Right arm IK ---
        right_actions, right_ok = right_ik.compute_inverse_kinematics(
            target_position=right_target_pos,
            target_orientation=None,
            frame_name="openarm_right_hand",
            warm_start=last_right_joints,
            position_tolerance=0.02,
        )
        if right_ok:
            right_joints = np.array(right_actions).flatten()[:7]
            last_right_joints = right_joints.copy()
            for i, idx in enumerate(right_arm_idx):
                target_positions[idx] = right_joints[i]
        else:
            if active_arm == "right":
                right_target_pos -= delta  # undo the step
            for i, idx in enumerate(right_arm_idx):
                target_positions[idx] = last_right_joints[i]

        # --- Gripper control ---
        grip_target = GRIPPER_CLOSED if kb_state.gripper_closed else GRIPPER_OPEN

        # Left gripper (tracks active arm toggle if you add per-arm gripper logic later)
        left_grip_smooth += np.clip(grip_target - left_grip_smooth,
                                    -GRIPPER_SPEED, GRIPPER_SPEED)
        for idx in left_grip_idx:
            target_positions[idx] = left_grip_smooth

        right_grip_smooth += np.clip(grip_target - right_grip_smooth,
                                     -GRIPPER_SPEED, GRIPPER_SPEED)
        for idx in right_grip_idx:
            target_positions[idx] = right_grip_smooth

        # --- Apply ---
        openarm.apply_action(ArticulationAction(joint_positions=target_positions))
        world.step(render=True)

    # ------------------------------------------------------------------
    # Cleanup
    # ------------------------------------------------------------------
    input_iface.unsubscribe_to_keyboard_events(keyboard, _kb_sub)
    simulation_app.close()


if __name__ == "__main__":
    main()
