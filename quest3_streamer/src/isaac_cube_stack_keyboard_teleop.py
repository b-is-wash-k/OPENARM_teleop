# isaac_cube_stack_keyboard_teleop.py
# OpenArm Bimanual Cube-Stack Teleop — KEYBOARD VERSION
#
# Built on the same carb keyboard pattern as isaac_openarm_keyboard_teleop.py
# (confirmed working).  Cube task, camera cycling, per-arm gripper layered on top.
# Keyboard registration code is unchanged from the working script.
#
# Key bindings:
#   L         - Reset active arm to home
#   Tab       - Switch active arm (left / right)
#   K         - Toggle gripper (active arm)
#   B         - Toggle gripper (both arms)
#   W / S     - EE forward / back  (X)
#   A / D     - EE left   / right  (Y)
#   Q / E     - EE up     / down   (Z)
#   Z / X     - Roll   + / -
#   T / G     - Pitch  + / -
#   C / V     - Yaw    + / -
#   N         - Cycle camera view
#   R         - Reset cubes to start positions
#   P         - Print current EE target positions

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless":      False,
    "width":         1920,
    "height":        1080,
    "window_width":  1920,
    "window_height": 1080,
})

import os
import numpy as np
from scipy.spatial.transform import Rotation as R

import carb
import carb.input
import omni
import omni.appwindow
import omni.ui
import omni.kit.viewport.utility   # must be top-level — local import inside main() shadows global 'omni'

from omni.isaac.core import World
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)

import yaml
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "config.yaml")
with open(CONFIG_PATH) as f:
    PATH_CONFIG = yaml.safe_load(f)

USD_PATH          = os.path.join(PROJECT_ROOT, PATH_CONFIG["paths"]["openarm"]["usd"])
URDF_PATH         = os.path.join(PROJECT_ROOT, PATH_CONFIG["paths"]["openarm"]["urdf"])
LEFT_ARM_CFG_DIR  = os.path.join(PROJECT_ROOT, PATH_CONFIG["paths"]["openarm"]["left_arm_config"])
RIGHT_ARM_CFG_DIR = os.path.join(PROJECT_ROOT, PATH_CONFIG["paths"]["openarm"]["right_arm_config"])

# ── Tuning knobs ──────────────────────────────────────────────────────────────
POS_SPEED      = 0.004    # metres per sim step while key held
ROT_SPEED      = 0.008    # radians per sim step while key held
GRIPPER_SPEED  = 0.05
GRIPPER_OPEN   = 0.132
GRIPPER_CLOSED = -1.0

# ── Joint names ───────────────────────────────────────────────────────────────
LEFT_ARM_JOINTS      = [f"openarm_left_joint{i}"  for i in range(1, 8)]
RIGHT_ARM_JOINTS     = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_GRIPPER_JOINTS  = ["openarm_left_finger_joint1",  "openarm_left_finger_joint2"]
RIGHT_GRIPPER_JOINTS = ["openarm_right_finger_joint1", "openarm_right_finger_joint2"]

LEFT_SEED  = np.array([0.0, -1.0, 0.0, 1.2, 0.0, 0.0, 0.0])
RIGHT_SEED = np.array([0.0,  1.0, 0.0, 1.2, 0.0, 0.0, 0.0])

# ── Cube task config ──────────────────────────────────────────────────────────
CUBE_SIZE          = 0.10
TABLE_SURFACE_Z    = 0.99
STACK_TOLERANCE_XY = CUBE_SIZE * 0.6
STACK_TOLERANCE_Z  = CUBE_SIZE * 0.5
BOX_CENTER_X       = 0.034
BOX_CENTER_Y       = -0.012
CUBE_SPACING       = CUBE_SIZE * 1.6

CUBE_START_POSITIONS = {
    "cubeA": np.array([BOX_CENTER_X + CUBE_SPACING, BOX_CENTER_Y, 0.0]),  # Red
    "cubeB": np.array([BOX_CENTER_X,                BOX_CENTER_Y, 0.0]),  # Green
    "cubeC": np.array([BOX_CENTER_X - CUBE_SPACING, BOX_CENTER_Y, 0.0]),  # Blue
}
CUBE_COLORS = {
    "cubeA": np.array([0.8, 0.1, 0.1]),
    "cubeB": np.array([0.1, 0.7, 0.1]),
    "cubeC": np.array([0.1, 0.3, 0.9]),
}

# ── Key → axis mapping ────────────────────────────────────────────────────────
_POS_MAP = {
    "W": np.array([ 1., 0., 0.]),  "S": np.array([-1., 0., 0.]),
    "A": np.array([ 0., 1., 0.]),  "D": np.array([ 0.,-1., 0.]),
    "Q": np.array([ 0., 0., 1.]),  "E": np.array([ 0., 0.,-1.]),
}
_ROT_MAP = {
    "Z": np.array([ 1., 0., 0.]),  "X": np.array([-1., 0., 0.]),
    "T": np.array([ 0., 1., 0.]),  "G": np.array([ 0.,-1., 0.]),
    "C": np.array([ 0., 0., 1.]),  "V": np.array([ 0., 0.,-1.]),
}

# ═════════════════════════════════════════════════════════════════════════════
# Keyboard state  (written by carb callback, read in main loop)
# ═════════════════════════════════════════════════════════════════════════════
class KeyboardState:
    def __init__(self):
        self.delta_pos             = np.zeros(3)
        self.delta_rot             = np.zeros(3)
        self.left_gripper_closed   = False
        self.right_gripper_closed  = False
        self.reset_arm_requested   = False   # L
        self.switch_arm_requested  = False   # TAB
        self.toggle_grip_active    = False   # K
        self.toggle_grip_both      = False   # B
        self.cycle_cam_requested   = False   # N
        self.reset_cubes_requested = False   # R
        self.print_pos_requested   = False   # P

kb = KeyboardState()
_KB_DEBUG_PRINTED = False

def _on_keyboard_event(event, *args, **kwargs):
    """
    carb keyboard callback — same pattern as isaac_openarm_keyboard_teleop.py.
    KEY_PRESS adds to delta vectors, KEY_RELEASE subtracts.
    Guards against CHAR events (str has no .name) with isinstance check.
    """
    global _KB_DEBUG_PRINTED
    raw  = event.input
    name = raw if isinstance(raw, str) else raw.name

    if not _KB_DEBUG_PRINTED and event.type == carb.input.KeyboardEventType.KEY_PRESS:
        print(f"[KB] First event — name={repr(name)}")
        _KB_DEBUG_PRINTED = True

    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if   name == "L":   kb.reset_arm_requested   = True
        elif name == "TAB": kb.switch_arm_requested  = True
        elif name == "K":   kb.toggle_grip_active    = True
        elif name == "B":   kb.toggle_grip_both      = True
        elif name == "N":   kb.cycle_cam_requested   = True
        elif name == "R":   kb.reset_cubes_requested = True
        elif name == "P":   kb.print_pos_requested   = True
        elif name in _POS_MAP: kb.delta_pos += _POS_MAP[name]
        elif name in _ROT_MAP: kb.delta_rot += _ROT_MAP[name]

    elif event.type == carb.input.KeyboardEventType.KEY_RELEASE:
        if   name in _POS_MAP: kb.delta_pos -= _POS_MAP[name]
        elif name in _ROT_MAP: kb.delta_rot -= _ROT_MAP[name]

    return True


# ═════════════════════════════════════════════════════════════════════════════
# Cube helpers
# ═════════════════════════════════════════════════════════════════════════════
def get_table_surface_z(stage, fallback=TABLE_SURFACE_Z):
    try:
        from pxr import UsdGeom
        cache    = UsdGeom.BBoxCache(0, ["default", "render"])
        box_prim = stage.GetPrimAtPath("/box")
        if box_prim.IsValid():
            bbox  = cache.ComputeWorldBound(box_prim)
            z_min = bbox.GetRange().GetMin()[2]
            if 0.3 < z_min < 2.5:
                print(f"[Scene] Table surface Z = {z_min:.4f}")
                return z_min
        tp = stage.GetPrimAtPath("/packing_table_01")
        if tp.IsValid():
            bbox  = UsdGeom.BBoxCache(0, ["default","render"]).ComputeWorldBound(tp)
            z_max = bbox.GetRange().GetMax()[2]
            if 0.3 < z_max < 2.5:
                return z_max
    except Exception as e:
        print(f"[Scene] Table Z detect failed: {e}")
    print(f"[Scene] Using fallback Z = {fallback}")
    return fallback


def delete_prim(stage, path):
    try:
        p = stage.GetPrimAtPath(path)
        if p.IsValid():
            stage.RemovePrim(path)
            print(f"[Scene] Deleted {path}")
    except Exception as e:
        print(f"[Scene] delete_prim({path}): {e}")


def spawn_cubes(world, table_z):
    try:
        from omni.isaac.core.objects import DynamicCuboid
    except ImportError:
        try:
            from isaacsim.core.api.objects import DynamicCuboid
        except ImportError:
            return _spawn_cubes_usd(world, table_z)
    half  = CUBE_SIZE / 2.0
    cubes = {}
    for name, xy in CUBE_START_POSITIONS.items():
        pos = xy.copy();  pos[2] = table_z + half + 0.002
        cubes[name] = world.scene.add(DynamicCuboid(
            prim_path=f"/World/Cubes/{name}", name=name,
            position=pos, scale=np.array([CUBE_SIZE]*3),
            color=CUBE_COLORS[name], mass=0.2,
        ))
        print(f"[Cubes] {name} at {np.round(pos,3)}")
    return cubes


def _spawn_cubes_usd(world, table_z):
    from pxr import UsdGeom, UsdPhysics, Gf
    stage = world.stage;  half = CUBE_SIZE / 2.0;  cubes = {}
    for name, xy in CUBE_START_POSITIONS.items():
        path = f"/World/Cubes/{name}";  pos = xy.copy();  pos[2] = table_z + half + 0.002
        xf   = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*pos.tolist()))
        cb   = UsdGeom.Cube.Define(stage, path + "/Cube")
        cb.GetSizeAttr().Set(CUBE_SIZE)
        c = CUBE_COLORS[name];  cb.GetDisplayColorAttr().Set([(c[0],c[1],c[2])])
        UsdPhysics.RigidBodyAPI.Apply(xf.GetPrim())
        UsdPhysics.MassAPI.Apply(xf.GetPrim()).GetMassAttr().Set(0.2)
        UsdPhysics.CollisionAPI.Apply(cb.GetPrim())
        cubes[name] = xf;  print(f"[Cubes] USD {name} at {np.round(pos,3)}")
    return cubes


def reset_cubes(world, cubes, table_z):
    half = CUBE_SIZE / 2.0
    for name, cube in cubes.items():
        pos = CUBE_START_POSITIONS[name].copy();  pos[2] = table_z + half + 0.002
        try:
            cube.set_world_pose(position=pos, orientation=np.array([1,0,0,0]))
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))
        except AttributeError:
            from pxr import Gf, UsdGeom as UG
            xf = UG.Xformable(cube.GetPrim() if hasattr(cube,'GetPrim') else cube)
            for op in xf.GetOrderedXformOps():
                if "translate" in str(op.GetOpName()).lower():
                    op.Set(Gf.Vec3d(*pos.tolist()))
    print("[Cubes] Reset to start positions")


def check_stacking(cubes):
    names = list(cubes.keys());  stacked = []
    for top in names:
        try: tp, _ = cubes[top].get_world_pose();  tp = np.array(tp)
        except Exception: continue
        for bot in names:
            if bot == top: continue
            try: bp, _ = cubes[bot].get_world_pose();  bp = np.array(bp)
            except Exception: continue
            if (np.linalg.norm(tp[:2]-bp[:2]) < STACK_TOLERANCE_XY and
                    abs(tp[2]-bp[2] - CUBE_SIZE)  < STACK_TOLERANCE_Z):
                stacked.append(f"{top} ON {bot}")
    return stacked


def apply_rot_delta(rot_wxyz, delta_xyz_rad):
    """Compose a small XYZ-euler delta into a wxyz quaternion."""
    if np.linalg.norm(delta_xyz_rad) < 1e-9:
        return rot_wxyz
    q  = np.array([rot_wxyz[1], rot_wxyz[2], rot_wxyz[3], rot_wxyz[0]])  # xyzw
    nq = (R.from_quat(q) * R.from_euler('xyz', delta_xyz_rad)).as_quat()
    return np.array([nq[3], nq[0], nq[1], nq[2]])  # wxyz


# ═════════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════════
def main():
    # 1. Warm up & load stage
    print("[Init] Warming up Isaac Sim...")
    for _ in range(30): simulation_app.update()
    print(f"[Init] Loading USD: {USD_PATH}")
    open_stage(USD_PATH)
    for _ in range(50): simulation_app.update()

    # 2. World
    world = World(stage_units_in_meters=1.0)
    for _ in range(20): simulation_app.update()
    stage = world.stage

    # 3. Scene setup — table, cubes
    table_z = get_table_surface_z(stage)
    delete_prim(stage, "/electric_screw_driver")
    delete_prim(stage, "/box")
    print(f"[Init] Spawning cubes at table Z = {table_z:.3f} m")
    cubes = spawn_cubes(world, table_z)

    # 4. Robot
    robot_path = None
    for p in ["/World/Robot", "/World/openarm", "/openarm", "/Robot"]:
        if stage.GetPrimAtPath(p).IsValid():
            robot_path = p; break
    if robot_path is None:
        print("[ERROR] Robot prim not found!"); simulation_app.close(); return
    print(f"[Init] Robot prim: {robot_path}")
    openarm = world.scene.add(Articulation(prim_path=robot_path, name="openarm"))

    # 5. IK solvers
    from omni.isaac.motion_generation import LulaKinematicsSolver
    left_ik  = LulaKinematicsSolver(
        robot_description_path=os.path.join(LEFT_ARM_CFG_DIR,  "robot_descriptor.yaml"),
        urdf_path=URDF_PATH)
    right_ik = LulaKinematicsSolver(
        robot_description_path=os.path.join(RIGHT_ARM_CFG_DIR, "robot_descriptor.yaml"),
        urdf_path=URDF_PATH)
    print("[Init] IK solvers ready.")

    # 6. Register carb keyboard  ← same as working script
    appwindow   = omni.appwindow.get_default_app_window()
    input_iface = carb.input.acquire_input_interface()
    keyboard    = appwindow.get_keyboard()
    _kb_sub     = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
    print("[Init] Keyboard registered.")

    # 7. Minimise UI
    for win in ["Stage","Layer","Render Settings","Content","Console","Property","Properties","Semantics"]:
        try:
            w = omni.ui.Workspace.get_window(win)
            if w: w.visible = False
        except Exception: pass

    # 8. Reset world, DOF info
    world.reset()
    for _ in range(20): simulation_app.update()
    dof_names = openarm.dof_names

    def _idx(jlist): return [i for i,n in enumerate(dof_names) if n in jlist]
    left_arm_idx   = _idx(LEFT_ARM_JOINTS)
    right_arm_idx  = _idx(RIGHT_ARM_JOINTS)
    left_grip_idx  = _idx(LEFT_GRIPPER_JOINTS)
    right_grip_idx = _idx(RIGHT_GRIPPER_JOINTS)
    print(f"[Info] L arm:{left_arm_idx}  R arm:{right_arm_idx}")

    # 9. Home poses from FK
    zero7 = np.zeros(7)
    l_home_pos, l_home_quat = left_ik.compute_forward_kinematics("openarm_left_hand",  zero7)
    r_home_pos, r_home_quat = right_ik.compute_forward_kinematics("openarm_right_hand", zero7)
    l_home_pos = np.array(l_home_pos);  l_home_quat = np.array(l_home_quat)
    r_home_pos = np.array(r_home_pos);  r_home_quat = np.array(r_home_quat)
    print(f"[FK] L home: {np.round(l_home_pos,3)}")
    print(f"[FK] R home: {np.round(r_home_pos,3)}")

    l_tgt_pos = l_home_pos.copy();  l_tgt_rot = l_home_quat.copy()
    r_tgt_pos = r_home_pos.copy();  r_tgt_rot = r_home_quat.copy()
    last_lj   = LEFT_SEED.copy();   last_rj   = RIGHT_SEED.copy()
    l_grip_sm = GRIPPER_OPEN;       r_grip_sm = GRIPPER_OPEN
    active    = "right"

    # 10. Camera setup
    from pxr import UsdGeom as UG
    cameras      = ["/OmniverseKit_Persp"];  camera_names = ["Perspective"]
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if prim.IsA(UG.Camera) and path not in \
                ["/OmniverseKit_Persp", "/OmniverseKit_Front", "/OmniverseKit_Right"]:
            cameras.append(path)
            camera_names.append(path.split("/")[-1].replace("_"," ").title())
    print(f"[Camera] {len(cameras)} cameras: {camera_names}")
    current_cam = 0

    frame_ctr = 0;  last_stack_msg = ""

    print("\n" + "="*60)
    print("  OpenArm Cube Stack Teleop  [KEYBOARD]")
    print("  3 cubes on table: Red | Green | Blue")
    print("="*60)
    print("  L         — Reset active arm to home")
    print("  Tab       — Switch active arm (left ↔ right)")
    print("  K         — Toggle gripper (active arm)")
    print("  B         — Toggle gripper (both arms)")
    print("  W/S A/D Q/E  — EE X / Y / Z")
    print("  Z/X T/G C/V  — Roll / Pitch / Yaw")
    print("  N         — Cycle camera")
    print("  R         — Reset cubes")
    print("  P         — Print EE positions")
    print(f"\n  Active arm: {active.upper()}")
    print("="*60 + "\n")

    # 11. Main loop
    while simulation_app.is_running():
        frame_ctr += 1

        # One-shot events
        if kb.reset_arm_requested:
            kb.reset_arm_requested = False
            if active == "left":  l_tgt_pos = l_home_pos.copy(); l_tgt_rot = l_home_quat.copy()
            else:                 r_tgt_pos = r_home_pos.copy(); r_tgt_rot = r_home_quat.copy()
            print(f"[Reset] {active.upper()} arm → home")

        if kb.switch_arm_requested:
            kb.switch_arm_requested = False
            active = "left" if active == "right" else "right"
            print(f"[Switch] Active arm → {active.upper()}")

        if kb.toggle_grip_active:
            kb.toggle_grip_active = False
            if active == "left":
                kb.left_gripper_closed  = not kb.left_gripper_closed
                print(f"[Gripper] LEFT  → {'CLOSED' if kb.left_gripper_closed  else 'OPEN'}")
            else:
                kb.right_gripper_closed = not kb.right_gripper_closed
                print(f"[Gripper] RIGHT → {'CLOSED' if kb.right_gripper_closed else 'OPEN'}")

        if kb.toggle_grip_both:
            kb.toggle_grip_both = False
            new = not kb.left_gripper_closed
            kb.left_gripper_closed = kb.right_gripper_closed = new
            print(f"[Gripper] BOTH → {'CLOSED' if new else 'OPEN'}")

        if kb.cycle_cam_requested:
            kb.cycle_cam_requested = False
            current_cam = (current_cam + 1) % len(cameras)
            vp = omni.kit.viewport.utility.get_active_viewport()
            if vp:
                try: vp.camera_path = cameras[current_cam]; print(f"[Camera] → {camera_names[current_cam]}")
                except Exception: pass

        if kb.reset_cubes_requested:
            kb.reset_cubes_requested = False
            reset_cubes(world, cubes, table_z)

        if kb.print_pos_requested:
            kb.print_pos_requested = False
            print(f"[L] pos={np.round(l_tgt_pos,3)}  rot={np.round(l_tgt_rot,3)}")
            print(f"[R] pos={np.round(r_tgt_pos,3)}  rot={np.round(r_tgt_rot,3)}")

        # Integrate held-key velocity into active arm target position
        dp = kb.delta_pos * POS_SPEED
        if active == "left":
            l_tgt_pos += dp
        else:
            r_tgt_pos += dp

        # Joint positions
        cur = openarm.get_joint_positions()
        if cur is None:
            world.step(render=True); continue
        tgt = cur.copy()

        # Left IK
        l_acts, l_ok = left_ik.compute_inverse_kinematics(
            target_position=l_tgt_pos, target_orientation=None,
            frame_name="openarm_left_hand", warm_start=last_lj, position_tolerance=0.02)
        if l_ok:
            lj = np.array(l_acts).flatten()[:7]; last_lj = lj.copy()
            for i, idx in enumerate(left_arm_idx): tgt[idx] = lj[i]
        else:
            if active == "left": l_tgt_pos -= dp   # undo on IK fail
            for i, idx in enumerate(left_arm_idx): tgt[idx] = last_lj[i]

        # Right IK
        r_acts, r_ok = right_ik.compute_inverse_kinematics(
            target_position=r_tgt_pos, target_orientation=None,
            frame_name="openarm_right_hand", warm_start=last_rj, position_tolerance=0.02)
        if r_ok:
            rj = np.array(r_acts).flatten()[:7]; last_rj = rj.copy()
            for i, idx in enumerate(right_arm_idx): tgt[idx] = rj[i]
        else:
            if active == "right": r_tgt_pos -= dp
            for i, idx in enumerate(right_arm_idx): tgt[idx] = last_rj[i]

        # Per-arm gripper
        l_g = GRIPPER_CLOSED if kb.left_gripper_closed  else GRIPPER_OPEN
        r_g = GRIPPER_CLOSED if kb.right_gripper_closed else GRIPPER_OPEN
        l_grip_sm += np.clip(l_g - l_grip_sm, -GRIPPER_SPEED, GRIPPER_SPEED)
        r_grip_sm += np.clip(r_g - r_grip_sm, -GRIPPER_SPEED, GRIPPER_SPEED)
        for idx in left_grip_idx:  tgt[idx] = l_grip_sm
        for idx in right_grip_idx: tgt[idx] = r_grip_sm

        openarm.apply_action(ArticulationAction(joint_positions=tgt))

        # Stacking check
        if frame_ctr % 30 == 0:
            stacked = check_stacking(cubes)
            msg = ", ".join(stacked) if stacked else ""
            if msg != last_stack_msg:
                if stacked: print(f"[Stack] ✅ {msg}")
                last_stack_msg = msg

        world.step(render=True)

    # Cleanup
    input_iface.unsubscribe_to_keyboard_events(keyboard, _kb_sub)
    simulation_app.close()


if __name__ == "__main__":
    main()
