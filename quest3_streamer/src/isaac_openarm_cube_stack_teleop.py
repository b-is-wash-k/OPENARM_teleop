# isaac_openarm_cube_stack_teleop.py
# OpenArm Bimanual Teleoperation — Cube Stack Task
# Based on isaac_openarm_teleop.py v3 (two-stage IK + smooth_quaternion)
# Adds: 3 colored cubes (Red / Green / Blue), stacking detection, cube reset
#
# Controls:
#   Trigger / Grip     → Close gripper
#   A button           → Cycle camera
#   B / Y button or R  → Reset cubes to start positions

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
    "window_width": 1920,
    "window_height": 1080,
})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")

try:
    import rclpy
except ImportError:
    print("ERROR: rclpy not found. Don't source system ROS 2 before running Isaac Sim.")
    raise

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState, Image
import numpy as np
from scipy.spatial.transform import Rotation as R
import carb.input
import omni.appwindow
from omni.isaac.core import World
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.articulations import Articulation
from pxr import UsdGeom, UsdPhysics, Gf
import os
import yaml
import time

# ── Config paths ──────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH  = os.path.join(PROJECT_ROOT, "config", "config.yaml")

with open(CONFIG_PATH, 'r') as f:
    PATH_CONFIG = yaml.safe_load(f)

USD_PATH             = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['usd'])
URDF_PATH            = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['urdf'])
LEFT_ARM_CONFIG_DIR  = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['left_arm_config'])
RIGHT_ARM_CONFIG_DIR = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['right_arm_config'])

# ── Robot / teleop config ─────────────────────────────────────────────────────
CONFIG = {
    "pos_scale":               1,
    "robot_workspace_center":  [0.3, 0.0, 0.3],
    "left_arm_offset":         [0.0,  0.15, 0.0],
    "right_arm_offset":        [0.0, -0.15, 0.0],
    "smoothing":               0.1,
    "gripper_threshold":       0.5,
    "gripper_open_pos":        0.132,
    "gripper_closed_pos":     -1,
    "gripper_speed":           0.05,
    "calibration_samples":     30,
    "debug_ik":                False,
    "use_orientation_ik":      True,
    "ik_orientation_fallback": True,
    "position_tolerance":      0.01,
    "orientation_tolerance":   0.7,

    # ── Cube size (metres) ────────────────────────────────────────────────────
    # Change this to resize all 3 cubes uniformly.
    "cube_size": 0.1,

    # ── Scene offset — shifts the table + cubes together as a group ───────────
    # [X, Y, Z] in metres. Positive X moves everything away from the robot.
    # Applied once at startup; reset (B/Y/R) restores both table and cubes to
    # these offset positions, not back to the raw USD defaults.
    "environment_object_offset": [-0.25, 0.0, 0.0],
}

# ── Cube task config ──────────────────────────────────────────────────────────
CUBE_SIZE          = CONFIG["cube_size"]   # driven by CONFIG — change there, not here
TABLE_SURFACE_Z    = 0.99
STACK_TOLERANCE_XY = 0.04
STACK_TOLERANCE_Z  = 0.03

CUBE_SPACING = 0.10   # metres between cube centres — tune if needed

# Cube order: Red right, Green centre, Blue left (relative to table centre)
CUBE_OFFSETS = {
    "cubeA": np.array([ CUBE_SPACING, 0.0, 0.0]),   # Red
    "cubeB": np.array([ 0.0,          0.0, 0.0]),   # Green
    "cubeC": np.array([-CUBE_SPACING, 0.0, 0.0]),   # Blue
}
CUBE_COLORS = {
    "cubeA": np.array([0.8, 0.1, 0.1]),
    "cubeB": np.array([0.1, 0.7, 0.1]),
    "cubeC": np.array([0.1, 0.3, 0.9]),
}

# ── Joint names ───────────────────────────────────────────────────────────────
LEFT_ARM_JOINTS = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7"
]
RIGHT_ARM_JOINTS = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7"
]
LEFT_GRIPPER_JOINTS  = ["openarm_left_finger_joint1",  "openarm_left_finger_joint2"]
RIGHT_GRIPPER_JOINTS = ["openarm_right_finger_joint1", "openarm_right_finger_joint2"]

LEFT_ARM_PREFERRED_CONFIG  = np.array([0.0, -1.0, 0.0, 1.2, 0.0, 0.0, 0.0])
RIGHT_ARM_PREFERRED_CONFIG = np.array([0.0,  1.0, 0.0, 1.2, 0.0, 0.0, 0.0])


# ── Keyboard state ────────────────────────────────────────────────────────────
class KeyboardState:
    def __init__(self):
        self.reset_requested = False

kb_state = KeyboardState()


def _on_keyboard_event(event, *args, **kwargs):
    raw = event.input
    name = raw if isinstance(raw, str) else raw.name
    if event.type == carb.input.KeyboardEventType.KEY_PRESS and name.upper() == "R":
        kb_state.reset_requested = True
    return True


# ── IK helpers (from isaac_openarm_teleop v3) ─────────────────────────────────
def smooth_quaternion(current, target, alpha):
    """Blend wxyz quaternions while avoiding sign-flip jumps."""
    if np.dot(current, target) < 0:
        target = -target
    blended = alpha * current + (1 - alpha) * target
    norm = np.linalg.norm(blended)
    if norm < 1e-8:
        return target
    return blended / norm


def solve_ik_with_fallback(solver, target_pos, target_rot, frame_name,
                            warm_start, position_tolerance, orientation_tolerance,
                            use_orientation, enable_fallback):
    """Two-stage IK: full pose first, position-only fallback if orientation fails."""
    if use_orientation and target_rot is not None:
        actions, success = solver.compute_inverse_kinematics(
            target_position=target_pos,
            target_orientation=target_rot,
            frame_name=frame_name,
            warm_start=warm_start,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        )
        if success:
            return actions, True, "full"
        if not enable_fallback:
            return None, False, "failed"

    actions, success = solver.compute_inverse_kinematics(
        target_position=target_pos,
        target_orientation=None,
        frame_name=frame_name,
        warm_start=warm_start,
        position_tolerance=position_tolerance,
    )
    if success:
        return actions, True, "pos_only"
    return None, False, "failed"


# ── Cube helpers ──────────────────────────────────────────────────────────────
def get_table_surface_z(stage, fallback=TABLE_SURFACE_Z):
    """Detect table surface Z from the bottom of /box, falling back to table top."""
    try:
        cache = UsdGeom.BBoxCache(0, ["default", "render"])
        box_prim = stage.GetPrimAtPath("/box")
        if box_prim.IsValid():
            bbox  = cache.ComputeWorldBound(box_prim)
            z_min = bbox.GetRange().GetMin()[2]
            z_max = bbox.GetRange().GetMax()[2]
            print(f"[Scene] /box bbox  min_Z={z_min:.4f}  max_Z={z_max:.4f}")
            if 0.3 < z_min < 2.5:
                print(f"[Scene] Table surface = bottom of /box = {z_min:.4f} m")
                return z_min
        table_prim = stage.GetPrimAtPath("/packing_table_01")
        if table_prim.IsValid():
            bbox  = cache.ComputeWorldBound(table_prim)
            z_max = bbox.GetRange().GetMax()[2]
            print(f"[Scene] /packing_table_01 top = {z_max:.4f} m")
            if 0.3 < z_max < 2.5:
                return z_max
    except Exception as e:
        print(f"[Scene] Table Z auto-detect failed ({e}), using fallback")
    print(f"[Scene] Using TABLE_SURFACE_Z fallback = {fallback} m")
    return fallback


def get_table_center_xy(stage):
    """Return the XY centre of /packing_table_01 from its world bounding box."""
    try:
        cache = UsdGeom.BBoxCache(0, ["default", "render"])
        prim  = stage.GetPrimAtPath("/packing_table_01")
        if prim.IsValid():
            bbox = cache.ComputeWorldBound(prim)
            rng  = bbox.GetRange()
            cx   = (rng.GetMin()[0] + rng.GetMax()[0]) / 2.0
            cy   = (rng.GetMin()[1] + rng.GetMax()[1]) / 2.0
            print(f"[Scene] Table centre XY = ({cx:.4f}, {cy:.4f})")
            return np.array([cx, cy])
    except Exception as e:
        print(f"[Scene] Table centre detection failed ({e}), using (0, 0)")
    return np.zeros(2)


def delete_prim(stage, path):
    try:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            stage.RemovePrim(path)
            print(f"[Scene] Deleted: {path}")
        else:
            print(f"[Scene] Prim not found (skip): {path}")
    except Exception as e:
        print(f"[Scene] Could not delete {path}: {e}")


# ── Table offset / reset helpers ──────────────────────────────────────────────
def apply_offset_to_prim(stage, path, offset):
    """Shift a prim's translate xform op by `offset` (numpy [x,y,z])."""
    if np.linalg.norm(offset) < 1e-8:
        return
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"[Scene] Warning: {path} not found, offset skipped")
        return
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            current = np.array(op.Get(), dtype=float)
            op.Set(Gf.Vec3d(*(current + offset)))
            print(f"[Scene] Moved {path} by {offset} → {np.round(current + offset, 3)}")
            return
    print(f"[Scene] Warning: no xformOp:translate on {path}")


def capture_prim_xform(stage, path):
    """Save all xform ops of a prim (list of (name, value) tuples)."""
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    xform = UsdGeom.Xformable(prim)
    return [(op.GetOpName(), op.Get()) for op in xform.GetOrderedXformOps()]


def restore_prim_xform(stage, path, saved_ops):
    """Restore xform ops previously captured by capture_prim_xform."""
    if saved_ops is None:
        return
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"[Reset] Warning: {path} not found, skipped")
        return
    xform    = UsdGeom.Xformable(prim)
    ops_dict = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    for op_name, value in saved_ops:
        op = ops_dict.get(op_name)
        if op is not None:
            op.Set(value)
    print(f"[Reset] Restored: {path}")


def _cube_positions(table_center_xy, table_z):
    """Compute world positions for all 3 cubes centred on the table."""
    half = CUBE_SIZE / 2.0
    result = {}
    for name, offset in CUBE_OFFSETS.items():
        result[name] = np.array([
            table_center_xy[0] + offset[0],
            table_center_xy[1] + offset[1],
            table_z + half + 0.002,
        ])
    return result


def spawn_cubes(world, table_z, table_center_xy):
    """Add 3 dynamic cubes centred on the table. Returns dict name→cube."""
    try:
        from isaacsim.core.api.objects import DynamicCuboid
    except ImportError:
        try:
            from omni.isaac.core.objects import DynamicCuboid
        except ImportError:
            print("[Cubes] DynamicCuboid not found — using UsdGeom fallback")
            return _spawn_cubes_usd(world, table_z, table_center_xy)

    positions = _cube_positions(table_center_xy, table_z)
    cubes = {}
    for name, pos in positions.items():
        cube = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Cubes/{name}",
                name=name,
                position=pos,
                scale=np.array([CUBE_SIZE, CUBE_SIZE, CUBE_SIZE]),
                color=CUBE_COLORS[name],
                mass=0.2,
            )
        )
        cubes[name] = cube
        print(f"[Cubes] Spawned {name} at {np.round(pos, 3)}  color={CUBE_COLORS[name]}")
    return cubes


def _spawn_cubes_usd(world, table_z, table_center_xy):
    stage     = world.stage
    positions = _cube_positions(table_center_xy, table_z)
    cubes     = {}
    for name, pos in positions.items():
        prim_path = f"/World/Cubes/{name}"
        xform     = UsdGeom.Xform.Define(stage, prim_path)
        xform.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
        cube = UsdGeom.Cube.Define(stage, prim_path + "/Cube")
        cube.GetSizeAttr().Set(CUBE_SIZE)
        c = CUBE_COLORS[name]
        cube.GetDisplayColorAttr().Set([(c[0], c[1], c[2])])
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.MassAPI.Apply(xform.GetPrim()).GetMassAttr().Set(0.2)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        cubes[name] = xform
        print(f"[Cubes] Spawned (USD) {name} at {np.round(pos, 3)}")
    return cubes


def reset_cubes(world, cubes, table_z, table_center_xy):
    """Teleport cubes back to their table-centred start positions."""
    positions = _cube_positions(table_center_xy, table_z)
    for name, cube in cubes.items():
        pos = positions[name]
        try:
            cube.set_world_pose(position=pos, orientation=np.array([1, 0, 0, 0]))
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))
        except AttributeError:
            xform = UsdGeom.Xformable(cube.GetPrim() if hasattr(cube, 'GetPrim') else cube)
            for op in xform.GetOrderedXformOps():
                if "translate" in str(op.GetOpName()).lower():
                    op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    print("[Cubes] Reset to table centre positions")


def get_cube_position(cube):
    try:
        pos, _ = cube.get_world_pose()
        return np.array(pos)
    except Exception:
        return None


def check_stacking(cubes):
    """Return list of 'X ON Y' strings for cubes that are stacked."""
    names   = list(cubes.keys())
    stacked = []
    for top_name in names:
        top_pos = get_cube_position(cubes[top_name])
        if top_pos is None:
            continue
        for bot_name in names:
            if bot_name == top_name:
                continue
            bot_pos = get_cube_position(cubes[bot_name])
            if bot_pos is None:
                continue
            xy_dist = np.linalg.norm(top_pos[:2] - bot_pos[:2])
            z_diff  = top_pos[2] - bot_pos[2]
            if xy_dist < STACK_TOLERANCE_XY and abs(z_diff - CUBE_SIZE) < STACK_TOLERANCE_Z:
                stacked.append(f"{top_name} ON {bot_name}")
    return stacked


# ── Arm state ─────────────────────────────────────────────────────────────────
class ArmState:
    def __init__(self, name, arm_offset, transform_matrix):
        self.name                  = name
        self.arm_offset            = np.array(arm_offset)
        self.transform_matrix      = transform_matrix
        self.home_pos              = None
        self.target_pos            = np.array([0.3, 0.0, 0.3])
        self.target_rot            = np.array([1.0, 0.0, 0.0, 0.0])
        self.smoothed_pos          = np.array([0.3, 0.0, 0.3])
        self.smoothed_rot          = np.array([1.0, 0.0, 0.0, 0.0])
        self.gripper_closed        = False
        self.smoothed_gripper_pos  = 0.132
        self.calibrated            = False
        self.calibration_poses     = []
        self.reference_pos         = None
        self.pose_count            = 0


# ── ROS node ──────────────────────────────────────────────────────────────────
class BimanualQuestTeleop(Node):
    def __init__(self, config):
        super().__init__('isaac_openarm_cube_stack_teleop')
        self.config           = config
        self.T                = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        self.workspace_center = np.array(config["robot_workspace_center"])
        self.left_arm         = ArmState("left",  config["left_arm_offset"],  self.T)
        self.right_arm        = ArmState("right", config["right_arm_offset"], self.T)

        self.button_a_pressed = False
        self.button_b_pressed = False
        self.button_x_pressed = False
        self.button_y_pressed = False

        self.create_subscription(PoseStamped, '/quest/left_hand/pose',
            lambda msg: self.pose_callback(msg, self.left_arm), 10)
        self.create_subscription(Joy, '/quest/left_hand/inputs',
            lambda msg: self.input_callback(msg, self.left_arm, is_left=True), 10)
        self.create_subscription(PoseStamped, '/quest/right_hand/pose',
            lambda msg: self.pose_callback(msg, self.right_arm), 10)
        self.create_subscription(Joy, '/quest/right_hand/inputs',
            lambda msg: self.input_callback(msg, self.right_arm, is_left=False), 10)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.camera_pubs = {
            'head':        self.create_publisher(Image, '/camera/head/image_raw', 10),
            'wrist_left':  self.create_publisher(Image, '/camera/wrist_left/image_raw', 10),
            'wrist_right': self.create_publisher(Image, '/camera/wrist_right/image_raw', 10),
        }
        self.get_logger().info("BimanualQuestTeleop (Cube Stack v3) initialized")
        self.get_logger().info("Waiting for Quest controller data...")

    def pose_callback(self, msg, arm_state):
        arm_state.pose_count += 1
        xr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        if not arm_state.calibrated:
            arm_state.calibration_poses.append(xr_pos.copy())
            if len(arm_state.calibration_poses) >= self.config["calibration_samples"]:
                arm_state.reference_pos = np.mean(arm_state.calibration_poses, axis=0)
                arm_state.home_pos      = self.workspace_center + arm_state.arm_offset
                arm_state.target_pos    = arm_state.home_pos.copy()
                arm_state.smoothed_pos  = arm_state.home_pos.copy()
                arm_state.calibrated    = True
                self.get_logger().info(f"{arm_state.name.upper()} ARM CALIBRATION COMPLETE")
                self.get_logger().info(f"  Home position: {arm_state.home_pos}")
            return

        xr_offset  = xr_pos - arm_state.reference_pos
        robot_pos  = arm_state.transform_matrix @ xr_offset * self.config["pos_scale"] + arm_state.home_pos

        xr_quat    = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z, msg.pose.orientation.w])
        mat_robot  = (arm_state.transform_matrix
                      @ R.from_quat(xr_quat).as_matrix()
                      @ arm_state.transform_matrix.T
                      @ R.from_euler('x', 180, degrees=True).as_matrix())
        q          = R.from_matrix(mat_robot).as_quat()
        robot_rot  = np.array([q[3], q[0], q[1], q[2]])

        arm_state.target_pos = robot_pos
        arm_state.target_rot = robot_rot

    def input_callback(self, msg, arm_state, is_left):
        trigger = msg.axes[0] if len(msg.axes) > 0 else 0.0
        squeeze = msg.axes[1] if len(msg.axes) > 1 else 0.0
        arm_state.gripper_closed = (
            trigger > self.config["gripper_threshold"] or
            squeeze > self.config["gripper_threshold"]
        )
        btn_a_x = int(bool(msg.buttons[0])) if len(msg.buttons) > 0 else 0
        btn_b_y = int(bool(msg.buttons[1])) if len(msg.buttons) > 1 else 0
        if is_left:
            self.button_x_pressed = btn_a_x == 1
            self.button_y_pressed = btn_b_y == 1
        else:
            self.button_a_pressed = btn_a_x == 1
            self.button_b_pressed = btn_b_y == 1

    @property
    def both_calibrated(self):
        return self.left_arm.calibrated and self.right_arm.calibrated

    @property
    def reset_requested(self):
        return self.button_b_pressed or self.button_y_pressed


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    print("[Init] Warming up Isaac Sim...")
    for _ in range(30):
        simulation_app.update()

    print(f"[Init] Loading stage: {USD_PATH}")
    open_stage(USD_PATH)

    from omni.usd import get_context
    stage = get_context().get_stage()

    # Apply environment offset to the table (moves the whole table in the scene)
    env_offset = np.array(CONFIG.get("environment_object_offset", [0.0, 0.0, 0.0]), dtype=float)
    apply_offset_to_prim(stage, "/packing_table_01", env_offset)

    # Capture table state NOW (after offset) so reset restores to offset position
    table_saved_xform = capture_prim_xform(stage, "/packing_table_01")

    # Detect table surface Z and XY centre (both read after offset is applied)
    table_z          = get_table_surface_z(stage)
    table_center_xy  = get_table_center_xy(stage)
    delete_prim(stage, "/electric_screw_driver")
    delete_prim(stage, "/box")

    print("[Init] Stabilizing stage...")
    for _ in range(50):
        simulation_app.update()

    print("[Init] Creating World...")
    world = World(stage_units_in_meters=1.0)
    for _ in range(20):
        simulation_app.update()

    print(f"[Init] Spawning 3 cubes at table Z={table_z:.3f} m  centre={np.round(table_center_xy, 3)} ...")
    cubes = spawn_cubes(world, table_z, table_center_xy)

    # Find robot prim
    stage = world.stage
    robot_prim_path = None
    for path in ["/World/Robot", "/World/openarm", "/openarm", "/Robot",
                 "/Environment/openarm", "/World/openarm_bimanual"]:
        if stage.GetPrimAtPath(path).IsValid():
            robot_prim_path = path
            break

    if robot_prim_path is None:
        print("[ERROR] Robot not found in USD stage!")
        for p in stage.GetPseudoRoot().GetChildren():
            print(f"  - {p.GetPath()}")
        simulation_app.close()
        return

    print(f"[Init] Found robot at: {robot_prim_path}")
    openarm = world.scene.add(Articulation(prim_path=robot_prim_path, name="openarm"))

    # IK solvers
    print("[Init] Loading IK Solvers...")
    from omni.isaac.motion_generation import LulaKinematicsSolver

    left_desc  = os.path.join(LEFT_ARM_CONFIG_DIR,  "robot_descriptor.yaml")
    right_desc = os.path.join(RIGHT_ARM_CONFIG_DIR, "robot_descriptor.yaml")
    try:
        left_ik_solver  = LulaKinematicsSolver(robot_description_path=left_desc,  urdf_path=URDF_PATH)
        right_ik_solver = LulaKinematicsSolver(robot_description_path=right_desc, urdf_path=URDF_PATH)
        ik_enabled = True
        print("[Init] IK Solvers loaded!")

        try:
            fk_pos_l, _ = left_ik_solver.compute_forward_kinematics("openarm_left_hand",  np.zeros(7))
            fk_pos_r, _ = right_ik_solver.compute_forward_kinematics("openarm_right_hand", np.zeros(7))
            print(f"[FK] Left  EE at zero config: {np.round(fk_pos_l, 3)}")
            print(f"[FK] Right EE at zero config: {np.round(fk_pos_r, 3)}")
            auto_center = ((np.array(fk_pos_l) + np.array(fk_pos_r)) / 2.0).tolist()
            print(f"[FK] Auto workspace center:   {np.round(auto_center, 3)}")
            CONFIG["robot_workspace_center"] = auto_center
            CONFIG["left_arm_offset"]  = (np.array(fk_pos_l) - np.array(auto_center)).tolist()
            CONFIG["right_arm_offset"] = (np.array(fk_pos_r) - np.array(auto_center)).tolist()
            print(f"[FK] Left  offset: {np.round(CONFIG['left_arm_offset'],  3)}")
            print(f"[FK] Right offset: {np.round(CONFIG['right_arm_offset'], 3)}")
        except Exception as fk_e:
            print(f"[FK] Could not compute FK: {fk_e}")
    except Exception as e:
        print(f"[WARNING] IK Solvers unavailable: {e}")
        ik_enabled = False

    # Hide UI panels
    import omni.ui
    for win_name in ["Stage", "Layer", "Render Settings", "Content", "Content Library",
                     "Console", "Property", "Properties", "Semantics", "Visual Scripting"]:
        try:
            w = omni.ui.Workspace.get_window(win_name)
            if w:
                w.visible = False
        except Exception:
            pass

    print("[Init] Resetting World...")
    world.reset()
    for _ in range(20):
        simulation_app.update()

    # Joint indices
    dof_names          = openarm.dof_names
    left_arm_indices   = [i for i, n in enumerate(dof_names) if n in LEFT_ARM_JOINTS]
    right_arm_indices  = [i for i, n in enumerate(dof_names) if n in RIGHT_ARM_JOINTS]
    left_gripper_idx   = [i for i, n in enumerate(dof_names) if n in LEFT_GRIPPER_JOINTS]
    right_gripper_idx  = [i for i, n in enumerate(dof_names) if n in RIGHT_GRIPPER_JOINTS]
    print(f"[Info] Left arm:    {left_arm_indices}")
    print(f"[Info] Right arm:   {right_arm_indices}")
    print(f"[Info] L gripper:   {left_gripper_idx}")
    print(f"[Info] R gripper:   {right_gripper_idx}")

    # ROS node
    rclpy.init()
    teleop_node = BimanualQuestTeleop(CONFIG)

    print("=" * 60)
    print("  OpenArm Cube Stack Teleop  (v3 two-stage IK)")
    print("  3 cubes on table: Red | Green | Blue")
    print("  Trigger / Grip = Gripper  |  A = Camera cycle")
    print("  B / Y button or R key = Reset cubes")
    print("=" * 60)

    # Keyboard
    input_iface = keyboard = kb_sub = None
    try:
        appwindow   = omni.appwindow.get_default_app_window()
        input_iface = carb.input.acquire_input_interface()
        keyboard    = appwindow.get_keyboard()
        kb_sub      = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
        print("[Init] Keyboard registered: R resets cubes")
    except Exception as e:
        print(f"[Init] Keyboard registration skipped: {e}")

    # Camera enumeration
    import omni.kit.viewport.utility
    cameras      = ["/OmniverseKit_Persp"]
    camera_names = ["Perspective"]
    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if prim.IsA(UsdGeom.Camera) and path_str not in \
                ["/OmniverseKit_Persp", "/OmniverseKit_Front", "/OmniverseKit_Right"]:
            cameras.append(path_str)
            camera_names.append(path_str.split("/")[-1].replace("_", " ").title())
    print(f"[Camera] Found {len(cameras)} cameras: {camera_names}")
    current_cam_index = 0
    last_button_a     = False

    # Async camera publisher
    import omni.replicator.core as rep
    import threading
    import queue

    RECORDING_CAMERAS = {
        'head':        '/openarm/openarm_body_link/head_camera',
        'wrist_left':  '/openarm/openarm_left_link7/left_wrist_camera',
        'wrist_right': '/openarm/openarm_right_link7/right_wrist_camera',
    }
    CAMERA_RESOLUTION = (480, 360)

    camera_annotators = {}
    for cam_name, cam_path in RECORDING_CAMERAS.items():
        if stage.GetPrimAtPath(cam_path).IsValid():
            rp  = rep.create.render_product(cam_path, CAMERA_RESOLUTION)
            ann = rep.AnnotatorRegistry.get_annotator("rgb")
            ann.attach([rp])
            camera_annotators[cam_name] = ann
            print(f"[Camera] Setup: {cam_name} at {CAMERA_RESOLUTION}")
        else:
            print(f"[Camera] Warning: {cam_path} not found")

    camera_queue          = queue.Queue(maxsize=3)
    camera_thread_running = True

    def camera_publish_thread():
        while camera_thread_running:
            try:
                cam_name, img_rgb, ts = camera_queue.get(timeout=0.1)
                msg                   = Image()
                msg.header.stamp      = ts
                msg.header.frame_id   = cam_name
                msg.height, msg.width = img_rgb.shape[:2]
                msg.encoding          = 'rgb8'
                msg.step              = img_rgb.shape[1] * 3
                msg.data              = img_rgb.tobytes()
                teleop_node.camera_pubs[cam_name].publish(msg)
                camera_queue.task_done()
            except queue.Empty:
                continue
            except Exception:
                pass

    threading.Thread(target=camera_publish_thread, daemon=True).start()
    print("[Camera] Async camera publisher started")

    # IK tuning params
    use_orientation_ik    = CONFIG.get("use_orientation_ik",      True)
    enable_fallback       = CONFIG.get("ik_orientation_fallback",  True)
    position_tolerance    = CONFIG.get("position_tolerance",       0.01)
    orientation_tolerance = CONFIG.get("orientation_tolerance",    0.7)

    # Loop state
    left_full  = left_pos  = left_fail  = 0
    right_full = right_pos = right_fail = 0
    last_left_arm_positions  = None
    last_right_arm_positions = None
    frame_counter     = 0
    CAMERA_EVERY      = 3
    STACK_CHECK_EVERY = 30
    last_reset_button = False
    last_stack_msg    = ""

    # ── Main loop ─────────────────────────────────────────────────────────────
    while simulation_app.is_running():
        rclpy.spin_once(teleop_node, timeout_sec=0.0)

        # Camera cycle (A button)
        if teleop_node.button_a_pressed and not last_button_a:
            current_cam_index = (current_cam_index + 1) % len(cameras)
            vp = omni.kit.viewport.utility.get_active_viewport()
            if vp:
                try:
                    vp.camera_path = cameras[current_cam_index]
                    print(f"[Camera] → {camera_names[current_cam_index]}")
                except Exception:
                    pass
        last_button_a = teleop_node.button_a_pressed

        # Scene reset (keyboard R or B/Y button) — restores table + cubes together
        def do_reset():
            restore_prim_xform(stage, "/packing_table_01", table_saved_xform)
            reset_cubes(world, cubes, table_z, table_center_xy)

        if kb_state.reset_requested:
            kb_state.reset_requested = False
            do_reset()

        reset_now = teleop_node.reset_requested
        if reset_now and not last_reset_button:
            do_reset()
        last_reset_button = reset_now

        # Wait for calibration
        if not teleop_node.both_calibrated:
            if teleop_node.left_arm.pose_count > 0 or teleop_node.right_arm.pose_count > 0:
                l_s = ("CALIBRATED" if teleop_node.left_arm.calibrated
                       else f"{len(teleop_node.left_arm.calibration_poses)}/30")
                r_s = ("CALIBRATED" if teleop_node.right_arm.calibrated
                       else f"{len(teleop_node.right_arm.calibration_poses)}/30")
                total = teleop_node.left_arm.pose_count + teleop_node.right_arm.pose_count
                if total % 30 == 1:
                    print(f"[Calibration] Left: {l_s}  Right: {r_s}")
            else:
                if not hasattr(teleop_node, '_last_wait'):
                    teleop_node._last_wait = 0
                now = time.time()
                if now - teleop_node._last_wait > 2.0:
                    print("[Waiting] No Quest data yet — is the Quest ROS2 bridge running?")
                    print("          Check: ros2 topic list | grep quest")
                    teleop_node._last_wait = now
            world.step(render=True)
            continue

        current_positions = openarm.get_joint_positions()
        if current_positions is None:
            world.step(render=True)
            continue

        target_positions = current_positions.copy()

        # Smoothing
        alpha = CONFIG["smoothing"]
        if alpha > 0:
            teleop_node.left_arm.smoothed_pos  = alpha * teleop_node.left_arm.smoothed_pos  + (1 - alpha) * teleop_node.left_arm.target_pos
            teleop_node.right_arm.smoothed_pos = alpha * teleop_node.right_arm.smoothed_pos + (1 - alpha) * teleop_node.right_arm.target_pos
            teleop_node.left_arm.smoothed_rot  = smooth_quaternion(teleop_node.left_arm.smoothed_rot,  teleop_node.left_arm.target_rot,  alpha)
            teleop_node.right_arm.smoothed_rot = smooth_quaternion(teleop_node.right_arm.smoothed_rot, teleop_node.right_arm.target_rot, alpha)
        else:
            teleop_node.left_arm.smoothed_pos  = teleop_node.left_arm.target_pos
            teleop_node.left_arm.smoothed_rot  = teleop_node.left_arm.target_rot
            teleop_node.right_arm.smoothed_pos = teleop_node.right_arm.target_pos
            teleop_node.right_arm.smoothed_rot = teleop_node.right_arm.target_rot

        frame_count = left_full + left_pos + left_fail + right_full + right_pos + right_fail
        if CONFIG.get("debug_ik") and frame_count % 100 == 0:
            print(f"[IK Debug] L pos={teleop_node.left_arm.smoothed_pos}")
            print(f"[IK Debug] R pos={teleop_node.right_arm.smoothed_pos}")

        # Left arm IK
        if ik_enabled:
            ws_l = last_left_arm_positions if last_left_arm_positions is not None else LEFT_ARM_PREFERRED_CONFIG
            left_actions, left_ok, left_mode = solve_ik_with_fallback(
                left_ik_solver,
                teleop_node.left_arm.smoothed_pos,
                teleop_node.left_arm.smoothed_rot,
                "openarm_left_hand",
                ws_l, position_tolerance, orientation_tolerance,
                use_orientation_ik, enable_fallback,
            )
            if left_ok:
                if left_mode == "full": left_full += 1
                else:                   left_pos  += 1
                lp = np.array(left_actions).flatten()[:7]
                last_left_arm_positions = lp.copy()
                for i, idx in enumerate(left_arm_indices):
                    if i < len(lp):
                        target_positions[idx] = lp[i]
            else:
                left_fail += 1
                if left_fail <= 3:
                    print(f"[IK] Left arm FAIL for target: {teleop_node.left_arm.smoothed_pos}")
                if last_left_arm_positions is not None:
                    for i, idx in enumerate(left_arm_indices):
                        if i < len(last_left_arm_positions):
                            target_positions[idx] = last_left_arm_positions[i]

        # Right arm IK
        if ik_enabled:
            ws_r = last_right_arm_positions if last_right_arm_positions is not None else RIGHT_ARM_PREFERRED_CONFIG
            right_actions, right_ok, right_mode = solve_ik_with_fallback(
                right_ik_solver,
                teleop_node.right_arm.smoothed_pos,
                teleop_node.right_arm.smoothed_rot,
                "openarm_right_hand",
                ws_r, position_tolerance, orientation_tolerance,
                use_orientation_ik, enable_fallback,
            )
            if right_ok:
                if right_mode == "full": right_full += 1
                else:                    right_pos  += 1
                rp = np.array(right_actions).flatten()[:7]
                last_right_arm_positions = rp.copy()
                for i, idx in enumerate(right_arm_indices):
                    if i < len(rp):
                        target_positions[idx] = rp[i]
            else:
                right_fail += 1
                if right_fail <= 3:
                    print(f"[IK] Right arm FAIL for target: {teleop_node.right_arm.smoothed_pos}")
                if last_right_arm_positions is not None:
                    for i, idx in enumerate(right_arm_indices):
                        if i < len(last_right_arm_positions):
                            target_positions[idx] = last_right_arm_positions[i]

        # Periodic IK quality summary
        if frame_count > 0 and frame_count % 1200 == 0:
            l_t = max(1, left_full  + left_pos  + left_fail)
            r_t = max(1, right_full + right_pos + right_fail)
            print(f"[IK Stats] Left  full={100*left_full/l_t:.0f}% pos-only={100*left_pos/l_t:.0f}% fail={100*left_fail/l_t:.0f}%")
            print(f"[IK Stats] Right full={100*right_full/r_t:.0f}% pos-only={100*right_pos/r_t:.0f}% fail={100*right_fail/r_t:.0f}%")

        # Gripper
        gs  = CONFIG["gripper_speed"]
        gop = CONFIG["gripper_open_pos"]
        gcl = CONFIG["gripper_closed_pos"]

        l_tgt = gcl if teleop_node.left_arm.gripper_closed else gop
        if teleop_node.left_arm.smoothed_gripper_pos < l_tgt:
            teleop_node.left_arm.smoothed_gripper_pos = min(teleop_node.left_arm.smoothed_gripper_pos + gs, l_tgt)
        else:
            teleop_node.left_arm.smoothed_gripper_pos = max(teleop_node.left_arm.smoothed_gripper_pos - gs, l_tgt)
        for idx in left_gripper_idx:
            target_positions[idx] = teleop_node.left_arm.smoothed_gripper_pos

        r_tgt = gcl if teleop_node.right_arm.gripper_closed else gop
        if teleop_node.right_arm.smoothed_gripper_pos < r_tgt:
            teleop_node.right_arm.smoothed_gripper_pos = min(teleop_node.right_arm.smoothed_gripper_pos + gs, r_tgt)
        else:
            teleop_node.right_arm.smoothed_gripper_pos = max(teleop_node.right_arm.smoothed_gripper_pos - gs, r_tgt)
        for idx in right_gripper_idx:
            target_positions[idx] = teleop_node.right_arm.smoothed_gripper_pos

        openarm.apply_action(ArticulationAction(joint_positions=target_positions))

        # Joint state publish
        js          = JointState()
        js.header.stamp = teleop_node.get_clock().now().to_msg()
        js.name     = list(dof_names)
        js.position = target_positions.tolist()
        teleop_node.joint_state_pub.publish(js)

        frame_counter += 1

        # Stacking check
        if frame_counter % STACK_CHECK_EVERY == 0:
            stacked = check_stacking(cubes)
            msg_str = ", ".join(stacked) if stacked else ""
            if msg_str != last_stack_msg:
                if stacked:
                    print(f"[Stack] {msg_str}")
                last_stack_msg = msg_str

        # Camera capture (async)
        if frame_counter % CAMERA_EVERY == 0:
            ts = teleop_node.get_clock().now().to_msg()
            for cam_name, annotator in camera_annotators.items():
                try:
                    data = annotator.get_data()
                    if data is not None and len(data) > 0:
                        img_rgb = (data[:, :, :3].astype(np.uint8)
                                   if data.shape[2] == 4 else data.astype(np.uint8))
                        try:
                            camera_queue.put_nowait((cam_name, img_rgb.copy(), ts))
                        except queue.Full:
                            pass
                except Exception:
                    pass

        world.step(render=True)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    camera_thread_running = False
    print("\n" + "=" * 60)
    print("Session Statistics:")
    l_t = max(1, left_full  + left_pos  + left_fail)
    r_t = max(1, right_full + right_pos + right_fail)
    print(f"  Left  full={left_full}  pos-only={left_pos}  fail={left_fail}  "
          f"({100*left_full/l_t:.1f}% / {100*left_pos/l_t:.1f}% / {100*left_fail/l_t:.1f}%)")
    print(f"  Right full={right_full} pos-only={right_pos} fail={right_fail}  "
          f"({100*right_full/r_t:.1f}% / {100*right_pos/r_t:.1f}% / {100*right_fail/r_t:.1f}%)")
    print("=" * 60)

    if input_iface is not None and keyboard is not None and kb_sub is not None:
        try:
            input_iface.unsubscribe_to_keyboard_events(keyboard, kb_sub)
        except Exception:
            pass

    teleop_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
