# isaac_cube_stack_teleop.py
# OpenArm Bimanual Teleoperation — Cube Stack Task
#
# Same robot/teleop/deadman logic as isaac_openarm_teleop.py
# New: 3 colored cubes on the table (Red, Green, Blue)
#       B/Y button resets cubes to start positions
#       Terminal prints stacking status
#
# Controls:
#   Squeeze (Grip)  → Deadman: hold to move arm, release to freeze
#   Trigger         → Gripper open/close
#   A button        → Cycle camera
#   B/Y button      → Reset cubes to start positions

from isaacsim import SimulationApp

simulation_app = SimulationApp({
    "headless": False,
    "width": 1920,
    "height": 1080,
    "window_width": 1920,
    "window_height": 1080,
})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.isaac.ros2_bridge")

try:
    import rclpy
except ImportError:
    print("ERROR: rclpy not found. Run via terminal2_isaac_teleop.sh (no system ROS sourced).")
    raise

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, JointState, Image
import numpy as np
from scipy.spatial.transform import Rotation as R
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.prims import SingleArticulation as Articulation
from pxr import UsdGeom, Gf
import os
import yaml
import time

# ── Config paths ──────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH  = os.path.join(PROJECT_ROOT, "config", "config.yaml")

with open(CONFIG_PATH, 'r') as f:
    PATH_CONFIG = yaml.safe_load(f)

USD_PATH          = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['usd'])
URDF_PATH         = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['urdf'])
LEFT_ARM_CONFIG   = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['left_arm_config'])
RIGHT_ARM_CONFIG  = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['right_arm_config'])

# ── Task / robot config ───────────────────────────────────────────────────────
CONFIG = {
    "pos_scale":              1.0,
    "robot_workspace_center": [0.3, 0.0, 0.3],
    "left_arm_offset":        [0.0,  0.15, 0.0],
    "right_arm_offset":       [0.0, -0.15, 0.0],
    "smoothing":              0.9,
    "gripper_threshold":      0.5,
    "gripper_open_pos":       0.132,
    "gripper_closed_pos":    -1,
    "gripper_speed":          0.05,
    "calibration_samples":    30,
    "debug_ik":               False,
    "deadman_threshold":      0.3,
}

# ── Cube task config ──────────────────────────────────────────────────────────
CUBE_SIZE          = 0.05      # 5 cm
TABLE_SURFACE_Z    = 0.99      # fallback — derived from box Z=1.19 in scene
STACK_TOLERANCE_XY = 0.04      # horizontal tolerance for "stacked" check
STACK_TOLERANCE_Z  = 0.03      # vertical tolerance for "stacked" check

# Box was at X=0.034, Y=-0.012 on the table (from scene inspector).
# Cubes are spread around that same XY area.
# Z is filled at runtime from table surface detection.
BOX_CENTER_X = 0.034
BOX_CENTER_Y = -0.012
CUBE_SPACING = 0.10   # metres between cube centres

CUBE_START_POSITIONS = {
    "cubeA": np.array([BOX_CENTER_X + CUBE_SPACING, BOX_CENTER_Y, 0.0]),  # Red   — right
    "cubeB": np.array([BOX_CENTER_X,                BOX_CENTER_Y, 0.0]),  # Green — centre
    "cubeC": np.array([BOX_CENTER_X - CUBE_SPACING, BOX_CENTER_Y, 0.0]),  # Blue  — left
}
CUBE_COLORS = {
    "cubeA": np.array([0.8, 0.1, 0.1]),   # Red
    "cubeB": np.array([0.1, 0.7, 0.1]),   # Green
    "cubeC": np.array([0.1, 0.3, 0.9]),   # Blue
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


# ═════════════════════════════════════════════════════════════════════════════
# Arm state
# ═════════════════════════════════════════════════════════════════════════════
class ArmState:
    def __init__(self, name, arm_offset, transform_matrix):
        self.name             = name
        self.arm_offset       = np.array(arm_offset)
        self.transform_matrix = transform_matrix
        self.home_pos         = None
        self.target_pos       = np.array([0.3, 0.0, 0.3])
        self.target_rot       = np.array([1.0, 0.0, 0.0, 0.0])
        self.smoothed_pos     = np.array([0.3, 0.0, 0.3])
        self.smoothed_rot     = np.array([1.0, 0.0, 0.0, 0.0])
        self.gripper_closed   = False
        self.smoothed_gripper_pos = 0.132
        self.calibrated       = False
        self.calibration_poses = []
        self.reference_pos    = None
        self.pose_count       = 0
        # Deadman
        self.deadman_active   = False
        self.frozen_pos       = None
        self.frozen_rot       = None


# ═════════════════════════════════════════════════════════════════════════════
# ROS node
# ═════════════════════════════════════════════════════════════════════════════
class BimanualCubeStackTeleop(Node):
    def __init__(self, config):
        super().__init__('isaac_cube_stack_teleop')
        self.config = config
        self.T = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        self.workspace_center = np.array(config["robot_workspace_center"])
        self.left_arm  = ArmState("left",  config["left_arm_offset"],  self.T)
        self.right_arm = ArmState("right", config["right_arm_offset"], self.T)

        self.button_a_pressed = False
        self.button_b_pressed = False   # right B → reset cubes
        self.button_x_pressed = False
        self.button_y_pressed = False   # left  Y → reset cubes

        # Subscriptions
        self.create_subscription(PoseStamped, '/quest/left_hand/pose',
            lambda m: self.pose_callback(m, self.left_arm), 10)
        self.create_subscription(Joy, '/quest/left_hand/inputs',
            lambda m: self.input_callback(m, self.left_arm, is_left=True), 10)
        self.create_subscription(PoseStamped, '/quest/right_hand/pose',
            lambda m: self.pose_callback(m, self.right_arm), 10)
        self.create_subscription(Joy, '/quest/right_hand/inputs',
            lambda m: self.input_callback(m, self.right_arm, is_left=False), 10)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.camera_pubs = {
            'head':        self.create_publisher(Image, '/camera/head/image_raw', 10),
            'wrist_left':  self.create_publisher(Image, '/camera/wrist_left/image_raw', 10),
            'wrist_right': self.create_publisher(Image, '/camera/wrist_right/image_raw', 10),
        }
        self.get_logger().info("CubeStack Teleop ready — 3 cubes loaded on table")
        self.get_logger().info("  B/Y button: reset cubes to start positions")

    def pose_callback(self, msg, arm_state):
        arm_state.pose_count += 1
        xr_pos = np.array([msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z])
        # Calibration
        if not arm_state.calibrated:
            arm_state.calibration_poses.append(xr_pos.copy())
            if len(arm_state.calibration_poses) >= self.config["calibration_samples"]:
                arm_state.reference_pos = np.mean(arm_state.calibration_poses, axis=0)
                arm_state.home_pos      = self.workspace_center + arm_state.arm_offset
                arm_state.target_pos    = arm_state.home_pos.copy()
                arm_state.smoothed_pos  = arm_state.home_pos.copy()
                arm_state.calibrated    = True
                self.get_logger().info(
                    f"{arm_state.name.upper()} ARM CALIBRATED — home: {arm_state.home_pos}")
            return

        # Transform VR → robot
        xr_offset  = xr_pos - arm_state.reference_pos
        robot_pos  = (arm_state.transform_matrix @ xr_offset) * self.config["pos_scale"] \
                     + arm_state.home_pos
        xr_quat    = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                                msg.pose.orientation.z, msg.pose.orientation.w])
        mat_robot  = arm_state.transform_matrix \
                     @ R.from_quat(xr_quat).as_matrix() \
                     @ arm_state.transform_matrix.T \
                     @ R.from_euler('x', 180, degrees=True).as_matrix()
        q          = R.from_matrix(mat_robot).as_quat()
        robot_rot  = np.array([q[3], q[0], q[1], q[2]])

        # Deadman gate
        if arm_state.deadman_active:
            arm_state.target_pos = robot_pos
            arm_state.target_rot = robot_rot
            arm_state.frozen_pos = robot_pos.copy()
            arm_state.frozen_rot = robot_rot.copy()
        else:
            if arm_state.frozen_pos is not None:
                arm_state.target_pos = arm_state.frozen_pos.copy()
                arm_state.target_rot = arm_state.frozen_rot.copy()

    def input_callback(self, msg, arm_state, is_left):
        trigger = msg.axes[0] if len(msg.axes) > 0 else 0.0
        squeeze = msg.axes[1] if len(msg.axes) > 1 else 0.0

        # Deadman
        was = arm_state.deadman_active
        arm_state.deadman_active = squeeze > self.config["deadman_threshold"]
        if was and not arm_state.deadman_active:
            self.get_logger().info(f"[Deadman] {arm_state.name.upper()} FROZEN")
        elif not was and arm_state.deadman_active:
            self.get_logger().info(f"[Deadman] {arm_state.name.upper()} ACTIVE")

        # Gripper (trigger only)
        arm_state.gripper_closed = trigger > self.config["gripper_threshold"]

        # Buttons
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


# ═════════════════════════════════════════════════════════════════════════════
# Helpers
# ═════════════════════════════════════════════════════════════════════════════
def get_table_surface_z(stage, fallback=TABLE_SURFACE_Z):
    """
    Detect table surface Z from the bottom of the /box prim.
    The box sits ON the table, so its bottom face = table surface.
    Box centre was observed at Z=1.19181 in the scene.
    """
    try:
        from pxr import UsdGeom
        cache = UsdGeom.BBoxCache(0, ["default", "render"])

        # Primary: use /box bottom as table surface reference
        box_prim = stage.GetPrimAtPath("/box")
        if box_prim.IsValid():
            bbox  = cache.ComputeWorldBound(box_prim)
            z_min = bbox.GetRange().GetMin()[2]
            z_max = bbox.GetRange().GetMax()[2]
            print(f"[Scene] /box bbox  min_Z={z_min:.4f}  max_Z={z_max:.4f}")
            if 0.3 < z_min < 2.5:
                print(f"[Scene] Table surface = bottom of /box = {z_min:.4f} m")
                return z_min

        # Secondary: use packing table prim bbox top
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


def delete_prim(stage, path):
    """Permanently remove a prim from the stage."""
    try:
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            stage.RemovePrim(path)
            print(f"[Scene] Deleted: {path}")
        else:
            print(f"[Scene] Prim not found (already gone?): {path}")
    except Exception as e:
        print(f"[Scene] Could not delete {path}: {e}")


def spawn_cubes(world, table_z):
    """Add 3 dynamic cubes to the world scene. Returns dict name→prim."""
    try:
        from isaacsim.core.api.objects import DynamicCuboid
    except ImportError:
        try:
            from omni.isaac.core.objects import DynamicCuboid
        except ImportError:
            print("[Cubes] WARNING: DynamicCuboid not found — using UsdGeom fallback")
            return spawn_cubes_usd(world, table_z)

    half = CUBE_SIZE / 2.0
    cubes = {}
    for name, xy_pos in CUBE_START_POSITIONS.items():
        pos = xy_pos.copy()
        pos[2] = table_z + half + 0.002   # tiny gap so cube doesn't clip table
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
        print(f"[Cubes] Spawned {name} at {pos}  color={CUBE_COLORS[name]}")
    return cubes


def spawn_cubes_usd(world, table_z):
    """Fallback: add cubes using raw USD prims if DynamicCuboid unavailable."""
    from pxr import UsdGeom, UsdPhysics, Gf
    stage = world.stage
    cubes = {}
    half = CUBE_SIZE / 2.0

    for name, xy_pos in CUBE_START_POSITIONS.items():
        prim_path = f"/World/Cubes/{name}"
        pos = xy_pos.copy()
        pos[2] = table_z + half + 0.002

        xform = UsdGeom.Xform.Define(stage, prim_path)
        xform.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))

        cube = UsdGeom.Cube.Define(stage, prim_path + "/Cube")
        cube.GetSizeAttr().Set(CUBE_SIZE)

        # Color
        c = CUBE_COLORS[name]
        cube.GetDisplayColorAttr().Set([(c[0], c[1], c[2])])

        # Physics
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())
        UsdPhysics.MassAPI.Apply(xform.GetPrim()).GetMassAttr().Set(0.2)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())

        cubes[name] = xform
        print(f"[Cubes] Spawned (USD) {name} at {pos}")
    return cubes


def reset_cubes(world, cubes, table_z):
    """Teleport cubes back to their start positions."""
    half = CUBE_SIZE / 2.0
    for name, cube in cubes.items():
        pos = CUBE_START_POSITIONS[name].copy()
        pos[2] = table_z + half + 0.002
        try:
            # DynamicCuboid API
            cube.set_world_pose(position=pos, orientation=np.array([1, 0, 0, 0]))
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))
        except AttributeError:
            # USD Xform fallback
            from pxr import Gf, UsdGeom
            xform = UsdGeom.Xformable(cube.GetPrim() if hasattr(cube, 'GetPrim') else cube)
            ops = xform.GetOrderedXformOps()
            for op in ops:
                if "translate" in str(op.GetOpName()).lower():
                    op.Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))
    print("[Cubes] Reset to start positions")


def get_cube_position(cube):
    """Get cube world position as numpy array."""
    try:
        pos, _ = cube.get_world_pose()
        return np.array(pos)
    except Exception:
        return None


def check_stacking(cubes):
    """Print stacking status. Returns list of stack descriptions."""
    names   = list(cubes.keys())
    stacked = []
    for i, top_name in enumerate(names):
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


# ═════════════════════════════════════════════════════════════════════════════
# Main
# ═════════════════════════════════════════════════════════════════════════════
def main():
    # ── Isaac Sim warmup ──────────────────────────────────────────────────────
    print("[Init] Warming up Isaac Sim...")
    for _ in range(30):
        simulation_app.update()

    # ── Load stage ────────────────────────────────────────────────────────────
    print(f"[Init] Loading stage: {USD_PATH}")
    open_stage(USD_PATH)
    for _ in range(50):
        simulation_app.update()

    # ── World ─────────────────────────────────────────────────────────────────
    print("[Init] Creating World...")
    world = World(stage_units_in_meters=1.0)
    for _ in range(20):
        simulation_app.update()

    stage = world.stage

    # ── Delete existing objects (box + screwdriver) ───────────────────────────
    # Must happen BEFORE get_table_surface_z so BBoxCache can still read /box
    table_z = get_table_surface_z(stage)
    delete_prim(stage, "/electric_screw_driver")
    delete_prim(stage, "/box")

    # ── Spawn cubes ───────────────────────────────────────────────────────────
    print(f"[Init] Spawning 3 cubes at table Z = {table_z:.3f} m ...")
    cubes = spawn_cubes(world, table_z)

    # ── Load robot ────────────────────────────────────────────────────────────
    from pxr import Usd
    robot_prim_path = None
    for path in ["/World/Robot", "/World/openarm", "/openarm", "/Robot"]:
        if stage.GetPrimAtPath(path).IsValid():
            robot_prim_path = path
            break

    if robot_prim_path is None:
        print("[ERROR] Robot not found!")
        simulation_app.close()
        return

    print(f"[Init] Robot at: {robot_prim_path}")
    openarm = world.scene.add(Articulation(prim_path=robot_prim_path, name="openarm"))

    # ── IK solvers ────────────────────────────────────────────────────────────
    from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
    try:
        left_ik  = LulaKinematicsSolver(
            robot_description_path=os.path.join(LEFT_ARM_CONFIG,  "robot_descriptor.yaml"),
            urdf_path=URDF_PATH)
        right_ik = LulaKinematicsSolver(
            robot_description_path=os.path.join(RIGHT_ARM_CONFIG, "robot_descriptor.yaml"),
            urdf_path=URDF_PATH)
        ik_enabled = True
        print("[Init] IK solvers loaded")
    except Exception as e:
        print(f"[WARNING] IK unavailable: {e}")
        ik_enabled = False

    # ── Minimise UI panels ────────────────────────────────────────────────────
    import omni.ui
    for name in ["Stage", "Layer", "Render Settings", "Content", "Console",
                 "Property", "Properties", "Semantics"]:
        try:
            w = omni.ui.Workspace.get_window(name)
            if w:
                w.visible = False
        except Exception:
            pass

    world.reset()
    for _ in range(20):
        simulation_app.update()

    # ── Joint indices ─────────────────────────────────────────────────────────
    dof_names         = openarm.dof_names
    left_arm_idx      = [i for i, n in enumerate(dof_names) if n in LEFT_ARM_JOINTS]
    right_arm_idx     = [i for i, n in enumerate(dof_names) if n in RIGHT_ARM_JOINTS]
    left_gripper_idx  = [i for i, n in enumerate(dof_names) if n in LEFT_GRIPPER_JOINTS]
    right_gripper_idx = [i for i, n in enumerate(dof_names) if n in RIGHT_GRIPPER_JOINTS]
    print(f"[Info] Left arm: {left_arm_idx} | Right arm: {right_arm_idx}")

    # ── ROS node ──────────────────────────────────────────────────────────────
    rclpy.init()
    node = BimanualCubeStackTeleop(CONFIG)

    print("=" * 60)
    print("  OpenArm Cube Stack Teleop")
    print("  3 cubes on table: Red | Green | Blue")
    print("  Squeeze = Deadman  |  Trigger = Gripper")
    print("  A = Camera cycle   |  B/Y = Reset cubes")
    print("=" * 60)

    # ── Camera setup ──────────────────────────────────────────────────────────
    import omni.kit.viewport.utility
    from pxr import UsdGeom as UG
    cameras      = ["/OmniverseKit_Persp"]
    camera_names = ["Perspective"]
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if prim.IsA(UG.Camera) and path not in \
                ["/OmniverseKit_Persp", "/OmniverseKit_Front", "/OmniverseKit_Right"]:
            cameras.append(path)
            camera_names.append(path.split("/")[-1].replace("_", " ").title())
    print(f"[Camera] {len(cameras)} cameras: {camera_names}")
    current_cam = 0
    last_btn_a  = False

    # ── Async camera thread ───────────────────────────────────────────────────
    import omni.replicator.core as rep
    import threading, queue
    RECORDING_CAMERAS = {
        'head':        '/openarm/openarm_body_link/head_camera',
        'wrist_left':  '/openarm/openarm_left_link7/left_wrist_camera',
        'wrist_right': '/openarm/openarm_right_link7/right_wrist_camera',
    }
    CAMERA_RES = (480, 360)
    cam_annotators = {}
    for cam_name, cam_path in RECORDING_CAMERAS.items():
        if stage.GetPrimAtPath(cam_path).IsValid():
            rp = rep.create.render_product(cam_path, CAMERA_RES)
            ann = rep.AnnotatorRegistry.get_annotator("rgb")
            ann.attach([rp])
            cam_annotators[cam_name] = ann

    cam_queue          = queue.Queue(maxsize=3)
    cam_thread_running = True

    def cam_thread_fn():
        while cam_thread_running:
            try:
                cname, img, ts = cam_queue.get(timeout=0.1)
                msg = Image()
                msg.header.stamp   = ts
                msg.header.frame_id = cname
                msg.height, msg.width = img.shape[:2]
                msg.encoding       = 'rgb8'
                msg.step           = img.shape[1] * 3
                msg.data           = img.tobytes()
                node.camera_pubs[cname].publish(msg)
                cam_queue.task_done()
            except queue.Empty:
                continue
            except Exception:
                pass

    threading.Thread(target=cam_thread_fn, daemon=True).start()

    # ── Tracking vars ─────────────────────────────────────────────────────────
    left_ik_success = left_ik_fail = right_ik_success = right_ik_fail = 0
    last_left_pos   = last_right_pos = None
    frame_counter   = 0
    CAPTURE_EVERY   = 2
    last_reset_btn  = False
    last_stack_msg  = ""
    stack_check_interval = 30   # frames between stack checks

    # ── Main loop ─────────────────────────────────────────────────────────────
    while simulation_app.is_running():
        rclpy.spin_once(node, timeout_sec=0.0)

        # Camera cycle (A button)
        if node.button_a_pressed and not last_btn_a:
            current_cam = (current_cam + 1) % len(cameras)
            vp = omni.kit.viewport.utility.get_active_viewport()
            if vp:
                try:
                    vp.camera_path = cameras[current_cam]
                    print(f"[Camera] → {camera_names[current_cam]}")
                except Exception:
                    pass
        last_btn_a = node.button_a_pressed

        # Cube reset (B or Y button)
        reset_now = node.reset_requested
        if reset_now and not last_reset_btn:
            reset_cubes(world, cubes, table_z)
        last_reset_btn = reset_now

        # Wait for calibration
        if not node.both_calibrated:
            if node.left_arm.pose_count > 0 or node.right_arm.pose_count > 0:
                total = node.left_arm.pose_count + node.right_arm.pose_count
                if total % 30 == 1:
                    l = ("DONE" if node.left_arm.calibrated
                         else f"{len(node.left_arm.calibration_poses)}/30")
                    r = ("DONE" if node.right_arm.calibrated
                         else f"{len(node.right_arm.calibration_poses)}/30")
                    print(f"[Calibration] Left: {l}  Right: {r}")
            else:
                if not hasattr(node, '_last_wait'):
                    node._last_wait = 0
                if time.time() - node._last_wait > 2.0:
                    print("[Waiting] No Quest data yet — open webxr_streamer.html and connect")
                    node._last_wait = time.time()
            world.step(render=True)
            continue

        cur = openarm.get_joint_positions()
        if cur is None:
            world.step(render=True)
            continue

        target = cur.copy()
        alpha  = CONFIG["smoothing"]

        # Smoothing (deadman-gated)
        for arm in (node.left_arm, node.right_arm):
            if arm.deadman_active and alpha > 0:
                arm.smoothed_pos = alpha * arm.smoothed_pos + (1 - alpha) * arm.target_pos
                arm.smoothed_rot = alpha * arm.smoothed_rot + (1 - alpha) * arm.target_rot
                arm.smoothed_rot /= np.linalg.norm(arm.smoothed_rot)
            elif not arm.deadman_active and alpha > 0:
                pass  # frozen — smoothed already holds last position

        # IK — Left arm
        if ik_enabled:
            ws = last_left_pos if last_left_pos is not None else LEFT_ARM_PREFERRED_CONFIG
            acts, ok = left_ik.compute_inverse_kinematics(
                target_position=node.left_arm.smoothed_pos,
                target_orientation=node.left_arm.smoothed_rot,
                frame_name="openarm_left_hand", warm_start=ws)
            if ok:
                left_ik_success += 1
                pos7 = np.array(acts).flatten()[:7]
                last_left_pos = pos7.copy()
                for i, idx in enumerate(left_arm_idx):
                    if i < len(pos7):
                        target[idx] = pos7[i]
            else:
                left_ik_fail += 1
                if last_left_pos is not None:
                    for i, idx in enumerate(left_arm_idx):
                        target[idx] = last_left_pos[i]

        # IK — Right arm
        if ik_enabled:
            ws = last_right_pos if last_right_pos is not None else RIGHT_ARM_PREFERRED_CONFIG
            acts, ok = right_ik.compute_inverse_kinematics(
                target_position=node.right_arm.smoothed_pos,
                target_orientation=node.right_arm.smoothed_rot,
                frame_name="openarm_right_hand", warm_start=ws)
            if ok:
                right_ik_success += 1
                pos7 = np.array(acts).flatten()[:7]
                last_right_pos = pos7.copy()
                for i, idx in enumerate(right_arm_idx):
                    if i < len(pos7):
                        target[idx] = pos7[i]
            else:
                right_ik_fail += 1
                if last_right_pos is not None:
                    for i, idx in enumerate(right_arm_idx):
                        target[idx] = last_right_pos[i]

        # Gripper
        gs  = CONFIG["gripper_speed"]
        gop = CONFIG["gripper_open_pos"]
        gcl = CONFIG["gripper_closed_pos"]
        for arm, g_idx in [(node.left_arm, left_gripper_idx),
                           (node.right_arm, right_gripper_idx)]:
            tgt = gcl if arm.gripper_closed else gop
            if arm.smoothed_gripper_pos < tgt:
                arm.smoothed_gripper_pos = min(arm.smoothed_gripper_pos + gs, tgt)
            else:
                arm.smoothed_gripper_pos = max(arm.smoothed_gripper_pos - gs, tgt)
            for idx in g_idx:
                target[idx] = arm.smoothed_gripper_pos

        openarm.apply_action(ArticulationAction(joint_positions=target))

        # Joint state publish
        js = JointState()
        js.header.stamp = node.get_clock().now().to_msg()
        js.name         = list(dof_names)
        js.position     = target.tolist()
        node.joint_state_pub.publish(js)

        # Stacking check
        frame_counter += 1
        if frame_counter % stack_check_interval == 0:
            stacked = check_stacking(cubes)
            msg_str = ", ".join(stacked) if stacked else ""
            if msg_str != last_stack_msg:
                if stacked:
                    print(f"[Stack] ✅ {msg_str}")
                last_stack_msg = msg_str

        # Camera capture
        if frame_counter % CAPTURE_EVERY == 0:
            ts = node.get_clock().now().to_msg()
            for cname, ann in cam_annotators.items():
                try:
                    data = ann.get_data()
                    if data is not None and len(data) > 0:
                        img = data[:, :, :3].astype(np.uint8) if data.shape[2] == 4 \
                              else data.astype(np.uint8)
                        cam_queue.put_nowait((cname, img.copy(), ts))
                except Exception:
                    pass

        world.step(render=True)

    # ── Cleanup ───────────────────────────────────────────────────────────────
    cam_thread_running = False
    print(f"\nIK stats — L ok:{left_ik_success} fail:{left_ik_fail} | "
          f"R ok:{right_ik_success} fail:{right_ik_fail}")
    node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
