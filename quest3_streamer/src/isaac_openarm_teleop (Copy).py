# isaac_openarm_teleop.py
# Real-time VR Bimanual Teleoperation for OpenArm robot
# Uses both Meta Quest 3 controllers for left/right arm control
#
# v3 changes (vs the previous v2):
#   - Two-stage IK: try full pose (position + orientation) first; if that
#     fails, fall back to position-only IK so the arm NEVER freezes when the
#     wrist orientation is unreachable. This is the single biggest fix for
#     the "laggy / position tracking is worse than the old version" feeling.
#   - Loosened orientation_tolerance (0.5 -> 1.0 rad) so full-pose IK
#     succeeds more often.
#   - Light smoothing (0.0 -> 0.2) to kill VR controller jitter without
#     adding noticeable lag.
#   - Per-frame stats now distinguish full-pose vs position-only successes.

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
from omni.isaac.core.robots import Robot
from pxr import UsdGeom, UsdPhysics, PhysxSchema, Gf
import os
import yaml

# LOAD CONFIG
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
CONFIG_PATH = os.path.join(PROJECT_ROOT, "config", "config.yaml")

with open(CONFIG_PATH, 'r') as f:
    PATH_CONFIG = yaml.safe_load(f)

USD_PATH = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['usd'])
URDF_PATH = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['urdf'])
LEFT_ARM_CONFIG_DIR = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['left_arm_config'])
RIGHT_ARM_CONFIG_DIR = os.path.join(PROJECT_ROOT, PATH_CONFIG['paths']['openarm']['right_arm_config'])


CONFIG = {
    "pos_scale": 1,

    # Workspace center is auto-overwritten from FK at startup.
    "robot_workspace_center": [0.3, 0.0, 0.3],
    "left_arm_offset":  [0.0,  0.15, 0.0],
    "right_arm_offset": [0.0, -0.15, 0.0],

    # Light smoothing kills VR jitter without much added latency.
    # 0.0 = raw VR (jittery, more IK failures on borderline frames)
    # 0.2 = nice balance (recommended)
    # 0.5 = noticeably damped, slightly laggy
    # 0.9 = molasses
    # "smoothing": 0.2,
    "smoothing": 0.1,

    "gripper_threshold": 0.5,
    "gripper_open_pos": 0.132,
    "gripper_closed_pos": -1,
    "gripper_speed": 0.05,
    "calibration_samples": 30,
    "debug_ik": False,

    # ---------------- IK behaviour ----------------
    # Two-stage IK: when True, the solver first tries to match full pose
    # (position + orientation). If that fails (orientation unreachable
    # within tolerance, or VR jitter pushed target past joint limits),
    # we retry with position-only IK so the arm keeps tracking the user's
    # hand position. This is what makes the arm feel responsive again.
    "use_orientation_ik": True,
    "ik_orientation_fallback": True,

    # Tolerances passed per IK call.
    "position_tolerance": 0.01,   # 5 cm of the target

    # 1.0 rad ~= 57 deg. The previous 0.5 rad (~28 deg) was tight enough
    # that small VR jitter near joint limits caused frequent failures.
    "orientation_tolerance": 0.7,

    # ---------------- Scene reset ----------------
    "environment_object_offset": [-0.10, 0.0, 0.0],
    "environment_object_paths": [
        "/packing_table_01",
        "/electric_screw_driver",
        "/box",
    ],
}


class KeyboardState:
    def __init__(self):
        self.reset_objects_requested = False


kb_state = KeyboardState()


def _on_keyboard_event(event, *args, **kwargs):
    raw = event.input
    name = raw if isinstance(raw, str) else raw.name
    if event.type == carb.input.KeyboardEventType.KEY_PRESS and name.upper() == "R":
        kb_state.reset_objects_requested = True
    return True


# OpenArm joint names
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
LEFT_GRIPPER_JOINTS = ["openarm_left_finger_joint1", "openarm_left_finger_joint2"]
RIGHT_GRIPPER_JOINTS = ["openarm_right_finger_joint1", "openarm_right_finger_joint2"]

# Preferred IK seed configs: opposite signs on joint2 push elbows outward.
LEFT_ARM_PREFERRED_CONFIG  = np.array([0.0, -1.0, 0.0, 1.2, 0.0, 0.0, 0.0])
RIGHT_ARM_PREFERRED_CONFIG = np.array([0.0,  1.0, 0.0, 1.2, 0.0, 0.0, 0.0])


def smooth_quaternion(current, target, alpha):
    """Blend wxyz quaternions while avoiding sign-flip jumps."""
    if np.dot(current, target) < 0:
        target = -target
    blended = alpha * current + (1 - alpha) * target
    norm = np.linalg.norm(blended)
    if norm < 1e-8:
        return target
    return blended / norm


def solve_ik_with_fallback(
    solver,
    target_pos,
    target_rot,
    frame_name,
    warm_start,
    position_tolerance,
    orientation_tolerance,
    use_orientation,
    enable_fallback,
):
    """
    Two-stage IK: try full pose first, fall back to position-only if that
    fails. Returns (joint_array, success, mode) where mode is one of
    'full', 'pos_only', 'failed'.

    Why this matters:
      Lula's compute_inverse_kinematics is binary success/fail. When you ask
      for full pose match and any small piece of the target is unreachable
      (joint limit, near singularity, VR controller jitter), the whole call
      fails and the caller has to freeze the arm at the last good config.
      That feels like lag / sluggish tracking.

      Position-only IK on a 7-DOF arm almost never fails — there's enough
      redundancy that any reachable XYZ has many valid joint solutions, and
      warm_start biases the solver toward a wrist orientation similar to
      the previous frame. So the arm always tracks position, and tracks
      orientation whenever physically possible.
    """
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

    # Position-only fallback (or primary if orientation IK is disabled).
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


def get_xform_op_values(prim):
    xform = UsdGeom.Xformable(prim)
    return [(op.GetOpName(), op.Get()) for op in xform.GetOrderedXformOps()]


def restore_xform_op_values(prim, op_values):
    xform = UsdGeom.Xformable(prim)
    ops_by_name = {op.GetOpName(): op for op in xform.GetOrderedXformOps()}
    for op_name, value in op_values:
        op = ops_by_name.get(op_name)
        if op is not None:
            op.Set(value)


def iter_prim_tree(prim):
    yield prim
    for child in prim.GetChildren():
        yield from iter_prim_tree(child)


def zero_rigid_body_velocities(prim):
    zero = Gf.Vec3f(0.0, 0.0, 0.0)
    for child in iter_prim_tree(prim):
        for attr_name in ("physics:velocity", "physics:angularVelocity"):
            attr = child.GetAttribute(attr_name)
            if attr and attr.IsValid():
                attr.Set(zero)


def capture_environment_object_states(stage, object_paths):
    states = {}
    for object_path in object_paths:
        prim = stage.GetPrimAtPath(object_path)
        if prim.IsValid():
            states[object_path] = get_xform_op_values(prim)
    return states


def reset_environment_objects(stage, object_states):
    for object_path, op_values in object_states.items():
        prim = stage.GetPrimAtPath(object_path)
        if not prim.IsValid():
            print(f"[Reset] Warning: {object_path} not found, skipped")
            continue
        restore_xform_op_values(prim, op_values)
        zero_rigid_body_velocities(prim)
    print("[Reset] Workspace objects restored")


class ArmState:
    """Tracks state for a single arm controller."""
    def __init__(self, name, arm_offset, transform_matrix):
        self.name = name
        self.arm_offset = np.array(arm_offset)
        self.transform_matrix = transform_matrix
        self.home_pos = None
        self.target_pos = np.array([0.3, 0.0, 0.3])
        self.target_rot = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz
        self.smoothed_pos = np.array([0.3, 0.0, 0.3])
        self.smoothed_rot = np.array([1.0, 0.0, 0.0, 0.0])
        self.gripper_closed = False
        self.smoothed_gripper_pos = 0.132
        self.calibrated = False
        self.calibration_poses = []
        self.reference_pos = None
        self.pose_count = 0


class BimanualQuestTeleop(Node):
    """ROS2 node for bimanual Quest 3 teleoperation."""

    def __init__(self, config):
        super().__init__('isaac_openarm_teleop')
        self.config = config

        self.T = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
        self.workspace_center = np.array(config["robot_workspace_center"])

        self.left_arm  = ArmState("left",  config["left_arm_offset"],  self.T)
        self.right_arm = ArmState("right", config["right_arm_offset"], self.T)

        self.button_a_pressed = False
        self.button_b_pressed = False
        self.button_x_pressed = False
        self.button_y_pressed = False

        self.left_pose_sub = self.create_subscription(
            PoseStamped, '/quest/left_hand/pose',
            lambda msg: self.pose_callback(msg, self.left_arm), 10)
        self.left_input_sub = self.create_subscription(
            Joy, '/quest/left_hand/inputs',
            lambda msg: self.input_callback(msg, self.left_arm, is_left=True), 10)

        self.right_pose_sub = self.create_subscription(
            PoseStamped, '/quest/right_hand/pose',
            lambda msg: self.pose_callback(msg, self.right_arm), 10)
        self.right_input_sub = self.create_subscription(
            Joy, '/quest/right_hand/inputs',
            lambda msg: self.input_callback(msg, self.right_arm, is_left=False), 10)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.camera_pubs = {
            'head':        self.create_publisher(Image, '/camera/head/image_raw', 10),
            'wrist_left':  self.create_publisher(Image, '/camera/wrist_left/image_raw', 10),
            'wrist_right': self.create_publisher(Image, '/camera/wrist_right/image_raw', 10),
        }

        self.get_logger().info("BimanualQuestTeleop Initialized (v3 - two-stage IK)")
        self.get_logger().info("Waiting for Quest controller data...")

    def pose_callback(self, msg, arm_state):
        arm_state.pose_count += 1
        xr_pos = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

        if not arm_state.calibrated:
            arm_state.calibration_poses.append(xr_pos.copy())
            if len(arm_state.calibration_poses) >= self.config["calibration_samples"]:
                arm_state.reference_pos = np.mean(arm_state.calibration_poses, axis=0)
                arm_state.home_pos = self.workspace_center + arm_state.arm_offset
                arm_state.target_pos = arm_state.home_pos.copy()
                arm_state.smoothed_pos = arm_state.home_pos.copy()
                arm_state.calibrated = True
                self.get_logger().info(f"{arm_state.name.upper()} ARM CALIBRATION COMPLETE")
                self.get_logger().info(f"  Home position set to: {arm_state.home_pos}")
            return

        xr_offset = xr_pos - arm_state.reference_pos
        robot_offset = arm_state.transform_matrix @ xr_offset
        robot_pos = robot_offset * self.config["pos_scale"] + arm_state.home_pos

        xr_quat = np.array([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ])
        r_xr = R.from_quat(xr_quat)
        mat_xr = r_xr.as_matrix()
        mat_robot = arm_state.transform_matrix @ mat_xr @ arm_state.transform_matrix.T
        flip = R.from_euler('x', 180, degrees=True).as_matrix()
        mat_robot = mat_robot @ flip
        quat_robot = R.from_matrix(mat_robot).as_quat()
        robot_rot = np.array([quat_robot[3], quat_robot[0], quat_robot[1], quat_robot[2]])

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


def main():
    print("[Init] Warming up Isaac Sim...")
    for _ in range(30):
        simulation_app.update()

    print(f"[Init] Loading stage from {USD_PATH}...")
    open_stage(USD_PATH)

    from omni.usd import get_context
    stage = get_context().get_stage()
    object_offset = np.array(CONFIG.get("environment_object_offset", [0.0, 0.0, 0.0]), dtype=float)
    if np.linalg.norm(object_offset) > 0:
        for object_path in CONFIG.get("environment_object_paths", []):
            object_prim = stage.GetPrimAtPath(object_path)
            if not object_prim.IsValid():
                print(f"[Init] Warning: {object_path} not found, object offset skipped")
                continue
            object_xform = UsdGeom.Xformable(object_prim)
            translate_ops = [
                op for op in object_xform.GetOrderedXformOps()
                if op.GetOpName() == "xformOp:translate"
            ]
            if translate_ops:
                current_translation = np.array(translate_ops[0].Get(), dtype=float)
                new_translation = current_translation + object_offset
                translate_ops[0].Set(Gf.Vec3d(*new_translation))
                print(f"[Init] Moved {object_path} to {np.round(new_translation, 3)}")
    environment_object_states = capture_environment_object_states(
        stage,
        CONFIG.get("environment_object_paths", []),
    )

    print("[Init] Stabilizing stage...")
    for _ in range(50):
        simulation_app.update()

    print("[Init] Creating World...")
    world = World(stage_units_in_meters=1.0)

    for _ in range(20):
        simulation_app.update()

    print("[Init] Loading OpenArm robot...")
    robot_prim_path = None
    from pxr import Usd
    stage = world.stage
    possible_paths = [
        "/World/Robot",
        "/World/openarm",
        "/openarm",
        "/Robot",
        "/Environment/openarm",
        "/World/openarm_bimanual",
    ]
    for path in possible_paths:
        robot_prim = stage.GetPrimAtPath(path)
        if robot_prim.IsValid():
            robot_prim_path = path
            break

    if robot_prim_path is None:
        print("[ERROR] Could not find OpenArm robot in the USD stage!")
        for p in stage.GetPseudoRoot().GetChildren():
            print(f"  - {p.GetPath()}")
        simulation_app.close()
        return

    print(f"[Init] Found robot at: {robot_prim_path}")

    openarm = world.scene.add(
        Articulation(
            prim_path=robot_prim_path,
            name="openarm"
        )
    )

    print("[Init] Loading IK Solvers...")
    from omni.isaac.motion_generation import LulaKinematicsSolver

    left_robot_desc_path  = os.path.join(LEFT_ARM_CONFIG_DIR,  "robot_descriptor.yaml")
    right_robot_desc_path = os.path.join(RIGHT_ARM_CONFIG_DIR, "robot_descriptor.yaml")

    try:
        left_ik_solver = LulaKinematicsSolver(
            robot_description_path=left_robot_desc_path,
            urdf_path=URDF_PATH
        )
        right_ik_solver = LulaKinematicsSolver(
            robot_description_path=right_robot_desc_path,
            urdf_path=URDF_PATH
        )
        ik_enabled = True
        print("[Init] IK Solvers loaded successfully!")

        try:
            fk_pos_l, _ = left_ik_solver.compute_forward_kinematics("openarm_left_hand",  np.zeros(7))
            fk_pos_r, _ = right_ik_solver.compute_forward_kinematics("openarm_right_hand", np.zeros(7))

            print(f"[FK] Left  end-effector at zero config: {np.round(fk_pos_l, 3)}")
            print(f"[FK] Right end-effector at zero config: {np.round(fk_pos_r, 3)}")
            auto_center = ((np.array(fk_pos_l) + np.array(fk_pos_r)) / 2.0).tolist()
            print(f"[FK] Auto workspace center: {np.round(auto_center, 3)}")
            CONFIG["robot_workspace_center"] = auto_center
            CONFIG["left_arm_offset"]  = (np.array(fk_pos_l) - np.array(auto_center)).tolist()
            CONFIG["right_arm_offset"] = (np.array(fk_pos_r) - np.array(auto_center)).tolist()
            print(f"[FK] Left  offset: {np.round(CONFIG['left_arm_offset'], 3)}")
            print(f"[FK] Right offset: {np.round(CONFIG['right_arm_offset'], 3)}")
        except Exception as fk_e:
            print(f"[FK] Could not compute FK: {fk_e}")
    except Exception as e:
        print(f"[WARNING] Could not load IK Solvers: {e}")
        print("[INFO] Falling back to joint position control mode")
        ik_enabled = False

    import omni.ui
    windows_to_hide = [
        "Stage", "Layer", "Render Settings", "Content", "Content Library",
        "Console", "Property", "Properties", "Semantics", "Visual Scripting"
    ]
    print("[UI] Minimizing panels for Zoom Mode...")
    for name in windows_to_hide:
        try:
            w = omni.ui.Workspace.get_window(name)
            if w:
                w.visible = False
        except:
            pass

    print("[Init] Resetting World...")
    world.reset()

    for _ in range(20):
        simulation_app.update()

    print("[Init] Getting joint information...")
    dof_names = openarm.dof_names
    print(f"[Info] Available DOFs: {dof_names}")

    left_arm_indices  = []
    right_arm_indices = []
    left_gripper_indices  = []
    right_gripper_indices = []

    for i, name in enumerate(dof_names):
        if name in LEFT_ARM_JOINTS:
            left_arm_indices.append(i)
        elif name in RIGHT_ARM_JOINTS:
            right_arm_indices.append(i)
        elif name in LEFT_GRIPPER_JOINTS:
            left_gripper_indices.append(i)
        elif name in RIGHT_GRIPPER_JOINTS:
            right_gripper_indices.append(i)

    print(f"[Info] Left arm indices:  {left_arm_indices}")
    print(f"[Info] Right arm indices: {right_arm_indices}")
    print(f"[Info] Left gripper indices:  {left_gripper_indices}")
    print(f"[Info] Right gripper indices: {right_gripper_indices}")

    print("[Init] Initializing ROS2...")
    rclpy.init()
    teleop_node = BimanualQuestTeleop(CONFIG)

    print("=" * 60)
    print("OpenArm Bimanual Teleop Ready (v3)")
    print(f"Loaded: {USD_PATH}")
    print("=" * 60)
    print("Controls:")
    print("  - Left  Quest Controller -> Left  Arm")
    print("  - Right Quest Controller -> Right Arm")
    print("  - Trigger/Grip           -> Close Gripper")
    print("  - A/X Button             -> Camera Switch")
    print("  - B/Y Button or keyboard R -> Reset workspace objects")
    print("=" * 60)

    input_iface = None
    keyboard = None
    kb_sub = None
    try:
        appwindow = omni.appwindow.get_default_app_window()
        input_iface = carb.input.acquire_input_interface()
        keyboard = appwindow.get_keyboard()
        kb_sub = input_iface.subscribe_to_keyboard_events(keyboard, _on_keyboard_event)
        print("[Init] Keyboard registered: R resets workspace objects")
    except Exception as e:
        print(f"[Init] Keyboard registration skipped: {e}")

    # Per-arm IK stats:
    #   *_full   = full pose (position + orientation) succeeded
    #   *_pos    = orientation IK failed but position-only IK succeeded
    #   *_fail   = even position-only IK failed (truly unreachable target)
    left_full = 0;  left_pos = 0;  left_fail = 0
    right_full = 0; right_pos = 0; right_fail = 0
    last_left_arm_positions = None
    last_right_arm_positions = None

    import omni.kit.viewport.utility
    cameras = ["/OmniverseKit_Persp"]
    camera_names = ["Perspective"]

    for prim in stage.Traverse():
        path_str = str(prim.GetPath())
        if (
            prim.IsA(UsdGeom.Camera)
            and path_str != "/OmniverseKit_Persp"
            and path_str != '/OmniverseKit_Front'
            and path_str != '/OmniverseKit_Right'
        ):
            cameras.append(path_str)
            cam_name = path_str.split("/")[-1].replace("_", " ").title()
            camera_names.append(cam_name)

    print(f"[Camera] Found {len(cameras)} cameras: {camera_names}")
    current_cam_index = 0
    last_button_a = False
    last_reset_button = False

    import omni.replicator.core as rep
    import threading
    import queue

    RECORDING_CAMERAS = {
        'head':        '/openarm/openarm_body_link/head_camera',
        'wrist_left':  '/openarm/openarm_left_link7/left_wrist_camera',
        'wrist_right': '/openarm/openarm_right_link7/right_wrist_camera',
    }

    CAMERA_RESOLUTION = (480, 360)

    camera_render_products = {}
    camera_annotators = {}
    for cam_name, cam_path in RECORDING_CAMERAS.items():
        cam_prim = stage.GetPrimAtPath(cam_path)
        if cam_prim.IsValid():
            rp = rep.create.render_product(cam_path, CAMERA_RESOLUTION)
            camera_render_products[cam_name] = rp
            rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb")
            rgb_annot.attach([rp])
            camera_annotators[cam_name] = rgb_annot
            print(f"[Camera] Setup recording for: {cam_name} at {CAMERA_RESOLUTION}")
        else:
            print(f"[Camera] Warning: Camera not found: {cam_path}")

    camera_queue = queue.Queue(maxsize=3)
    camera_thread_running = True

    def camera_publish_thread():
        while camera_thread_running:
            try:
                cam_name, img_rgb, timestamp = camera_queue.get(timeout=0.1)
                img_msg = Image()
                img_msg.header.stamp = timestamp
                img_msg.header.frame_id = cam_name
                img_msg.height = img_rgb.shape[0]
                img_msg.width = img_rgb.shape[1]
                img_msg.encoding = 'rgb8'
                img_msg.is_bigendian = False
                img_msg.step = img_rgb.shape[1] * 3
                img_msg.data = img_rgb.tobytes()
                teleop_node.camera_pubs[cam_name].publish(img_msg)
                camera_queue.task_done()
            except queue.Empty:
                continue
            except Exception:
                pass

    cam_thread = threading.Thread(target=camera_publish_thread, daemon=True)
    cam_thread.start()
    print("[Camera] Async camera publisher started")

    frame_counter = 0
    CAMERA_CAPTURE_INTERVAL = 3  # Slightly less aggressive than before to free up cycles

    use_orientation_ik   = CONFIG.get("use_orientation_ik", True)
    enable_fallback      = CONFIG.get("ik_orientation_fallback", True)
    position_tolerance   = CONFIG.get("position_tolerance", 0.05)
    orientation_tolerance = CONFIG.get("orientation_tolerance", 1.0)

    while simulation_app.is_running():
        rclpy.spin_once(teleop_node, timeout_sec=0.0)

        if teleop_node.button_a_pressed and not last_button_a:
            current_cam_index = (current_cam_index + 1) % len(cameras)
            new_cam = cameras[current_cam_index]
            cam_name = camera_names[current_cam_index]
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                try:
                    viewport.camera_path = new_cam
                    print(f"[Camera] Switched to: {cam_name} ({new_cam})")
                except:
                    print(f"[Camera] Could not switch to {cam_name}")
        last_button_a = teleop_node.button_a_pressed

        if kb_state.reset_objects_requested:
            kb_state.reset_objects_requested = False
            reset_environment_objects(stage, environment_object_states)

        reset_button_now = teleop_node.reset_requested
        if reset_button_now and not last_reset_button:
            reset_environment_objects(stage, environment_object_states)
        last_reset_button = reset_button_now

        if not teleop_node.both_calibrated:
            if teleop_node.left_arm.pose_count > 0 or teleop_node.right_arm.pose_count > 0:
                left_status = "CALIBRATED" if teleop_node.left_arm.calibrated else f"Calibrating ({len(teleop_node.left_arm.calibration_poses)}/{CONFIG['calibration_samples']})"
                right_status = "CALIBRATED" if teleop_node.right_arm.calibrated else f"Calibrating ({len(teleop_node.right_arm.calibration_poses)}/{CONFIG['calibration_samples']})"
                if (teleop_node.left_arm.pose_count + teleop_node.right_arm.pose_count) % 30 == 1:
                    print(f"[Calibration] Left: {left_status} | Right: {right_status}")
            else:
                import time
                if not hasattr(teleop_node, '_last_waiting_print'):
                    teleop_node._last_waiting_print = 0
                current_time = time.time()
                if current_time - teleop_node._last_waiting_print > 2.0:
                    print("[Waiting] No Quest controller data received yet. Is the Quest ROS2 bridge running?")
                    print("          Check: ros2 topic list | grep quest")
                    print("          Expected topics: /quest/left_hand/pose, /quest/right_hand/pose")
                    teleop_node._last_waiting_print = current_time
            world.step(render=True)
            continue

        current_positions = openarm.get_joint_positions()
        if current_positions is None:
            world.step(render=True)
            continue

        target_positions = current_positions.copy()

        # ---------------- Smoothing ----------------
        alpha = CONFIG["smoothing"]
        if alpha > 0:
            teleop_node.left_arm.smoothed_pos = (
                alpha * teleop_node.left_arm.smoothed_pos +
                (1 - alpha) * teleop_node.left_arm.target_pos
            )
            teleop_node.right_arm.smoothed_pos = (
                alpha * teleop_node.right_arm.smoothed_pos +
                (1 - alpha) * teleop_node.right_arm.target_pos
            )
            teleop_node.left_arm.smoothed_rot = smooth_quaternion(
                teleop_node.left_arm.smoothed_rot,
                teleop_node.left_arm.target_rot,
                alpha,
            )
            teleop_node.right_arm.smoothed_rot = smooth_quaternion(
                teleop_node.right_arm.smoothed_rot,
                teleop_node.right_arm.target_rot,
                alpha,
            )
        else:
            teleop_node.left_arm.smoothed_pos = teleop_node.left_arm.target_pos
            teleop_node.left_arm.smoothed_rot = teleop_node.left_arm.target_rot
            teleop_node.right_arm.smoothed_pos = teleop_node.right_arm.target_pos
            teleop_node.right_arm.smoothed_rot = teleop_node.right_arm.target_rot

        frame_count = (left_full + left_pos + left_fail
                       + right_full + right_pos + right_fail)

        if CONFIG.get("debug_ik") and frame_count % 100 == 0:
            print(f"[IK Debug] Left  target: pos={teleop_node.left_arm.smoothed_pos}, rot={teleop_node.left_arm.smoothed_rot}")
            print(f"[IK Debug] Right target: pos={teleop_node.right_arm.smoothed_pos}, rot={teleop_node.right_arm.smoothed_rot}")

        # ---------------- LEFT ARM IK ----------------
        if ik_enabled:
            warm_start_l = (last_left_arm_positions
                            if last_left_arm_positions is not None
                            else LEFT_ARM_PREFERRED_CONFIG)
            left_actions, left_success, left_mode = solve_ik_with_fallback(
                solver=left_ik_solver,
                target_pos=teleop_node.left_arm.smoothed_pos,
                target_rot=teleop_node.left_arm.smoothed_rot,
                frame_name="openarm_left_hand",
                warm_start=warm_start_l,
                position_tolerance=position_tolerance,
                orientation_tolerance=orientation_tolerance,
                use_orientation=use_orientation_ik,
                enable_fallback=enable_fallback,
            )

            if left_success:
                if left_mode == "full":
                    left_full += 1
                else:
                    left_pos += 1
                left_arm_positions = np.array(left_actions).flatten()[:7]
                last_left_arm_positions = left_arm_positions.copy()
                for i, idx in enumerate(left_arm_indices):
                    if i < len(left_arm_positions):
                        target_positions[idx] = left_arm_positions[i]
            else:
                left_fail += 1
                if left_fail <= 3:
                    print(f"[IK] Left arm FAIL (even pos-only) for target: {teleop_node.left_arm.smoothed_pos}")
                if last_left_arm_positions is not None:
                    for i, idx in enumerate(left_arm_indices):
                        if i < len(last_left_arm_positions):
                            target_positions[idx] = last_left_arm_positions[i]

        # ---------------- RIGHT ARM IK ----------------
        if ik_enabled:
            warm_start_r = (last_right_arm_positions
                            if last_right_arm_positions is not None
                            else RIGHT_ARM_PREFERRED_CONFIG)
            right_actions, right_success, right_mode = solve_ik_with_fallback(
                solver=right_ik_solver,
                target_pos=teleop_node.right_arm.smoothed_pos,
                target_rot=teleop_node.right_arm.smoothed_rot,
                frame_name="openarm_right_hand",
                warm_start=warm_start_r,
                position_tolerance=position_tolerance,
                orientation_tolerance=orientation_tolerance,
                use_orientation=use_orientation_ik,
                enable_fallback=enable_fallback,
            )

            if right_success:
                if right_mode == "full":
                    right_full += 1
                else:
                    right_pos += 1
                right_arm_positions = np.array(right_actions).flatten()[:7]
                last_right_arm_positions = right_arm_positions.copy()
                for i, idx in enumerate(right_arm_indices):
                    if i < len(right_arm_positions):
                        target_positions[idx] = right_arm_positions[i]
            else:
                right_fail += 1
                if right_fail <= 3:
                    print(f"[IK] Right arm FAIL (even pos-only) for target: {teleop_node.right_arm.smoothed_pos}")
                if last_right_arm_positions is not None:
                    for i, idx in enumerate(right_arm_indices):
                        if i < len(last_right_arm_positions):
                            target_positions[idx] = last_right_arm_positions[i]

        # Periodic IK quality summary every ~600 frames per arm
        if frame_count > 0 and frame_count % 1200 == 0:
            l_total = max(1, left_full + left_pos + left_fail)
            r_total = max(1, right_full + right_pos + right_fail)
            print(
                f"[IK Stats] Left  full={100*left_full/l_total:.0f}% "
                f"pos-only={100*left_pos/l_total:.0f}% "
                f"fail={100*left_fail/l_total:.0f}%"
            )
            print(
                f"[IK Stats] Right full={100*right_full/r_total:.0f}% "
                f"pos-only={100*right_pos/r_total:.0f}% "
                f"fail={100*right_fail/r_total:.0f}%"
            )

        # ---------------- Gripper ----------------
        gripper_speed = CONFIG["gripper_speed"]
        gripper_open  = CONFIG["gripper_open_pos"]
        gripper_closed = CONFIG["gripper_closed_pos"]

        left_target = gripper_closed if teleop_node.left_arm.gripper_closed else gripper_open
        if teleop_node.left_arm.smoothed_gripper_pos < left_target:
            teleop_node.left_arm.smoothed_gripper_pos = min(
                teleop_node.left_arm.smoothed_gripper_pos + gripper_speed, left_target)
        elif teleop_node.left_arm.smoothed_gripper_pos > left_target:
            teleop_node.left_arm.smoothed_gripper_pos = max(
                teleop_node.left_arm.smoothed_gripper_pos - gripper_speed, left_target)
        for idx in left_gripper_indices:
            target_positions[idx] = teleop_node.left_arm.smoothed_gripper_pos

        right_target = gripper_closed if teleop_node.right_arm.gripper_closed else gripper_open
        if teleop_node.right_arm.smoothed_gripper_pos < right_target:
            teleop_node.right_arm.smoothed_gripper_pos = min(
                teleop_node.right_arm.smoothed_gripper_pos + gripper_speed, right_target)
        elif teleop_node.right_arm.smoothed_gripper_pos > right_target:
            teleop_node.right_arm.smoothed_gripper_pos = max(
                teleop_node.right_arm.smoothed_gripper_pos - gripper_speed, right_target)
        for idx in right_gripper_indices:
            target_positions[idx] = teleop_node.right_arm.smoothed_gripper_pos

        openarm.apply_action(ArticulationAction(joint_positions=target_positions))

        # Joint state publish for LeRobot recording
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = teleop_node.get_clock().now().to_msg()
        joint_state_msg.name = dof_names
        joint_state_msg.position = target_positions.tolist()
        teleop_node.joint_state_pub.publish(joint_state_msg)

        # Camera capture (async-published)
        frame_counter += 1
        if frame_counter % CAMERA_CAPTURE_INTERVAL == 0:
            timestamp = teleop_node.get_clock().now().to_msg()
            for cam_name, annotator in camera_annotators.items():
                try:
                    data = annotator.get_data()
                    if data is not None and len(data) > 0:
                        if len(data.shape) == 3 and data.shape[2] == 4:
                            img_rgb = data[:, :, :3].astype(np.uint8)
                        else:
                            img_rgb = data.astype(np.uint8)
                        try:
                            camera_queue.put_nowait((cam_name, img_rgb.copy(), timestamp))
                        except queue.Full:
                            pass
                except Exception:
                    pass

        world.step(render=True)

    print("\n" + "=" * 60)
    print("Session Statistics:")
    l_total = max(1, left_full + left_pos + left_fail)
    r_total = max(1, right_full + right_pos + right_fail)
    print(f"  Left  Arm  full={left_full}  pos-only={left_pos}  fail={left_fail}  "
          f"({100*left_full/l_total:.1f}% / {100*left_pos/l_total:.1f}% / {100*left_fail/l_total:.1f}%)")
    print(f"  Right Arm  full={right_full} pos-only={right_pos} fail={right_fail}  "
          f"({100*right_full/r_total:.1f}% / {100*right_pos/r_total:.1f}% / {100*right_fail/r_total:.1f}%)")
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
