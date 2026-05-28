#!/usr/bin/env python3
"""
VR teleoperation for OpenArm bimanual robot in an IsaacSim cube-stacking scene.

Subscribes to /joint_trajectory published by:
  conda activate teleop_xr && ./scripts/run_openarm_vr_ros2_feedback.sh

Publishes:
  /joint_states                 -> TeleopXR robot visualizer feedback
  /camera/head/image_raw        -> optional head camera feed
  /camera/wrist_left/image_raw  -> optional left wrist camera feed
  /camera/wrist_right/image_raw -> optional right wrist camera feed

Run this script via:
  conda activate env_isaacsim (do NOT source /opt/ros)
  ./scripts/run_openarm_vr_cube_stack_isaacsim.sh
"""

import argparse
import sys
import os
import socket
import threading

# ── SimulationApp must be first ───────────────────────────────────────────────
from isaacsim import SimulationApp

_parser = argparse.ArgumentParser(add_help=False)
_parser.add_argument("--usd",      default=None,               help="Path to openarm_bimanual.usd")
_parser.add_argument("--topic",    default="/joint_trajectory", help="JointTrajectory input topic")
_parser.add_argument("--headless", action="store_true",         help="Run without GUI")
_parser.add_argument("--no-cameras", action="store_true",       help="Disable IsaacSim camera ROS publishing")
_parser.add_argument("--no-fallback-cameras", action="store_true", help="Do not create fallback robot-mounted cameras")
_parser.add_argument("--camera-width", type=int, default=480,   help="Camera stream width")
_parser.add_argument("--camera-height", type=int, default=360,  help="Camera stream height")
_parser.add_argument("--camera-interval", type=int, default=3,  help="Publish cameras every N simulation steps")
_parser.add_argument("--cube-size", type=float, default=0.10,   help="Cube side length in meters")
_parser.add_argument("--cube-spacing", type=float, default=0.10, help="Spacing between cube centers in meters")
_known, _ = _parser.parse_known_args()

app = SimulationApp({
    "headless": _known.headless,
    "width": 1920,
    "height": 1080,
})

# Warm up so kit extensions finish loading before importing anything else.
# We use direct rclpy in this script. The launch wrapper injects IsaacSim's
# Python 3.11 ROS2 bridge packages and sets the same FastRTPS RMW as TeleopXR.
for _ in range(30):
    app.update()

# ── All other imports AFTER warmup ───────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, JointState
    from trajectory_msgs.msg import JointTrajectory
except ImportError:
    print("ERROR: rclpy not available. Run via run_openarm_vr_isaacsim.sh")
    app.close()
    sys.exit(1)

import numpy as np
import omni.usd
from omni.isaac.core.utils.stage import open_stage
from pxr import Gf, Usd, UsdGeom, UsdPhysics
import isaaclab.sim as sim_utils

# ── Config ────────────────────────────────────────────────────────────────────
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT   = os.path.dirname(_SCRIPT_DIR)

_QUEST_STREAMER_USD = os.path.join(
    os.path.dirname(_REPO_ROOT),
    "quest3_streamer", "openarm_config",
    "openarm_bimanual", "openarm_bimanual.usd",
)

# IsaacLab USD kept as reference; the quest3_streamer USD includes the robot
# camera prims used by the feedback/camera workflow.
# _ISAAC_LAB_USD = os.path.join(
#     os.path.dirname(_REPO_ROOT),
#     "openarm_isaac_lab", "source", "openarm", "openarm",
#     "tasks", "manager_based", "openarm_manipulation",
#     "usds", "openarm_bimanual", "openarm_bimanual.usd",
# )

_DEFAULT_USD = _known.usd or _QUEST_STREAMER_USD

if not os.path.exists(_DEFAULT_USD):
    _DEFAULT_USD = os.path.join(
        os.path.dirname(_REPO_ROOT),
        "openarm_isaac_lab", "source", "openarm", "openarm",
        "tasks", "manager_based", "openarm_manipulation",
        "usds", "openarm_bimanual", "openarm_bimanual.usd",
    )

_ENVIRONMENT_OFFSET = np.array([-0.25, 0.0, 0.0], dtype=float)
_CUBE_COLORS = {
    "cubeA": (0.8, 0.1, 0.1),  # red
    "cubeB": (0.1, 0.7, 0.1),  # green
    "cubeC": (0.1, 0.3, 0.9),  # blue
}

# ── Joint command buffer ──────────────────────────────────────────────────────
_cmd_lock    = threading.Lock()
_pending_cmd: dict[str, float] | None = None
_received_cmd_count = 0


def _get_host_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("1.1.1.1", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "localhost"


class _Subscriber(Node):
    def __init__(self, topic: str):
        super().__init__("openarm_isaacsim_teleop")
        self.create_subscription(JointTrajectory, topic, self._cb, 10)
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.image_pubs = {
            "head": self.create_publisher(Image, "/camera/head/image_raw", 10),
            "wrist_left": self.create_publisher(Image, "/camera/wrist_left/image_raw", 10),
            "wrist_right": self.create_publisher(Image, "/camera/wrist_right/image_raw", 10),
        }
        self.get_logger().info(f"Subscribed to {topic}")

    def _cb(self, msg: JointTrajectory):
        if not msg.points:
            return
        positions = msg.points[0].positions
        if len(positions) != len(msg.joint_names):
            return
        global _pending_cmd, _received_cmd_count
        with _cmd_lock:
            _pending_cmd = dict(zip(msg.joint_names, positions))
            _received_cmd_count += 1
            if _received_cmd_count == 1 or _received_cmd_count % 100 == 0:
                sample = {
                    name: round(float(np.degrees(value)), 1)
                    for name, value in list(_pending_cmd.items())[:4]
                }
                print(f"[ROS2] received cmd #{_received_cmd_count}: {sample}...")

    def publish_joint_state(self, positions_by_name: dict[str, float]) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions_by_name.keys())
        msg.position = [float(v) for v in positions_by_name.values()]
        self.joint_state_pub.publish(msg)

    def publish_rgb(self, camera_key: str, frame: np.ndarray) -> None:
        pub = self.image_pubs.get(camera_key)
        if pub is None:
            return
        if frame.ndim != 3 or frame.shape[2] < 3:
            return
        rgb = np.ascontiguousarray(frame[:, :, :3], dtype=np.uint8)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = camera_key
        msg.height = int(rgb.shape[0])
        msg.width = int(rgb.shape[1])
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = int(rgb.shape[1] * 3)
        msg.data = rgb.tobytes()
        pub.publish(msg)


def _pop_pending_cmd() -> dict[str, float] | None:
    global _pending_cmd
    with _cmd_lock:
        cmd = _pending_cmd
        _pending_cmd = None
    return cmd


def _find_articulation_root(stage: Usd.Stage) -> str | None:
    """Return the prim path of the articulation root."""
    # Check well-known prim paths first (valid prim is sufficient — trust USD naming)
    for path in ["/openarm", "/World/openarm", "/World/openarm_bimanual", "/Robot", "/World/Robot"]:
        p = stage.GetPrimAtPath(path)
        if p.IsValid():
            return path
    # Fallback: traverse and look for ArticulationRootAPI via Python type (not string)
    for prim in stage.Traverse():
        if UsdPhysics.ArticulationRootAPI(prim) and "joint" not in prim.GetName().lower():
            return str(prim.GetPath())
    return None


def _build_joint_map(stage: Usd.Stage, robot_path: str) -> dict[str, str]:
    """
    Build {joint_name -> absolute_prim_path} for all drive-capable joints.
    Searches robot_path/joints/* and robot_path/* recursively.
    """
    joint_map: dict[str, str] = {}

    def _scan(prim: Usd.Prim):
        if UsdPhysics.DriveAPI.Get(prim, "angular") or UsdPhysics.DriveAPI.Get(prim, "linear"):
            joint_map[prim.GetName()] = str(prim.GetPath())
            return
        # Also accept joints that don't yet have DriveAPI applied (we'll apply later)
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            joint_map[prim.GetName()] = str(prim.GetPath())
            return
        for child in prim.GetChildren():
            _scan(child)

    robot_prim = stage.GetPrimAtPath(robot_path)
    if robot_prim.IsValid():
        _scan(robot_prim)

    return joint_map


def _apply_offset_to_prim(stage: Usd.Stage, path: str, offset: np.ndarray) -> None:
    if np.linalg.norm(offset) < 1e-8:
        return
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"[Scene] Offset skipped; {path} not found")
        return
    xform = UsdGeom.Xformable(prim)
    for op in xform.GetOrderedXformOps():
        if op.GetOpName() == "xformOp:translate":
            current = np.array(op.Get(), dtype=float)
            moved = current + offset
            op.Set(Gf.Vec3d(float(moved[0]), float(moved[1]), float(moved[2])))
            print(f"[Scene] Moved {path} to {np.round(moved, 3)}")
            return
    print(f"[Scene] Offset skipped; no translate op on {path}")


def _delete_prim(stage: Usd.Stage, path: str) -> None:
    prim = stage.GetPrimAtPath(path)
    if prim.IsValid():
        stage.RemovePrim(path)
        print(f"[Scene] Deleted {path}")


def _table_surface_z(stage: Usd.Stage, fallback: float = 0.99) -> float:
    try:
        cache = UsdGeom.BBoxCache(0, ["default", "render"])
        for path in ("/box", "/packing_table_01"):
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                continue
            bbox = cache.ComputeWorldBound(prim)
            rng = bbox.GetRange()
            z_min = float(rng.GetMin()[2])
            z_max = float(rng.GetMax()[2])
            if path == "/box" and 0.3 < z_min < 2.5:
                print(f"[Scene] Table surface from /box bottom: {z_min:.3f}")
                return z_min
            if path == "/packing_table_01" and 0.3 < z_max < 2.5:
                print(f"[Scene] Table surface from /packing_table_01 top: {z_max:.3f}")
                return z_max
    except Exception as exc:
        print(f"[Scene] Table Z detection failed ({exc})")
    print(f"[Scene] Using table surface fallback: {fallback:.3f}")
    return fallback


def _table_center_xy(stage: Usd.Stage) -> np.ndarray:
    try:
        cache = UsdGeom.BBoxCache(0, ["default", "render"])
        prim = stage.GetPrimAtPath("/packing_table_01")
        if prim.IsValid():
            bbox = cache.ComputeWorldBound(prim)
            rng = bbox.GetRange()
            center = np.array(
                [
                    (float(rng.GetMin()[0]) + float(rng.GetMax()[0])) / 2.0,
                    (float(rng.GetMin()[1]) + float(rng.GetMax()[1])) / 2.0,
                ],
                dtype=float,
            )
            print(f"[Scene] Table center XY: {np.round(center, 3)}")
            return center
    except Exception as exc:
        print(f"[Scene] Table center detection failed ({exc})")
    return np.zeros(2, dtype=float)


def _spawn_cube_stack_task(stage: Usd.Stage) -> dict[str, str]:
    _apply_offset_to_prim(stage, "/packing_table_01", _ENVIRONMENT_OFFSET)

    table_z = _table_surface_z(stage)
    table_xy = _table_center_xy(stage)
    _delete_prim(stage, "/electric_screw_driver")
    _delete_prim(stage, "/box")

    spacing = float(_known.cube_spacing)
    size = float(_known.cube_size)
    offsets = {
        "cubeA": np.array([spacing, 0.0, 0.0], dtype=float),
        "cubeB": np.array([0.0, 0.0, 0.0], dtype=float),
        "cubeC": np.array([-spacing, 0.0, 0.0], dtype=float),
    }

    root_path = "/World/Cubes"
    UsdGeom.Xform.Define(stage, root_path)
    cubes: dict[str, str] = {}
    for name, offset in offsets.items():
        pos = np.array(
            [table_xy[0] + offset[0], table_xy[1] + offset[1], table_z + size / 2.0 + 0.002],
            dtype=float,
        )
        cube_root_path = f"{root_path}/{name}"
        cube_mesh_path = f"{cube_root_path}/Cube"
        cube_root = UsdGeom.Xform.Define(stage, cube_root_path)
        cube_root.AddTranslateOp().Set(Gf.Vec3d(float(pos[0]), float(pos[1]), float(pos[2])))

        cube = UsdGeom.Cube.Define(stage, cube_mesh_path)
        cube.GetSizeAttr().Set(size)
        cube.GetDisplayColorAttr().Set([_CUBE_COLORS[name]])
        UsdPhysics.RigidBodyAPI.Apply(cube_root.GetPrim())
        UsdPhysics.MassAPI.Apply(cube_root.GetPrim()).GetMassAttr().Set(0.2)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        cubes[name] = cube_root_path
        print(f"[Cubes] Spawned {name} at {np.round(pos, 3)}")
    return cubes


def _find_camera_prims(stage: Usd.Stage) -> dict[str, str]:
    """Find likely OpenArm camera prims and map them to TeleopXR stream keys."""
    cameras: list[str] = []
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Camera):
            path = str(prim.GetPath())
            if path.startswith("/OmniverseKit_"):
                continue
            cameras.append(path)

    found: dict[str, str] = {}
    for path in cameras:
        lowered = path.lower()
        name = path.rsplit("/", 1)[-1].lower()
        if "head" in lowered and "head" not in found:
            found["head"] = path
        elif (
            "wrist_left" in lowered
            or "left_wrist" in lowered
            or ("left" in lowered and "camera" in name)
        ) and "wrist_left" not in found:
            found["wrist_left"] = path
        elif (
            "wrist_right" in lowered
            or "right_wrist" in lowered
            or ("right" in lowered and "camera" in name)
        ) and "wrist_right" not in found:
            found["wrist_right"] = path

    if cameras:
        print("[Camera] Camera prims on stage:")
        for path in cameras:
            print(f"  {path}")
    return found


def _find_prim_by_name(stage: Usd.Stage, name: str) -> Usd.Prim | None:
    for prim in stage.Traverse():
        if prim.GetName() == name:
            return prim
    return None


def _define_relative_camera(
    stage: Usd.Stage,
    parent_prim: Usd.Prim,
    name: str,
    translation: tuple[float, float, float],
    rotation_xyz_deg: tuple[float, float, float],
) -> str:
    camera_path = f"{parent_prim.GetPath()}/{name}"
    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.CreateFocalLengthAttr(18.0)
    camera.CreateHorizontalApertureAttr(20.955)
    camera.CreateClippingRangeAttr(Gf.Vec2f(0.01, 20.0))

    xform = UsdGeom.Xformable(camera.GetPrim())
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*translation))
    xform.AddRotateXYZOp().Set(Gf.Vec3f(*rotation_xyz_deg))
    print(f"[Camera] Created fallback camera: {camera_path}")
    return camera_path


def _add_fallback_camera_prims(stage: Usd.Stage) -> dict[str, str]:
    """Create robot-mounted cameras when the USD has none."""
    if _known.no_fallback_cameras:
        return {}

    fallback: dict[str, str] = {}
    specs = {
        # Camera local frames in USD look down -Z. These rotations are conservative
        # guesses; they prioritize having non-black feeds over perfect viewpoint.
        "head": ("openarm_body_link", "teleop_head_camera", (0.18, 0.0, 0.22), (75.0, 0.0, -90.0)),
        "wrist_left": ("openarm_left_link7", "teleop_left_wrist_camera", (0.05, 0.0, 0.04), (90.0, 0.0, -90.0)),
        "wrist_right": ("openarm_right_link7", "teleop_right_wrist_camera", (0.05, 0.0, 0.04), (90.0, 0.0, -90.0)),
    }

    for key, (parent_name, camera_name, translation, rotation) in specs.items():
        parent = _find_prim_by_name(stage, parent_name)
        if parent is None or not parent.IsValid():
            print(f"[Camera] Fallback {key} skipped; parent prim '{parent_name}' not found")
            continue
        fallback[key] = _define_relative_camera(
            stage,
            parent,
            camera_name,
            translation,
            rotation,
        )

    return fallback


def _camera_topic_for_key(key: str) -> str:
    if key == "wrist_left":
        return "/camera/wrist_left/image_raw"
    if key == "wrist_right":
        return "/camera/wrist_right/image_raw"
    return "/camera/head/image_raw"


def _setup_camera_renderers(stage: Usd.Stage) -> dict[str, object]:
    if _known.no_cameras:
        print("[Camera] Disabled by --no-cameras")
        return {}

    camera_prims = _find_camera_prims(stage)
    if not camera_prims:
        print("[Camera] No robot camera prims found in USD; creating fallback cameras.")
        camera_prims = _add_fallback_camera_prims(stage)
        if not camera_prims:
            print("[Camera] Fallback camera creation failed; camera topics will not publish.")
            return {}

    try:
        import omni.replicator.core as rep
    except Exception as exc:
        print(f"[Camera] Replicator unavailable ({exc}); camera topics will not publish.")
        return {}

    annotators: dict[str, object] = {}
    resolution = (_known.camera_width, _known.camera_height)
    for key, prim_path in camera_prims.items():
        try:
            render_product = rep.create.render_product(prim_path, resolution)
            annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            annotator.attach([render_product])
            annotators[key] = annotator
            print(f"[Camera] Publishing {key}: {prim_path} -> {_camera_topic_for_key(key)}")
        except Exception as exc:
            print(f"[Camera] Failed to set up {key} at {prim_path}: {exc}")
    return annotators


def _drive_joint(stage: Usd.Stage, prim_path: str, angle_rad: float, is_finger: bool = False):
    """Set a USD drive target.

    Revolute joints use angular drives in degrees. Prismatic joints use linear
    drives in native USD units, so OpenArm finger targets are passed through.
    """
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return
    use_linear = prim.IsA(UsdPhysics.PrismaticJoint) or bool(
        UsdPhysics.DriveAPI.Get(prim, "linear")
    )
    drive_name = "linear" if use_linear else "angular"
    drive = UsdPhysics.DriveAPI.Get(prim, drive_name)
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(prim, drive_name)
    if is_finger:
        drive.GetStiffnessAttr().Set(5000.0)
        drive.GetDampingAttr().Set(300.0)
    else:
        drive.GetStiffnessAttr().Set(1000.0)
        drive.GetDampingAttr().Set(100.0)
    drive.GetMaxForceAttr().Set(10000.0)
    target = float(angle_rad) if use_linear else float(np.degrees(angle_rad))
    drive.GetTargetPositionAttr().Set(target)


def _drive_gripper_side(
    stage: Usd.Stage,
    joint_map: dict[str, str],
    side: str,
    angle_rad: float,
) -> None:
    for name in (
        f"openarm_{side}_finger_joint1",
        f"openarm_{side}_finger_joint2",
    ):
        prim_path = joint_map.get(name)
        if prim_path:
            _drive_joint(stage, prim_path, angle_rad, is_finger=True)


def main():
    # ── Load USD via open_stage (preserves original prim paths) ──────────────
    if not os.path.exists(_DEFAULT_USD):
        print(f"[ERROR] USD not found: {_DEFAULT_USD}")
        print("Pass --usd /path/to/openarm_bimanual.usd")
        app.close()
        sys.exit(1)

    print(f"[Init] Loading USD: {_DEFAULT_USD}")
    open_stage(_DEFAULT_USD)

    print("[Init] Stabilizing stage...")
    for _ in range(50):
        app.update()

    stage = omni.usd.get_context().get_stage()
    cubes = _spawn_cube_stack_task(stage)

    # ── SimulationContext wraps the already-open stage ────────────────────────
    # Created AFTER open_stage so it picks up the existing stage & physics scene
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01, device="cpu"))

    # ── Find robot articulation root ──────────────────────────────────────────
    robot_path = _find_articulation_root(stage)
    if robot_path is None:
        print("[ERROR] Robot articulation root not found.")
        for p in stage.GetPseudoRoot().GetChildren():
            print(f"  {p.GetPath()}")
        app.close()
        sys.exit(1)
    print(f"[Init] Robot prim: {robot_path}")

    # ── Build joint lookup table ──────────────────────────────────────────────
    joint_map = _build_joint_map(stage, robot_path)
    print(f"[Init] Joints found ({len(joint_map)}):")
    for name, path in sorted(joint_map.items()):
        print(f"  {name}: {path}")

    # ── Pre-apply DriveAPI to all joints BEFORE simulation starts ─────────────
    # USD physics properties must be set before sim.play(); applying them after
    # sim.play() is unreliable.
    for name, path in joint_map.items():
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            continue
        drive = UsdPhysics.DriveAPI.Get(prim, "angular")
        if not drive:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        is_finger = "finger" in name or name.endswith("_hand")
        if is_finger:
            drive.GetStiffnessAttr().Set(5000.0)
            drive.GetDampingAttr().Set(300.0)
        else:
            drive.GetStiffnessAttr().Set(1000.0)
            drive.GetDampingAttr().Set(100.0)
        drive.GetMaxForceAttr().Set(10000.0)
        drive.GetTargetPositionAttr().Set(0.0)
    print("[Init] DriveAPI pre-applied to all joints.")

    camera_annotators = _setup_camera_renderers(stage)

    # ── ROS2 subscriber in background thread ──────────────────────────────────
    rclpy.init()
    node = _Subscriber(_known.topic)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    print(f"[Init] Quest browser → https://{_get_host_ip()}:4443")
    print("[Init] Hold BOTH left + right squeeze grips, then move controllers.")
    print(f"[Init] Cube stack task active ({len(cubes)} cubes).")
    if camera_annotators:
        print("[Init] Camera topics:")
        for key in sorted(camera_annotators):
            print(f"  {key}: {_camera_topic_for_key(key)}")

    # ── Start simulation (isaaclab SimulationContext pattern) ─────────────────
    sim.reset()
    sim.play()

    cmd_count = 0
    step_count = 0
    active_cmd: dict[str, float] | None = None

    # ── Simulation loop ───────────────────────────────────────────────────────
    try:
        while app.is_running():
            sim.step()
            step_count += 1

            cmd = _pop_pending_cmd()

            if cmd:
                active_cmd = cmd
                cmd_count += 1
                if cmd_count == 1 or cmd_count % 100 == 0:
                    sample = {k: round(np.degrees(v), 1) for k, v in list(cmd.items())[:4]}
                    grippers = {
                        k: round(float(v), 4)
                        for k, v in cmd.items()
                        if "finger" in k or k.endswith("_hand")
                    }
                    print(f"[Teleop] cmd #{cmd_count}: {sample}... gripper={grippers}")
            if active_cmd:
                for joint_name, angle_rad in active_cmd.items():
                    prim_path = joint_map.get(joint_name)
                    is_finger = "finger" in joint_name or joint_name.endswith("_hand")
                    if prim_path:
                        _drive_joint(stage, prim_path, angle_rad, is_finger=is_finger)
                    if joint_name == "openarm_left_finger_joint1":
                        _drive_gripper_side(stage, joint_map, "left", angle_rad)
                    elif joint_name == "openarm_right_finger_joint1":
                        _drive_gripper_side(stage, joint_map, "right", angle_rad)
                if cmd or step_count % 5 == 0:
                    node.publish_joint_state(
                        {name: value for name, value in active_cmd.items() if name in joint_map}
                    )
            elif step_count % 1000 == 0:
                print(f"[Status] {step_count} steps, {cmd_count} commands received so far")

            if camera_annotators and step_count % max(1, _known.camera_interval) == 0:
                for camera_key, annotator in camera_annotators.items():
                    try:
                        frame = annotator.get_data()
                        if isinstance(frame, dict):
                            frame = frame.get("data")
                        if frame is not None:
                            node.publish_rgb(camera_key, np.asarray(frame))
                    except Exception as exc:
                        if step_count % 1000 == 0:
                            print(f"[Camera] Failed to publish {camera_key}: {exc}")

    except KeyboardInterrupt:
        print("\n[Shutdown] Ctrl+C")
    finally:
        rclpy.shutdown()
        app.close()


if __name__ == "__main__":
    main()
