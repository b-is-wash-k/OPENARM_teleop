#!/usr/bin/env python3
"""
VR teleoperation for OpenArm bimanual robot in IsaacSim.

Subscribes to /joint_trajectory published by:
  conda activate teleop_xr && ./scripts/run_openarm_vr_ros2.sh

Run this script via:
  conda activate env_isaacsim (do NOT source /opt/ros)
  ./scripts/run_openarm_vr_isaacsim.sh
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
_known, _ = _parser.parse_known_args()

app = SimulationApp({
    "headless": _known.headless,
    "width": 1920,
    "height": 1080,
})

# Warm up so kit extensions finish loading before importing anything else
# Do NOT load isaacsim.ros2.bridge — it pre-initializes RCL with FastRTPS and
# conflicts with our direct rclpy.init() using CycloneDDS.
for _ in range(30):
    app.update()

# ── All other imports AFTER warmup ───────────────────────────────────────────
try:
    import rclpy
    from rclpy.node import Node
    from trajectory_msgs.msg import JointTrajectory
except ImportError:
    print("ERROR: rclpy not available. Run via run_openarm_vr_isaacsim.sh")
    app.close()
    sys.exit(1)

import numpy as np
import omni.usd
from omni.isaac.core.utils.stage import open_stage
from pxr import Usd, UsdPhysics
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

    # ── SimulationContext wraps the already-open stage ────────────────────────
    # Created AFTER open_stage so it picks up the existing stage & physics scene
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.01, device="cpu"))

    stage = omni.usd.get_context().get_stage()

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

    # ── ROS2 subscriber in background thread ──────────────────────────────────
    rclpy.init()
    node = _Subscriber(_known.topic)
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()

    print(f"[Init] Quest browser → https://{_get_host_ip()}:4443")
    print("[Init] Hold BOTH left + right squeeze grips, then move controllers.")

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
            elif step_count % 1000 == 0:
                print(f"[Status] {step_count} steps, {cmd_count} commands received so far")

    except KeyboardInterrupt:
        print("\n[Shutdown] Ctrl+C")
    finally:
        rclpy.shutdown()
        app.close()


if __name__ == "__main__":
    main()
