"""
OpenArm VR Delta Teleoperation
================================
Design:
  - Robot starts with all joints at zero (arms pointing down)
  - Hold your arms in the same "pointing down" pose, then press SQUEEZE to calibrate
  - From that point, position AND rotation deltas from your reference pose
    are applied to the robot EE via cuRobo IK
  - Press SQUEEZE again at any time to recalibrate

FK at all-zeros (from cuRobo):
  Left  EE: pos=[0.000,  0.616, 0.122], quat=[0.707, -0.707, 0, 0]
  Right EE: pos=[0.000, -0.616, 0.122], quat=[0.707,  0.707, 0, 0]

Quest 2 world axes (verified by calibration):
  x: your left(-) / right(+)
  y: toward robot(-) / away from robot(+)
  z: down(-) / up(+)

Robot EE frame (per-arm base frame):
  x: forward
  y: sideways
  z: up
"""

from isaacsim import SimulationApp

launcher_args = {
    "headless": False,
    "experience": "/home/vision/workspace/simlab/.venv-isaacsim/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.base.xr.vr.kit"
}
app = SimulationApp(launcher_args)

import numpy as np
import torch
import omni.kit.app
import omni.kit.xr.core as xr_core
import omni.usd
import isaaclab.sim as sim_utils
from pxr import UsdPhysics, Usd

# ─── Paths ────────────────────────────────────────────────────────────────────
BIMANUAL_USD = (
    "/home/vision/humanoids/openarm_isaac_lab/source/openarm/openarm/tasks"
    "/manager_based/openarm_manipulation/usds/openarm_bimanual/openarm_bimanual.usd"
)
ROBOT_PRIM_PATH = "/World/OpenArm"
CUROBO_DIR = "/home/vision/humanoids/openarm_module/config/curobo"

# ─── Joint paths ──────────────────────────────────────────────────────────────
LEFT_JOINTS = [f"joints/openarm_left_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"joints/openarm_right_joint{i}" for i in range(1, 8)]

# ─── FK home poses (all joints = 0) ──────────────────────────────────────────
LEFT_HOME_POS  = np.array([0.000,  0.6161, 0.1225])
RIGHT_HOME_POS = np.array([0.000, -0.6161, 0.1225])

def quat_to_rot(w, x, y, z) -> np.ndarray:
    return np.array([
        [1-2*(y*y+z*z),   2*(x*y-w*z),   2*(x*z+w*y)],
        [  2*(x*y+w*z), 1-2*(x*x+z*z),   2*(y*z-w*x)],
        [  2*(x*z-w*y),   2*(y*z+w*x), 1-2*(x*x+y*y)],
    ])

LEFT_HOME_ROT  = quat_to_rot( 0.7071, -0.7071, 0.0, 0.0)
RIGHT_HOME_ROT = quat_to_rot( 0.7071,  0.7071, 0.0, 0.0)

# ─── Workspace scale ─────────────────────────────────────────────────────────
WORKSPACE_SCALE = 1.0

# ─── XR Extensions ────────────────────────────────────────────────────────────
XR_EXTENSIONS = [
    "omni.kit.xr.core",
    "omni.kit.xr.system.openxr",
    "omni.kit.xr.profile.vr",
    "isaacsim.xr.input_devices",
]


def enable_xr_extensions():
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in XR_EXTENSIONS:
        if not ext_manager.is_extension_enabled(ext):
            ext_manager.set_extension_enabled_immediate(ext, True)


def pose_to_matrix(pose) -> np.ndarray:
    return np.array([
        [pose[0][0], pose[1][0], pose[2][0], pose[3][0]],
        [pose[0][1], pose[1][1], pose[2][1], pose[3][1]],
        [pose[0][2], pose[1][2], pose[2][2], pose[3][2]],
        [pose[0][3], pose[1][3], pose[2][3], pose[3][3]],
    ])


def get_controller_pose(xr: xr_core.XRCore, hand: str):
    """Returns (position [3], rotation [3,3]) in world frame, or (None, None)."""
    for device in xr.get_all_input_devices():
        if f"hand/{hand}" not in str(device.get_name()):
            continue
        pose_names = device.get_pose_names()
        if not pose_names:
            return None, None
        target = pose_names[0]
        for pn in pose_names:
            if "grip" in str(pn):
                target = pn
                break
        pose = device.get_pose(target)
        if pose is None:
            return None, None
        mat = pose_to_matrix(pose)
        return mat[:3, 3], mat[:3, :3]
    return None, None


def is_squeeze_pressed(xr: xr_core.XRCore, hand: str, token) -> bool:
    if token is None:
        return False
    for device in xr.get_all_input_devices():
        if f"hand/{hand}" not in str(device.get_name()):
            continue
        try:
            val = device.get_input_gesture_value(token, token)
            return float(val) > 0.5
        except Exception:
            pass
    return False


def set_joint_deg(stage: Usd.Stage, joint_path: str, angle_deg: float):
    prim = stage.GetPrimAtPath(joint_path)
    if not prim.IsValid():
        return
    drive = UsdPhysics.DriveAPI.Get(prim, "angular")
    if not drive:
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
    drive.GetTargetPositionAttr().Set(float(angle_deg))
    drive.GetStiffnessAttr().Set(1000.0)
    drive.GetDampingAttr().Set(100.0)


def world_delta_to_robot_left(pos_delta: np.ndarray) -> np.ndarray:
    return np.array([
        -pos_delta[2],   # robot forward  = world z
        -pos_delta[1],   # robot sideways = world -y
        -pos_delta[0],   # robot up       = world x
    ]) * WORKSPACE_SCALE


def world_delta_to_robot_right(pos_delta: np.ndarray) -> np.ndarray:
    return np.array([
        -pos_delta[2],   # robot forward  = world z
         pos_delta[1],   # robot sideways = world y (flipped)
         pos_delta[0],   # robot up       = world x (flipped)
    ]) * WORKSPACE_SCALE


def rot_matrix_to_quat(R: np.ndarray):
    """Returns [w, x, y, z]."""
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return np.array([0.25/s, (R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s])
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
        return np.array([(R[2,1]-R[1,2])/s, 0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s])
    elif R[1,1] > R[2,2]:
        s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
        return np.array([(R[0,2]-R[2,0])/s, (R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s])
    else:
        s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
        return np.array([(R[1,0]-R[0,1])/s, (R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s])


def setup_curobo(config_path: str):
    """Initialize cuRobo IK solver on CUDA."""
    from curobo.types.robot import RobotConfig
    from curobo.types.base import TensorDeviceType
    from curobo.util_file import load_yaml
    from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

    tensor_args = TensorDeviceType(device=torch.device("cuda:0"), dtype=torch.float32)
    robot_cfg = RobotConfig.from_dict(
        load_yaml(config_path)["robot_cfg"],
        tensor_args=tensor_args
    )
    ik_config = IKSolverConfig.load_from_robot_config(
        robot_cfg,
        rotation_threshold=0.05,
        position_threshold=0.005,
        num_seeds=20,
        self_collision_check=False,
        self_collision_opt=False,
        tensor_args=tensor_args,
    )
    return IKSolver(ik_config)


def solve_ik(solver, pos: np.ndarray, rot: np.ndarray):
    """
    Solve IK for target EE pose in arm base frame.
    Returns joint angles in degrees [7], or None if failed.
    """
    from curobo.types.math import Pose

    q = rot_matrix_to_quat(rot)
    target = Pose(
        position=torch.tensor([pos], dtype=torch.float32, device="cuda:0"),
        quaternion=torch.tensor([[q[0], q[1], q[2], q[3]]],
                                dtype=torch.float32, device="cuda:0"),
    )
    result = solver.solve_single(target)
    if result.success.item():
        return np.degrees(result.solution.squeeze().cpu().numpy())
    return None

def remap_rot(delta_rot: np.ndarray) -> np.ndarray:
    """Remap controller rotation delta to robot frame."""
    P = np.array([
        [0,  0, 1],
        [1,  0, 0],
        [0,  1, 0],
    ], dtype=float)
    result = P @ delta_rot @ P.T

    angle = np.arccos(np.clip((np.trace(result) - 1) / 2, -1, 1))
    if abs(angle) < 1e-6:
        return result

    axis = np.array([
        result[2, 1] - result[1, 2],
        result[0, 2] - result[2, 0],
        result[1, 0] - result[0, 1],
    ]) / (2 * np.sin(angle))

    # Negate Z (twist) and Y (left/right tilt)
    axis[2] *= -1
    axis[0] *= -1

    c, s = np.cos(angle), np.sin(angle)
    t = 1 - c
    x, y, z = axis
    return np.array([
        [t*x*x + c,   t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z, t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y, t*y*z + s*x, t*z*z + c  ],
    ])

def main():
    import os

    # ── Sim setup (CPU pipeline for USD DriveAPI) ─────────────────────────────
    sim_cfg = sim_utils.SimulationCfg(dt=0.01, device="cpu")
    sim = sim_utils.SimulationContext(sim_cfg)
    sim.set_camera_view([2.0, 2.0, 2.0], [0.0, 0.0, 0.5])

    cfg = sim_utils.UsdFileCfg(usd_path=BIMANUAL_USD)
    cfg.func(ROBOT_PRIM_PATH, cfg)
    enable_xr_extensions()
    sim.reset()
    sim.play()

    stage = omni.usd.get_context().get_stage()

    for jp in LEFT_JOINTS + RIGHT_JOINTS:
        set_joint_deg(stage, f"{ROBOT_PRIM_PATH}/{jp}", 0.0)

    ok = sum(1 for j in LEFT_JOINTS + RIGHT_JOINTS
             if stage.GetPrimAtPath(f"{ROBOT_PRIM_PATH}/{j}").IsValid())
    print(f"Joint paths verified: {ok}/{len(LEFT_JOINTS+RIGHT_JOINTS)}")

    # ── cuRobo IK (CUDA) ──────────────────────────────────────────────────────
    ik_left = ik_right = None
    try:
        print("Initializing cuRobo IK (CUDA)...")
        ik_left  = setup_curobo(os.path.join(CUROBO_DIR, "openarm_left.yml"))
        ik_right = setup_curobo(os.path.join(CUROBO_DIR, "openarm_right.yml"))
        print("cuRobo IK ready.")
    except Exception as e:
        print(f"cuRobo init failed: {e}")
        print("Will use direct joint mapping fallback.")

    print("\nEnable VR from the XR tab in Isaac Sim.")
    print("Hold arms pointing DOWN, then auto-calibration will begin.\n")

    # ── State ──────────────────────────────────────────────────────────────────
    xr = None
    xr_ready = False
    xr_ready_frame = None
    frame = 0
    squeeze_token = None
    calibrated = False

    left_ref_pos  = None
    right_ref_pos = None
    left_ref_rot  = None
    right_ref_rot = None

    last_left_angles  = np.zeros(7)
    last_right_angles = np.zeros(7)

    squeeze_was_pressed = False
    prev_L = prev_R = None

    while app.is_running():
        sim.step()
        frame += 1

        if xr is None:
            try:
                xr = xr_core.XRCore.get_singleton()
            except Exception:
                continue

        # ── Wait for XR devices ───────────────────────────────────────────────
        if not xr_ready:
            for d in xr.get_all_input_devices():
                if "hand/left" in str(d.get_name()) and len(d.get_pose_names()) > 1:
                    xr_ready = True
                    xr_ready_frame = frame
                    print("XR ready. Hold arms pointing DOWN - auto-calibrating in 3s.")
                    for dev in xr.get_all_input_devices():
                        if "hand" in str(dev.get_name()) and "left" in str(dev.get_name()):
                            for inp in dev.get_input_names():
                                if "squeeze" in str(inp).lower():
                                    squeeze_token = inp
                                    break
                    break
            if not xr_ready:
                if frame % 300 == 0:
                    print("Waiting for XR... (enable VR from XR tab)")
                continue

        if frame % 1 != 0:
            continue

        try:
            lpos, lrot = get_controller_pose(xr, "left")
            rpos, rrot = get_controller_pose(xr, "right")
            if lpos is None or rpos is None:
                continue

            # ── Calibration ───────────────────────────────────────────────
            sq = (is_squeeze_pressed(xr, "left",  squeeze_token) or
                  is_squeeze_pressed(xr, "right", squeeze_token))

            if sq and not squeeze_was_pressed:
                left_ref_pos  = lpos.copy()
                right_ref_pos = rpos.copy()
                left_ref_rot  = lrot.copy()
                right_ref_rot = rrot.copy()
                prev_L = prev_R = None
                calibrated = True
                print("Recalibrated.")
            squeeze_was_pressed = sq

            if not calibrated:
                frames_since_ready = frame - xr_ready_frame
                if frames_since_ready > 300:  # ~3 seconds
                    left_ref_pos  = lpos.copy()
                    right_ref_pos = rpos.copy()
                    left_ref_rot  = lrot.copy()
                    right_ref_rot = rrot.copy()
                    calibrated = True
                    print("Auto-calibrated. Move your arms to control the robot.")
                else:
                    secs_left = max(0, 3 - int(frames_since_ready / 100))
                    if frames_since_ready % 100 == 0:
                        print(f"Calibrating in {secs_left}s...")
                continue

            # ── World-space deltas ────────────────────────────────────────
            left_world_delta  = lpos - left_ref_pos
            right_world_delta = rpos - right_ref_pos

            left_pos_delta  = world_delta_to_robot_left(left_world_delta)
            right_pos_delta = world_delta_to_robot_right(right_world_delta)

            # ── Rotation deltas ───────────────────────────────────────────
            left_delta_rot   = lrot @ left_ref_rot.T
            right_delta_rot  = rrot @ right_ref_rot.T
            left_target_rot  = LEFT_HOME_ROT  @ remap_rot(left_delta_rot)
            right_target_rot = RIGHT_HOME_ROT @ remap_rot(right_delta_rot)

            # ── Target EE positions ───────────────────────────────────────
            left_target_pos  = LEFT_HOME_POS  + left_pos_delta
            right_target_pos = RIGHT_HOME_POS + right_pos_delta

            # ── Log on movement ───────────────────────────────────────────
            if prev_L is None or np.linalg.norm(lpos - prev_L) > 0.01:
                print(f"L delta={left_pos_delta.round(3)} target={left_target_pos.round(3)}")
                prev_L = lpos.copy()
            if prev_R is None or np.linalg.norm(rpos - prev_R) > 0.01:
                print(f"R delta={right_pos_delta.round(3)} target={right_target_pos.round(3)}")
                prev_R = rpos.copy()

            # ── Solve IK ──────────────────────────────────────────────────
            if ik_left is not None:
                la = solve_ik(ik_left,  left_target_pos,  left_target_rot)
                ra = solve_ik(ik_right, right_target_pos, right_target_rot)
                if la is not None:
                    last_left_angles = la
                else:
                    print(f"L IK failed target={left_target_pos.round(3)}")
                if ra is not None:
                    last_right_angles = ra
                else:
                    print(f"R IK failed target={right_target_pos.round(3)}")
            else:
                d = left_pos_delta
                last_left_angles = np.array([
                    np.clip( d[1] * 60,  -90,  80),
                    np.clip(-90 + d[0] * 60, -190, 10),
                    np.clip( d[1] * 40,  -90,  90),
                    np.clip( d[2] * 60,    0, 140),
                    0.0, 0.0, 0.0,
                ])
                d = right_pos_delta
                last_right_angles = np.array([
                    np.clip( d[1] * 60,  -80, 200),
                    np.clip(-90 + d[0] * 60, -190, 10),
                    np.clip( d[1] * 40,  -90,  90),
                    np.clip( d[2] * 60,    0, 140),
                    0.0, 0.0, 0.0,
                ])

            # ── Drive joints ──────────────────────────────────────────────
            for i, jp in enumerate(LEFT_JOINTS):
                set_joint_deg(stage, f"{ROBOT_PRIM_PATH}/{jp}", last_left_angles[i])
            for i, jp in enumerate(RIGHT_JOINTS):
                set_joint_deg(stage, f"{ROBOT_PRIM_PATH}/{jp}", last_right_angles[i])

        except Exception as e:
            print(f"Teleop error: {e}")
            import traceback
            traceback.print_exc()


if __name__ == "__main__":
    main()
    app.close()