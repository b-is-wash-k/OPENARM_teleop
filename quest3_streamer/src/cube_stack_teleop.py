#!/usr/bin/env python3
"""
Quest 3 → MuJoCo OpenArm Bimanual Cube Stack Teleoperation

Scene layout:
  Robot elevated 0.5 m on a pedestal (programmatic lift of openarm_body_link0).
  Work table: top at z=1.10 m, centred at x=0.45 m.
  Arms start at preferred joint config → TCP at (0.19, ±0.51, 1.52 m).
  Cubes at z=1.12 m, y=±0.25 m — reachable with Δ≈(0.17, 0.26, 0.40) m.

Controls:
  GRIP (side squeeze)  — deadman switch, must hold to move arm (per-arm)
  INDEX TRIGGER        — close gripper (0 = open, 1 = closed)
  Move controller      — arm end-effector follows (only while grip held)

Auto-reset: cubes that fall below z=0.90 m (off table) are auto-returned.

Run:
    source .venv/bin/activate && source /opt/ros/jazzy/setup.bash
    python src/cube_stack_teleop.py              # deadman ON (default)
    python src/cube_stack_teleop.py --no-deadman # arms always enabled
"""

import argparse
import os
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy

# ── Paths ──────────────────────────────────────────────────────────────────────
SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(SCRIPT_DIR)
MODEL_PATH   = os.path.realpath(os.path.join(
    PROJECT_ROOT, "..", "openarm", "openarm", "simulation", "models", "cube_stack_scene.xml"))

# ── Tuning ────────────────────────────────────────────────────────────────────
SCALE          = 1.0    # hand-to-robot scale (1.0 = 1:1)
SMOOTH         = 0.4    # IK target smoothing (0 = raw, 0.9 = heavy)
CALIB_FRAMES   = 60     # frames still for initial calibration
CALIB_STILL_M  = 0.015  # max jitter (m) that counts as "still"
DEADMAN_THRESH = 0.3    # squeeze axis threshold
IK_GAIN        = 0.5    # DLS step gain per iteration
IK_DAMP        = 0.005  # DLS damping
IK_ITERS       = 5      # IK iterations per viewer frame
MAX_STEP_M     = 0.05   # max IK target step per frame
GRIPPER_OPEN   = 0.044  # metres (prismatic joint upper limit)
GRIPPER_CLOSED = 0.0
SUBSTEPS       = 5      # physics steps per viewer frame
DROP_Z         = 0.90   # cubes below this z are auto-reset (m, ~0.20 below table top)
TABLE_TOP_Z    = 1.10   # world z of work table top surface (m)
TCP_MIN_Z      = TABLE_TOP_Z - 0.05  # IK target z floor — stops arm going through table

# Grasping
GRASP_DIST     = 0.07   # m — cube within this radius of TCP → graspable
GRASP_THRESH   = 0.75   # trigger value to activate grasp
RELEASE_THRESH = 0.20   # trigger value to release

# Robot lift: openarm_body_link0 is shifted +Z after model load so the robot
# appears to sit on the pedestal.  Must match pedestal height in scene XML.

# ROBOT_LIFT_Z   = 0.50   # metres

ROBOT_LIFT_Z   = 0.6  #metres

# Preferred joint config — arms start here (natural working position, not zero-down).
# FK at preferred: TCP ≈ (0.19, ±0.51, 1.02+ROBOT_LIFT_Z) = (0.19, ±0.51, 1.52 m).
L_PREFERRED = np.array([-1.047, -1.571, 0.0, 1.222, 0.0, 0.0, 0.0])
R_PREFERRED = np.array([ 1.047,  1.571, 0.0, 1.222, 0.0, 0.0, 0.0])

# VR → Robot coordinate transform
# Quest: X=right, Y=up, Z=back  →  Robot: X=forward, Y=left, Z=up
T_VR2ROBOT = np.array([[ 0,  0, -1],
                        [-1,  0,  0],
                        [ 0,  1,  0]], dtype=float)

LEFT_JOINTS  = [f"openarm_left_joint{i}"  for i in range(1, 8)]
RIGHT_JOINTS = [f"openarm_right_joint{i}" for i in range(1, 8)]
LEFT_TCP     = "openarm_left_hand_tcp"
RIGHT_TCP    = "openarm_right_hand_tcp"
CUBE_JOINTS  = ["cube_A_joint", "cube_B_joint", "cube_C_joint"]
CUBE_BODIES  = ["cube_A",       "cube_B",       "cube_C"]


# ─────────────────────────────────────────────────────────────────────────────
class ArmState:
    def __init__(self, name: str):
        self.name          = name
        self.raw_pos       = None
        self.calibrated    = False
        self.calib_ref     = None
        self.calib_cnt     = 0
        self.anchor_vr     = None
        self.anchor_robot  = None
        self.smooth_pos    = None
        self.enabled       = False
        self.prev_enabled  = False
        self.gripper       = GRIPPER_OPEN
        self.squeeze       = 0.0
        self.trigger       = 0.0

        # Grasping state
        self.grasped_idx   = None   # index into CUBE_BODIES, or None
        self.grasp_offset  = None   # cube_pos - tcp_pos at grasp time (3,)


# ─────────────────────────────────────────────────────────────────────────────
class TeleopNode(Node):
    def __init__(self, no_deadman: bool):
        super().__init__("quest_cube_stack_teleop")
        self.no_deadman = no_deadman
        self.left  = ArmState("left")
        self.right = ArmState("right")
        self.lock  = threading.Lock()

        self.create_subscription(PoseStamped, "/quest/left_hand/pose",
                                 lambda m: self._pose_cb(m, self.left),  10)
        self.create_subscription(PoseStamped, "/quest/right_hand/pose",
                                 lambda m: self._pose_cb(m, self.right), 10)
        self.create_subscription(Joy, "/quest/left_hand/inputs",
                                 lambda m: self._input_cb(m, self.left),  10)
        self.create_subscription(Joy, "/quest/right_hand/inputs",
                                 lambda m: self._input_cb(m, self.right), 10)

        mode = "ALWAYS ON (--no-deadman)" if no_deadman else "GRIP to enable each arm"
        self.get_logger().info(f"Deadman mode: {mode}")
        self.get_logger().info("Waiting for Quest 3 poses...")

    def _pose_cb(self, msg: PoseStamped, arm: ArmState):
        p = msg.pose.position
        with self.lock:
            arm.raw_pos = np.array([p.x, p.y, p.z])

    def _input_cb(self, msg: Joy, arm: ArmState):
        with self.lock:
            if len(msg.axes) >= 2:
                arm.trigger = float(msg.axes[0])
                arm.squeeze = float(msg.axes[1])
            arm.enabled = self.no_deadman or (arm.squeeze > DEADMAN_THRESH)
            arm.gripper = GRIPPER_OPEN - arm.trigger * (GRIPPER_OPEN - GRIPPER_CLOSED)


# ─────────────────────────────────────────────────────────────────────────────
def joint_indices(model, names):
    qp, dof = [], []
    for n in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        if jid < 0:
            raise ValueError(f"Joint '{n}' not found in model")
        qp.append(model.jnt_qposadr[jid])
        dof.append(model.jnt_dofadr[jid])
    return np.array(qp, dtype=int), np.array(dof, dtype=int)


def joint_limits(model, names):
    lo, hi = [], []
    for n in names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
        lo.append(model.jnt_range[jid, 0])
        hi.append(model.jnt_range[jid, 1])
    return np.array(lo), np.array(hi)


def get_body_pos(model, data, body_name):
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return data.xpos[bid].copy()


def ik_step(model, data, body_name, dof_ids, target, gain=IK_GAIN, damp=IK_DAMP):
    """Damped least-squares IK step. Returns (dq[7], position_error_m)."""
    bid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    curr = data.xpos[bid].copy()
    dp   = target - curr
    err  = float(np.linalg.norm(dp))
    jacp = np.zeros((3, model.nv))
    mujoco.mj_jacBody(model, data, jacp, None, bid)
    J  = jacp[:, dof_ids]
    dq = J.T @ np.linalg.solve(J @ J.T + damp**2 * np.eye(3), dp)
    return dq * gain, err


# ─────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Quest3 → MuJoCo cube stack teleop")
    parser.add_argument("--no-deadman", action="store_true",
                        help="Arms always follow (no grip required)")
    args = parser.parse_args()

    if not os.path.isfile(MODEL_PATH):
        print(f"ERROR: MuJoCo model not found:\n  {MODEL_PATH}")
        sys.exit(1)

    print(f"Loading: {MODEL_PATH}")
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data  = mujoco.MjData(model)

    # ── Arm joint indices ─────────────────────────────────────────────────────
    l_qpos, l_dof = joint_indices(model, LEFT_JOINTS)
    r_qpos, r_dof = joint_indices(model, RIGHT_JOINTS)
    l_lo, l_hi    = joint_limits(model, LEFT_JOINTS)
    r_lo, r_hi    = joint_limits(model, RIGHT_JOINTS)

    lf_jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "openarm_left_finger_joint1")
    rf_jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "openarm_right_finger_joint1")
    lf_qpos = model.jnt_qposadr[lf_jid]
    rf_qpos = model.jnt_qposadr[rf_jid]
    lf_dof  = model.jnt_dofadr[lf_jid]
    rf_dof  = model.jnt_dofadr[rf_jid]

    l_tcp_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, LEFT_TCP)
    r_tcp_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, RIGHT_TCP)

    # ── Cube freejoint indices (qpos: 7 per cube, qvel: 6 per cube) ───────────
    cube_qpos_slices = []
    cube_qvel_slices = []
    cube_body_ids    = []
    for jname, bname in zip(CUBE_JOINTS, CUBE_BODIES):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY,  bname)
        qpa = model.jnt_qposadr[jid]
        dfa = model.jnt_dofadr[jid]
        cube_qpos_slices.append(slice(qpa, qpa + 7))
        cube_qvel_slices.append(slice(dfa, dfa + 6))
        cube_body_ids.append(bid)

    # ── Lift robot body to match pedestal height ──────────────────────────────
    # Programmatically shift openarm_body_link0 up by ROBOT_LIFT_Z so the robot
    # sits on the pedestal defined in the scene XML.
    robot_base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "openarm_body_link0")
    model.body_pos[robot_base_id][2] += ROBOT_LIFT_Z
    print(f"[SETUP] Robot lifted by {ROBOT_LIFT_Z} m (pedestal).")

    # ── Initialise arms at preferred joint config (not zero-pointing-down) ────
    # This places TCP at ≈ (0.19, ±0.51, 1.52 m) — a natural working height.
    data.qpos[l_qpos] = L_PREFERRED.copy()
    data.qpos[r_qpos] = R_PREFERRED.copy()

    # ── Forward kinematics → home TCP positions (from preferred config) ────────
    mujoco.mj_forward(model, data)
    left_home  = get_body_pos(model, data, LEFT_TCP)
    right_home = get_body_pos(model, data, RIGHT_TCP)
    print(f"[FK] Left  home (preferred): {np.round(left_home,  3)}")
    print(f"[FK] Right home (preferred): {np.round(right_home, 3)}")
    print(f"     Work table top at z=1.10 m — delta to cube_A: "
          f"{np.round(np.array([0.35, 0.25, 1.12]) - left_home, 3)}")

    # Store initial cube poses for auto-reset
    cube_init_qpos = [data.qpos[s].copy() for s in cube_qpos_slices]

    # ── ROS2 node ─────────────────────────────────────────────────────────────
    rclpy.init()
    node = TeleopNode(no_deadman=args.no_deadman)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    print()
    if args.no_deadman:
        print("Mode: NO DEADMAN — arms always follow")
    else:
        print("Mode: DEADMAN ON")
        print("  Hold GRIP (side squeeze) to enable each arm independently")
        print("  Release GRIP → arm freezes in place")
    print("  INDEX TRIGGER → close gripper")
    print("  Cubes auto-reset if they fall off the table (z < 0.90 m)")
    print()
    print("Step 1: Hold both controllers still (~2s) for initial calibration")
    print("Step 2: Hold GRIP and move hands to control arms")
    print()
    print("Scene: red cube (left), blue cube (right), green cube (far centre)")
    print()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.distance  = 2.2
        viewer.cam.elevation = -20
        viewer.cam.azimuth   = -150
        viewer.cam.lookat[:] = [0.3, 0.0, 1.1]

        last_log = time.time()

        # IK state (desired qpos, updated once per viewer frame)
        l_qpos_des = data.qpos[l_qpos].copy()
        r_qpos_des = data.qpos[r_qpos].copy()
        l_grip_des = GRIPPER_OPEN
        r_grip_des = GRIPPER_OPEN

        while viewer.is_running():

            # ── Read ROS state snapshot ───────────────────────────────────────
            with node.lock:
                l_raw      = node.left.raw_pos.copy()  if node.left.raw_pos  is not None else None
                r_raw      = node.right.raw_pos.copy() if node.right.raw_pos is not None else None
                l_enabled  = node.left.enabled
                r_enabled  = node.right.enabled
                l_prev     = node.left.prev_enabled
                r_prev     = node.right.prev_enabled
                l_grip_des = node.left.gripper
                r_grip_des = node.right.gripper
                l_trigger  = node.left.trigger
                r_trigger  = node.right.trigger

            # ── Initial calibration ───────────────────────────────────────────
            for arm, raw, home in [(node.left,  l_raw, left_home),
                                   (node.right, r_raw, right_home)]:
                if arm.calibrated or raw is None:
                    continue
                if arm.calib_ref is None:
                    arm.calib_ref = raw.copy()
                    arm.calib_cnt = 0
                elif np.linalg.norm(raw - arm.calib_ref) < CALIB_STILL_M:
                    arm.calib_cnt += 1
                    if arm.calib_cnt >= CALIB_FRAMES:
                        arm.calibrated   = True
                        arm.smooth_pos   = home.copy()
                        arm.anchor_vr    = raw.copy()
                        arm.anchor_robot = home.copy()
                        print(f"[CAL] {arm.name.upper()} calibrated — home: {np.round(home, 3)}")
                else:
                    arm.calib_ref = raw.copy()
                    arm.calib_cnt = 0

            # ── Deadman rising / falling edge ─────────────────────────────────
            for arm, raw, enabled, prev in [
                    (node.left,  l_raw, l_enabled, l_prev),
                    (node.right, r_raw, r_enabled, r_prev)]:
                if arm.calibrated and raw is not None:
                    if enabled and not prev:
                        arm.anchor_vr    = raw.copy()
                        arm.anchor_robot = arm.smooth_pos.copy()
                        print(f"[GRIP] {arm.name.upper()} ENABLED  — anchored at "
                              f"{np.round(arm.anchor_robot, 3)}")
                    elif not enabled and prev:
                        print(f"[GRIP] {arm.name.upper()} DISABLED — frozen  at "
                              f"{np.round(arm.smooth_pos, 3)}")
                arm.prev_enabled = enabled

            # ── Update IK targets (only when enabled) ─────────────────────────
            for arm, raw, enabled in [
                    (node.left,  l_raw, l_enabled),
                    (node.right, r_raw, r_enabled)]:
                if not arm.calibrated or raw is None or not enabled:
                    continue
                if arm.anchor_vr is None:
                    continue
                delta_vr    = raw - arm.anchor_vr
                delta_robot = T_VR2ROBOT @ delta_vr * SCALE
                desired     = arm.anchor_robot + delta_robot
                if arm.smooth_pos is not None:
                    step = desired - arm.smooth_pos
                    norm = np.linalg.norm(step)
                    if norm > MAX_STEP_M:
                        desired = arm.smooth_pos + step * (MAX_STEP_M / norm)
                desired[2] = max(desired[2], TCP_MIN_Z)  # clamp: don't go through table
                arm.smooth_pos = SMOOTH * arm.smooth_pos + (1 - SMOOTH) * desired

            # ── IK: compute desired arm joint positions ────────────────────────
            # Run IK iterations on a temporary forward pass (no physics yet)
            mujoco.mj_forward(model, data)
            l_err = r_err = 0.0
            for _ in range(IK_ITERS):
                mujoco.mj_forward(model, data)
                if node.left.calibrated and node.left.smooth_pos is not None:
                    dq, l_err = ik_step(model, data, LEFT_TCP, l_dof,
                                        node.left.smooth_pos)
                    data.qpos[l_qpos] = np.clip(data.qpos[l_qpos] + dq, l_lo, l_hi)
                if node.right.calibrated and node.right.smooth_pos is not None:
                    dq, r_err = ik_step(model, data, RIGHT_TCP, r_dof,
                                        node.right.smooth_pos)
                    data.qpos[r_qpos] = np.clip(data.qpos[r_qpos] + dq, r_lo, r_hi)

            # Snapshot the IK solution — will be enforced during physics substeps
            l_qpos_des = data.qpos[l_qpos].copy()
            r_qpos_des = data.qpos[r_qpos].copy()

            # ── Grasp / release logic (programmatic attach) ───────────────────
            # MuJoCo kinematic arms can't exert contact forces to lift cubes.
            # Instead: when trigger closes near a cube, we track cube pos to TCP.
            # When trigger releases, the cube is left at its current position.
            tcp_bids     = [l_tcp_bid,      r_tcp_bid]
            triggers     = [l_trigger,      r_trigger]
            arm_states   = [node.left,      node.right]

            for tcp_bid, trigger, arm in zip(tcp_bids, triggers, arm_states):
                tcp_pos = data.xpos[tcp_bid].copy()

                if arm.grasped_idx is None:
                    # Try to grasp: trigger pressed + cube within reach
                    if trigger > GRASP_THRESH:
                        best_i, best_d = None, GRASP_DIST
                        for i, cbid in enumerate(cube_body_ids):
                            # Skip cubes already held by the other arm
                            other = arm_states[1] if arm is arm_states[0] else arm_states[0]
                            if other.grasped_idx == i:
                                continue
                            d = float(np.linalg.norm(data.xpos[cbid] - tcp_pos))
                            if d < best_d:
                                best_d, best_i = d, i
                        if best_i is not None:
                            arm.grasped_idx  = best_i
                            arm.grasp_offset = data.xpos[cube_body_ids[best_i]] - tcp_pos
                            print(f"[GRASP] {arm.name.upper()} grasped {CUBE_BODIES[best_i]}"
                                  f"  dist={best_d*100:.1f}cm")
                else:
                    # Holding: track cube to TCP + grasp_offset
                    if trigger < RELEASE_THRESH:
                        print(f"[RELEASE] {arm.name.upper()} released {CUBE_BODIES[arm.grasped_idx]}"
                              f"  at z={data.xpos[cube_body_ids[arm.grasped_idx]][2]*100:.1f}cm")
                        arm.grasped_idx  = None
                        arm.grasp_offset = None
                    else:
                        # Move cube with TCP
                        target_pos = tcp_pos + arm.grasp_offset
                        qs = cube_qpos_slices[arm.grasped_idx]
                        vs = cube_qvel_slices[arm.grasped_idx]
                        data.qpos[qs][:3] = target_pos
                        data.qvel[vs][:3] = 0.0

            # ── Physics substeps (cubes obey gravity/contacts) ────────────────
            for _ in range(SUBSTEPS):
                # Apply grasped cube positions before each step
                for arm in arm_states:
                    if arm.grasped_idx is not None:
                        tcp_pos = data.xpos[tcp_bids[arm_states.index(arm)]].copy()
                        qs = cube_qpos_slices[arm.grasped_idx]
                        vs = cube_qvel_slices[arm.grasped_idx]
                        data.qpos[qs][:3] = tcp_pos + arm.grasp_offset
                        data.qvel[vs][:3] = 0.0

                mujoco.mj_step(model, data)
                # Kinematic override: force arms to IK solution, zero velocities
                data.qpos[l_qpos] = l_qpos_des
                data.qpos[r_qpos] = r_qpos_des
                data.qvel[l_dof]  = 0.0
                data.qvel[r_dof]  = 0.0
                # Gripper
                data.qpos[lf_qpos] = float(np.clip(l_grip_des, GRIPPER_CLOSED, GRIPPER_OPEN))
                data.qpos[rf_qpos] = float(np.clip(r_grip_des, GRIPPER_CLOSED, GRIPPER_OPEN))
                data.qvel[lf_dof]  = 0.0
                data.qvel[rf_dof]  = 0.0

            # ── Auto-reset cubes that fell off the table ──────────────────────
            for i, (bid, qps, qvs, init_qp) in enumerate(
                    zip(cube_body_ids, cube_qpos_slices, cube_qvel_slices, cube_init_qpos)):
                if data.xpos[bid][2] < DROP_Z:
                    data.qpos[qps] = init_qp.copy()
                    data.qvel[qvs] = 0.0
                    print(f"[RESET] {CUBE_BODIES[i]} fell off table — reset to spawn position")

            mujoco.mj_forward(model, data)
            viewer.sync()

            # ── Periodic status log ───────────────────────────────────────────
            now = time.time()
            if now - last_log > 3.0:
                lc = node.left.calibrated
                rc = node.right.calibrated
                if lc and rc:
                    cube_z = [np.round(data.xpos[bid][2], 3) for bid in cube_body_ids]
                    print(f"[STATUS] L:{'ON ' if l_enabled else 'OFF'} err={l_err*100:.1f}cm  "
                          f"R:{'ON ' if r_enabled else 'OFF'} err={r_err*100:.1f}cm  "
                          f"cubes z={cube_z}")
                elif not lc and not rc:
                    print("[CAL] Hold controllers still to calibrate...")
                elif not lc:
                    print("[CAL] Waiting for LEFT controller...")
                else:
                    print("[CAL] Waiting for RIGHT controller...")
                last_log = now

    rclpy.shutdown()
    print("Viewer closed.")


if __name__ == "__main__":
    main()
