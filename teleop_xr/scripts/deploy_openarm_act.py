#!/usr/bin/env python3
"""
Deploy a trained ACT policy on the real OpenArm (bimanual or right-arm-only).

Mirror image of lerobot_recorder.py, inverted:
  - SUBSCRIBES /joint_states                       -> joint observation.state
  - SUBSCRIBES /camera/*/image_raw                 -> camera observations
  - RUNS the ACT policy (WITH its normalization processors) at --fps Hz
  - PUBLISHES a 16-joint action to /joint_trajectory
    joint_trajectory_relay.py splits it to the two arm controllers and the two
    gripper action servers, exactly as during teleop recording.

NORMALIZATION:
  In current lerobot, normalization lives in a separate processor pipeline saved
  next to the checkpoint (policy_preprocessor_*normalizer*.safetensors), NOT in
  the model. Inference MUST be:
      proc   = preprocess(raw_obs)
      action = policy.select_action(proc)
      action = postprocess(action)

BIMANUAL vs RIGHT-ONLY:
  Bimanual policy: 16-dim obs/action, 3 cameras -> published directly.
  Right-only policy (--right-only): 8-dim obs/action (right joints + right
    finger), 2 cameras (head + right_wrist). The 8 outputs are mapped onto the
    right slots of the 16-joint trajectory; the LEFT arm is held at the pose it
    is in when you press ENTER, so the relay still receives all 16 joints.

MODES
  --check     Offline. Runs the policy (with processors) on recorded frames and
              prints per-frame + per-joint |predicted - recorded| action error.
              Verdict is based on per-joint MEAN error (stable); the worst
              single-frame max is informational. No ROS2, no robot. RUN FIRST.
  (default)   Live deployment on hardware.

LIVE PRE-REQS (same stack as recording, minus VR driving the arms)
  T1: ros2 launch openarm_bringup openarm.bimanual.launch.py \
          right_can_interface:=robot_l left_can_interface:=robot_r
  T2: python3 scripts/joint_trajectory_relay.py
  T3: cameras only (demo_with_ros2 --mode ik, grips NOT held)
  T4: this script
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import numpy as np
import torch

from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors


# Full bimanual publish order (must match the relay split:
# left 1-7, right 1-7, left finger, right finger).
JOINT_NAMES: list[str] = [
    "openarm_left_joint1",  "openarm_left_joint2",  "openarm_left_joint3",
    "openarm_left_joint4",  "openarm_left_joint5",  "openarm_left_joint6",
    "openarm_left_joint7",
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7",
    "openarm_left_finger_joint1",
    "openarm_right_finger_joint1",
]
N_JOINTS = len(JOINT_NAMES)

# Right-only policy output order (8: right joints 1-7 + right finger)
RIGHT_JOINT_NAMES: list[str] = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7",
    "openarm_right_finger_joint1",
]
LEFT_JOINT_NAMES: list[str] = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7",
    "openarm_left_finger_joint1",
]
_RIGHT_IDX = [JOINT_NAMES.index(n) for n in RIGHT_JOINT_NAMES]
_LEFT_IDX = [JOINT_NAMES.index(n) for n in LEFT_JOINT_NAMES]

CAM_TOPICS = {
    "head":        "/camera/head/image_raw",
    "left_wrist":  "/camera/left_wrist/image_raw",
    "right_wrist": "/camera/right_wrist/image_raw",
}


def policy_image_keys(policy) -> list[str]:
    return [k for k in policy.config.input_features
            if k.startswith("observation.images.")]


def build_processors(policy, model_path, device):
    pre, post = make_pre_post_processors(
        policy.config,
        model_path,
        preprocessor_overrides={"device_processor": {"device": str(device)}},
    )
    return pre, post


# --------------------------------------------------------------------------- #
#  Offline check (no ROS2, no robot)
# --------------------------------------------------------------------------- #
def run_check(args, policy, preprocess, postprocess, device) -> None:
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = args.dataset_root
    ds = LeRobotDataset(args.repo_id, root=str(Path(root).expanduser()) if root else None)
    print(f"Loaded {args.repo_id}: {ds.num_episodes} episodes / {ds.num_frames} frames\n")

    img_keys = policy_image_keys(policy)
    print(f"Policy observation: state + cameras {[k.split('.')[-1] for k in img_keys]}\n")

    feats = getattr(ds, "features", None) or getattr(getattr(ds, "meta", None), "features", {})
    action_names = (feats.get("action", {}) or {}).get("names")

    n = ds.num_frames
    idxs = sorted(set([0, n // 4, n // 2, 3 * n // 4, n - 1]))
    per_joint = []
    for i in idxs:
        s = ds[i]
        policy.reset()
        batch = {"observation.state": s["observation.state"].unsqueeze(0)}
        for k in img_keys:
            batch[k] = s[k].unsqueeze(0)
        with torch.no_grad():
            proc = preprocess(batch)
            pred = policy.select_action(proc)
            pred = postprocess(pred).squeeze(0).cpu().numpy()
        rec = s["action"].cpu().numpy()
        err = np.abs(pred - rec)
        per_joint.append(err)
        print(f"  frame {i:6d}  max|err|={err.max():.4f}  mean|err|={err.mean():.4f} rad")

    per_joint = np.stack(per_joint)                  # (n_frames, n_joints)
    mean_pj = per_joint.mean(axis=0)                 # per-joint mean (stable stat)
    worst_joint_mean = float(mean_pj.max())
    overall_mean = float(per_joint.mean())
    worst_frame_max = float(per_joint.max())         # informational only
    if action_names is None or len(action_names) != per_joint.shape[1]:
        action_names = [f"joint_{j}" for j in range(per_joint.shape[1])]

    print("\nPer-joint mean |err| (rad), worst first:")
    for j in np.argsort(mean_pj)[::-1]:
        print(f"  {action_names[j]:32s} {mean_pj[j]:.4f}")

    print(f"\nOverall mean |err|     : {overall_mean:.4f} rad")
    print(f"Worst per-joint mean   : {worst_joint_mean:.4f} rad   (verdict basis)")
    print(f"Worst single-frame max : {worst_frame_max:.4f} rad   (informational)")

    if worst_joint_mean < 0.05 and overall_mean < 0.03:
        print("\nOK: obs->action pipeline (incl. normalization) is correct. "
              "Errors decay gracefully (no uniform offset, no single bad joint). "
              "Safe to proceed to a careful hardware test.")
    else:
        print("\nINVESTIGATE: uniform large error across joints -> processors/stats; "
              "one joint far above the rest -> ordering/index mismatch. "
              "Do NOT run on hardware yet.")


# --------------------------------------------------------------------------- #
#  Live deployment node
# --------------------------------------------------------------------------- #
def run_live(args, policy, preprocess, postprocess, device) -> None:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image, JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration

    img_keys = policy_image_keys(policy)
    cams = [k.split(".")[-1] for k in img_keys]
    right_only = args.right_only

    # sanity: action dim must match the mode
    act_dim = policy.config.output_features["action"].shape[0]
    expected = len(RIGHT_JOINT_NAMES) if right_only else N_JOINTS
    if act_dim != expected:
        raise SystemExit(
            f"Policy action dim {act_dim} != expected {expected} for "
            f"{'right-only' if right_only else 'bimanual'} mode. "
            f"Did you mean to {'add' if act_dim == len(RIGHT_JOINT_NAMES) else 'drop'} "
            f"--right-only?")

    class ACTDeployer(Node):
        def __init__(self):
            super().__init__("act_deployer")
            self._state: np.ndarray | None = None       # always full 16
            self._images: dict[str, np.ndarray | None] = {c: None for c in cams}
            self._enabled = False
            self._left_hold: np.ndarray | None = None    # captured at enable()

            self.create_subscription(JointState, "/joint_states", self._cb_state, 10)
            for c in cams:
                self.create_subscription(
                    Image, CAM_TOPICS[c], lambda m, k=c: self._cb_img(m, k), 10)

            self._pub = self.create_publisher(JointTrajectory, "/joint_trajectory", 10)
            self.create_timer(1.0 / args.fps, self._tick)
            mode = "RIGHT-ONLY (left arm held)" if right_only else "BIMANUAL"
            self.get_logger().info(
                f"ACT deployer [{mode}]: {args.fps} Hz | horizon "
                f"{args.action_horizon}s | cameras {cams} (DISABLED until ENTER)")

        def _cb_state(self, msg):
            lut = {n: p for n, p in zip(msg.name, msg.position)}
            self._state = np.array(
                [lut.get(n, 0.0) for n in JOINT_NAMES], dtype=np.float32)

        def _cb_img(self, msg, key):
            self._images[key] = (
                np.frombuffer(msg.data, dtype=np.uint8)
                .reshape(msg.height, msg.width, 3).copy())

        def ready(self):
            return (self._state is not None and
                    all(v is not None for v in self._images.values()))

        def enable(self):
            policy.reset()
            if right_only:
                self._left_hold = self._state[_LEFT_IDX].copy()
                self.get_logger().info(
                    f"Holding LEFT arm at: {np.round(self._left_hold, 3).tolist()}")
            self._enabled = True

        def disable(self):
            self._enabled = False

        def _tick(self):
            if not self._enabled or not self.ready():
                return

            state_obs = self._state[_RIGHT_IDX] if right_only else self._state
            batch = {"observation.state":
                     torch.from_numpy(state_obs).float().unsqueeze(0)}
            for c in cams:
                t = torch.from_numpy(self._images[c]).permute(2, 0, 1).float().div(255.0)
                batch[f"observation.images.{c}"] = t.unsqueeze(0)

            with torch.no_grad():
                proc = preprocess(batch)
                action = policy.select_action(proc)
                action = postprocess(action).squeeze(0).cpu().numpy()

            if right_only:
                out = np.zeros(N_JOINTS, dtype=np.float32)
                out[_LEFT_IDX] = self._left_hold
                out[_RIGHT_IDX] = action
            else:
                out = action

            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in out]
            sec = int(args.action_horizon)
            nsec = int((args.action_horizon - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            traj = JointTrajectory()
            traj.joint_names = JOINT_NAMES
            traj.points = [pt]
            self._pub.publish(traj)

    rclpy.init()
    node = ACTDeployer()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()

    print("\n=== OpenArm ACT deployment ===")
    print("Waiting for /joint_states + cameras", end="", flush=True)
    while not node.ready():
        time.sleep(0.2)
        print(".", end="", flush=True)
    print(" ready!\n")

    print("SAFETY CHECKLIST:")
    print("  - Robot at (or near) the recording START pose")
    print("  - Hand on the e-stop / power")
    print("  - Cube roughly where it was during recording")
    print("  - VR grips NOT held (IK stays IDLE)")
    if right_only:
        print("  - LEFT arm will be FROZEN at its current pose; make sure it is "
              "somewhere safe/out of the way before you press ENTER")
    print("  - Consider lowering gains first (ros2 param set "
          "/controller_manager right_kp1 40.0, etc.)")
    try:
        input("\nPress ENTER to START policy control (Ctrl+C to abort)... ")
    except KeyboardInterrupt:
        print("\nAborted before start.")
        node.destroy_node(); rclpy.shutdown(); return

    node.enable()
    print("Policy ACTIVE. Ctrl+C to stop.\n")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\nStopping policy...")
    finally:
        node.disable()
        time.sleep(0.2)
        node.destroy_node()
        rclpy.shutdown()
        spin.join(timeout=2.0)
    print("Done.")


# --------------------------------------------------------------------------- #
def main() -> None:
    ap = argparse.ArgumentParser(description="Deploy/check an ACT policy on OpenArm")
    ap.add_argument("--policy", required=True,
                    help="HF repo id or local path of the trained ACT policy")
    ap.add_argument("--right-only", action="store_true",
                    help="Policy controls the RIGHT arm only (8-dim); left arm is "
                         "held at its pose when you press ENTER. Use with the "
                         "right-only checkpoint + dataset.")
    ap.add_argument("--check", action="store_true",
                    help="Offline sanity check against the local dataset (no robot)")
    ap.add_argument("--repo-id", default=None,
                    help="(--check) dataset repo id MATCHING the policy")
    ap.add_argument("--dataset-root", default=None, help="(--check) local dataset root")
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--action-horizon", type=float, default=0.10,
                    help="time_from_start per published point (s)")
    ap.add_argument("--n-action-steps", type=int, default=None,
                    help="Override chunk steps executed before re-inferring "
                         "(trained=100). Lower=more reactive.")
    ap.add_argument("--device", default="cuda")
    args = ap.parse_args()

    device = args.device if (torch.cuda.is_available() or args.device == "cpu") else "cpu"
    print(f"Loading policy {args.policy} on {device} ...")
    policy = ACTPolicy.from_pretrained(args.policy)
    if args.n_action_steps is not None:
        policy.config.n_action_steps = args.n_action_steps
        print(f"Overrode n_action_steps -> {args.n_action_steps}")
    policy.to(device)
    policy.eval()

    print("Building pre/post processors (normalization pipeline) ...")
    preprocess, postprocess = build_processors(policy, args.policy, device)
    print("Ready.\n")

    if args.check:
        if not args.repo_id:
            ap.error("--check requires --repo-id (and usually --dataset-root) "
                     "MATCHING the policy")
        run_check(args, policy, preprocess, postprocess, device)
    else:
        run_live(args, policy, preprocess, postprocess, device)


if __name__ == "__main__":
    main()
