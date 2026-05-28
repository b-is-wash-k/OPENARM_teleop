#!/usr/bin/env python3
"""
Run trained policy on bimanual OpenArm robot for N evaluation trials.
No dataset recording - just loads the policy and runs the robot.
"""

import time
import traceback
from pathlib import Path

import numpy as np
import torch
from safetensors.torch import load_file

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.cameras.realsense import RealSenseCameraConfig
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.robots import make_robot_from_config
from lerobot.robots.bi_openarm_follower import BiOpenArmFollower
from lerobot.robots.bi_openarm_follower.config_bi_openarm_follower import BiOpenArmFollowerConfig
from lerobot.robots.openarm_follower import OpenArmFollowerConfigBase


# ─── CONFIG ──────────────────────────────────────────────────────────────────
POLICY_PATH = Path.home() / "humanoids/openarm_module/outputs/train/openarm_open_lab_door_v1/checkpoints/100000/pretrained_model"
CAMERA_SERIAL  = "025222071898"
WRIST_CAM_PATH = "/dev/video-wrist-right"
NUM_TRIALS     = 10
EPISODE_LENGTH = 40   # seconds
RESET_TIME     = 5    # seconds between trials
DEVICE         = "cuda"
FPS            = 30
# ─────────────────────────────────────────────────────────────────────────────

STATE_KEYS = [
    'left_joint_1.pos',  'left_joint_1.vel',  'left_joint_1.torque',
    'left_joint_2.pos',  'left_joint_2.vel',  'left_joint_2.torque',
    'left_joint_3.pos',  'left_joint_3.vel',  'left_joint_3.torque',
    'left_joint_4.pos',  'left_joint_4.vel',  'left_joint_4.torque',
    'left_joint_5.pos',  'left_joint_5.vel',  'left_joint_5.torque',
    'left_joint_6.pos',  'left_joint_6.vel',  'left_joint_6.torque',
    'left_joint_7.pos',  'left_joint_7.vel',  'left_joint_7.torque',
    'left_gripper.pos',  'left_gripper.vel',  'left_gripper.torque',
    'right_joint_1.pos', 'right_joint_1.vel', 'right_joint_1.torque',
    'right_joint_2.pos', 'right_joint_2.vel', 'right_joint_2.torque',
    'right_joint_3.pos', 'right_joint_3.vel', 'right_joint_3.torque',
    'right_joint_4.pos', 'right_joint_4.vel', 'right_joint_4.torque',
    'right_joint_5.pos', 'right_joint_5.vel', 'right_joint_5.torque',
    'right_joint_6.pos', 'right_joint_6.vel', 'right_joint_6.torque',
    'right_joint_7.pos', 'right_joint_7.vel', 'right_joint_7.torque',
    'right_gripper.pos', 'right_gripper.vel', 'right_gripper.torque',
]


def load_policy(policy_path, device):
    policy = ACTPolicy.from_pretrained(policy_path)
    policy.to(device)
    policy.eval()
    return policy


def load_normalizers(policy_path, device):
    policy_path = Path(policy_path)
    pre_files  = list(policy_path.glob("*normalizer_processor.safetensors"))
    post_files = list(policy_path.glob("*unnormalizer_processor.safetensors"))

    if not pre_files or not post_files:
        print("Warning: normalizer files not found, running without normalization")
        return None, None

    pre_weights  = load_file(pre_files[0], device=device)
    post_weights = load_file(post_files[0], device=device)
    return pre_weights, post_weights


def normalize_obs(batch, pre_weights, device):
    if pre_weights is None:
        return batch

    normalized = {}
    for key, value in batch.items():
        mean_key = f"{key}.mean"
        std_key  = f"{key}.std"
        if mean_key in pre_weights and std_key in pre_weights:
            mean = pre_weights[mean_key].to(device)
            std  = pre_weights[std_key].to(device)
            while mean.dim() < value.dim():
                mean = mean.unsqueeze(0)
                std  = std.unsqueeze(0)
            normalized[key] = (value - mean) / (std + 1e-8)
        else:
            normalized[key] = value
    return normalized


def denormalize_action(action, post_weights, device):
    if post_weights is None:
        return action

    mean_key = "action.mean"
    std_key  = "action.std"
    if mean_key in post_weights and std_key in post_weights:
        mean = post_weights[mean_key].to(device)
        std  = post_weights[std_key].to(device)
        while mean.dim() < action.dim():
            mean = mean.unsqueeze(0)
            std  = std.unsqueeze(0)
        return action * std + mean
    return action


def make_robot_cfg():
    return BiOpenArmFollowerConfig(
        left_arm_config=OpenArmFollowerConfigBase(
            port='can1',
            side='left',
            cameras={
                'chest': RealSenseCameraConfig(
                    serial_number_or_name=CAMERA_SERIAL,
                    fps=FPS,
                    width=848,
                    height=480,
                ),
                'wrist_left': OpenCVCameraConfig(
                    index_or_path='/dev/video-wrist-left',
                    fps=FPS,
                    width=640,
                    height=480,
                )
            }
        ),
        right_arm_config=OpenArmFollowerConfigBase(
            port='can0',
            side='right',
            cameras={
                'wrist_right': OpenCVCameraConfig(
                    index_or_path='/dev/video-wrist-right',
                    fps=FPS,
                    width=640,
                    height=480,
                )
            }
        ),
    )


def run_policy_episode(robot, policy, episode_length, device, fps, pre_weights, post_weights):
    dt = 1.0 / fps
    start_time = time.perf_counter()

    try:
        policy.reset()

        while True:
            loop_start = time.perf_counter()

            if time.perf_counter() - start_time >= episode_length:
                break

            obs = robot.get_observation()

            state = np.array([obs[k] for k in STATE_KEYS], dtype=np.float32)
            chest_image      = obs['left_chest'].astype(np.float32) / 255.0
            wrist_left_image = obs['left_wrist_left'].astype(np.float32) / 255.0
            wrist_right_image = obs['right_wrist_right'].astype(np.float32) / 255.0

            batch = {
                'observation.state': torch.from_numpy(state).float().unsqueeze(0).to(device),
                'observation.images.left_chest': torch.from_numpy(chest_image).permute(2, 0, 1).unsqueeze(0).to(device),
                'observation.images.left_wrist_left': torch.from_numpy(wrist_left_image).permute(2, 0, 1).unsqueeze(0).to(device),
                'observation.images.right_wrist_right': torch.from_numpy(wrist_right_image).permute(2, 0, 1).unsqueeze(0).to(device),
            }

            batch = normalize_obs(batch, pre_weights, device)

            with torch.inference_mode():
                action = policy.select_action(batch)

            action = denormalize_action(action, post_weights, device)

            action_np = action.squeeze().cpu().numpy()
            action_dict = {k: float(v) for k, v in zip(STATE_KEYS, action_np)}
            robot.send_action(action_dict)

            sleep_time = dt - (time.perf_counter() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

        return True

    except Exception as e:
        print(f"Error during episode: {e}")
        traceback.print_exc()
        return False


def main():
    print("=" * 70)
    print("OpenArm Bimanual Policy Deployment")
    print("=" * 70)
    print(f"Policy:  {POLICY_PATH}")
    print(f"Trials:  {NUM_TRIALS}")
    print(f"Length:  {EPISODE_LENGTH}s per trial")
    print(f"Reset:   {RESET_TIME}s between trials")
    print("=" * 70)

    print("\nLoading policy...")
    policy = load_policy(POLICY_PATH, DEVICE)
    print("Policy loaded")

    print("Loading normalizers...")
    pre_weights, post_weights = load_normalizers(POLICY_PATH, DEVICE)

    robot_cfg = make_robot_cfg()
    results = []

    try:
        for trial in range(1, NUM_TRIALS + 1):
            print(f"\n{'='*70}")
            print(f"Trial {trial}/{NUM_TRIALS}")
            print(f"{'='*70}")

            robot = make_robot_from_config(robot_cfg)
            robot.connect()

            try:
                print(f"Running for {EPISODE_LENGTH}s...")
                completed = run_policy_episode(
                    robot, policy, EPISODE_LENGTH, DEVICE, FPS, pre_weights, post_weights
                )
            finally:
                robot.disconnect()

            if not completed:
                print("Trial failed")
                results.append(False)
                continue

            while True:
                response = input("\nSuccessful? (y/n): ").strip().lower()
                if response in ['y', 'yes']:
                    results.append(True)
                    print("SUCCESS")
                    break
                elif response in ['n', 'no']:
                    results.append(False)
                    print("FAILURE")
                    break

            successes = sum(results)
            total = len(results)
            print(f"\nCurrent: {successes}/{total} ({100*successes/total:.1f}%)")

            if trial < NUM_TRIALS and RESET_TIME > 0:
                print(f"\nResetting - {RESET_TIME}s")
                time.sleep(RESET_TIME)

    except KeyboardInterrupt:
        print("\n\nStopped")

    print("\n" + "=" * 70)
    print("RESULTS")
    print("=" * 70)

    if results:
        successes = sum(results)
        total = len(results)
        print(f"\nTotal:   {total}")
        print(f"Success: {successes}")
        print(f"Failed:  {total - successes}")
        print(f"Rate:    {100*successes/total:.1f}%")
        print("\nDetails:")
        for i, result in enumerate(results, 1):
            print(f"  Trial {i}: {'SUCCESS' if result else 'FAILURE'}")

    print("\n" + "=" * 70)


if __name__ == "__main__":
    main()