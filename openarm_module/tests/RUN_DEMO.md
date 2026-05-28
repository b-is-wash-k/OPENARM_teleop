# Running OpenArm Demos

This guide shows you how to run the included demonstration examples on your OpenArm bimanual robot.

## Prerequisites

- OpenArm module installed and configured
- Robot powered on and connected via CAN bus
- CAN interfaces configured (should auto-configure via udev rules)

## Safety First: Move to Zero Position

**IMPORTANT:** Always start by moving the arms to their zero position before running any demo.

```bash
cd ~/humanoids/openarm_module
python scripts/move_to_zero.py
```

This ensures the robot starts from a safe, known configuration.

---

## Demo 1: Wave Demo (Scripted Motion)

A simple scripted demonstration where both arms perform a wave motion.

```bash
cd ~/humanoids/openarm_module
python tests/openarm_wave_demo.py
```

**What it does:** Moves right arm in a synchronized waving pattern.

---

## Demo 2: Replay Recorded Demonstrations

The following demos replay teleoperated demonstrations that were previously recorded.

### RPL Demo 1

```bash
lerobot-replay \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --dataset.fps=60 \
    --dataset.repo_id=$HOME/humanoids/openarm_module/tests/rpl_demo \
    --dataset.episode=0
```

### RPL Demo 2

```bash
lerobot-replay \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --dataset.fps=60 \
    --dataset.repo_id=$HOME/humanoids/openarm_module/tests/rpl_demo2 \
    --dataset.episode=0
```

### RPL Demo 3

```bash
lerobot-replay \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --dataset.fps=60 \
    --dataset.repo_id=$HOME/humanoids/openarm_module/tests/rpl_demo3 \
    --dataset.episode=0
```

---

## Safety Notes

- Always ensure the workspace is clear before running demos
- Keep the emergency stop within reach
- Start with `move_to_zero.py` before each demo session
- If the robot behaves unexpectedly, press Ctrl+C to stop execution

---

## Start Gamepad Teleoperation without recording

```bash
lerobot-teleoperate \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --teleop.type=openarm_bi_gamepad_joints \
    --teleop.joint_velocity_scale=60.0
```
## Recording Your Own Demos

---

To record your own demonstrations:

```bash
lerobot-record \
    --robot.type=bi_openarm_follower \
    --robot.left_arm_config.port=can1 \
    --robot.left_arm_config.side=left \
    --robot.right_arm_config.port=can0 \
    --robot.right_arm_config.side=right \
    --teleop.type=openarm_bi_gamepad_joints \
    --teleop.joint_velocity_scale=60.0 \
    --dataset.repo_id=$HOME/humanoids/openarm_module/examples/my_demo \
    --dataset.single_task="Your task description" \
    --dataset.fps=60 \
    --dataset.num_episodes=1 \
    --dataset.episode_time_s=30 \
    --dataset.reset_time_s=10 \
    --dataset.push_to_hub=false
```

See the main [README.md](README.md) for full teleoperation and recording instructions.