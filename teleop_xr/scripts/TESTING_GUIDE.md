# OpenArm ACT Policy Testing Guide

How to run each phase of the pipeline, which terminal to use for what, and how to
evaluate a trained policy without starting the full XR teleop demo.

---

## Terminal layout overview

```
┌──────────────┬──────────────────────────────────────────┐
│ Terminal 1   │  ROS2 hardware bringup (controllers)     │
│ Terminal 2   │  joint_trajectory_relay.py               │
│ Terminal 3   │  camera_publisher.py (standalone)         │
│ Terminal 4   │  policy evaluation / lerobot inference   │
│ Terminal 5   │  (optional) lerobot_recorder.py          │
└──────────────┴──────────────────────────────────────────┘
```

For **teleop recording** (adding data): replace Terminal 3 with `demo_with_ros2.py`
(it publishes cameras internally) and skip Terminal 4.

---

## Step 0 — Find your camera device IDs (one-time)

```bash
v4l2-ctl --list-devices
# or
ls -la /dev/video*
```

Current known mapping (may change after reboot — re-check if cameras fail):

| Camera | Topic published | Device |
|---|---|---|
| Head / chest | `/camera/head/image_raw` | `/dev/video59` |
| Left wrist | `/camera/left_wrist/image_raw` | `/dev/video65` |
| Right wrist | `/camera/right_wrist/image_raw` | `/dev/video57` |

---

## Terminal 1 — ROS2 hardware bringup

```bash
source /opt/ros/jazzy/setup.bash
source ~/OPEN_ARM/packages/install/setup.bash

ros2 launch openarm_bringup openarm.bimanual.launch.py
```

Wait for `[INFO] Controller Manager is running` before proceeding.

---

## Terminal 2 — Joint trajectory relay

```bash
source /opt/ros/jazzy/setup.bash
source ~/OPEN_ARM/packages/install/setup.bash

python3 ~/OPEN_ARM/teleop_xr/scripts/joint_trajectory_relay.py
```

Expected output:
```
✅ Relay READY:
   ARM TRAJECTORIES: /joint_trajectory → left + right controllers (7 joints each)
   GRIPPERS: stripped — controlled directly by demo_with_ros2.py
```

---

## Terminal 3A — Cameras only (for evaluation — no Quest needed)

Standalone script — does **not** touch `demo_with_ros2.py`, no aiortc/WebRTC dependency,
no IK, no TUI. Just opens the cameras with OpenCV and publishes ROS2 Image topics.

```bash
source /opt/ros/jazzy/setup.bash
source ~/OPEN_ARM/packages/install/setup.bash

python3 ~/OPEN_ARM/teleop_xr/scripts/camera_publisher.py \
    --head        /dev/video53 \
    --left-wrist  /dev/video65 \
    --right-wrist /dev/video59
```

Right-arm-only policy (omit `--left-wrist`):
```bash
python3 ~/OPEN_ARM/teleop_xr/scripts/camera_publisher.py \
    --head        /dev/video53 \
    --right-wrist /dev/video59
```

Verify:
```bash
ros2 topic hz /camera/head/image_raw
ros2 topic hz /camera/right_wrist/image_raw
```

---

## Terminal 3B — Full demo (for teleop recording only)

Only needed when collecting new data via XR teleop. Quest must be connected.

```bash
source /opt/ros/jazzy/setup.bash
source ~/OPEN_ARM/packages/install/setup.bash
cd ~/OPEN_ARM_NEW

python -m teleop_xr.demo.demo_with_ros2 \
    --mode ik \
    --robot-class openarm \
    --head-device        /dev/video53 \
    --wrist-left-device  /dev/video65 \
    --wrist-right-device /dev/video59
```

---

## Terminal 4 — Policy evaluation

First download the trained model from HuggingFace:

```bash
# Check available trained models:
#   https://huggingface.co/20-wasa/act_openarm_cube_pickup_20260528
#   https://huggingface.co/20-wasa/act_openarm_cube_pickup_right_20260528
```

### Option A — Use deploy_openarm_act.py (if it exists)

```bash
source ~/lerobot/.venv/bin/activate

python3 ~/OPEN_ARM/teleop_xr/scripts/deploy_openarm_act.py \
    --policy-repo-id 20-wasa/act_openarm_cube_pickup_20260528
```

### Option B — lerobot-eval (no real robot, sim only)

```bash
source ~/lerobot/.venv/bin/activate
cd ~/lerobot

lerobot-eval \
    --policy.repo_id=20-wasa/act_openarm_cube_pickup_20260528 \
    --policy.device=cuda
```

### Option C — Manual inference loop (real robot)

```bash
source ~/lerobot/.venv/bin/activate
cd ~/lerobot

python3 - <<'EOF'
from lerobot.datasets.lerobot_dataset import LeRobotDataset
# TODO: wire up real robot inference here
# See: lerobot/examples/ for policy inference examples
EOF
```

---

## Terminal 5 — Recording new episodes (optional)

```bash
source ~/lerobot/.venv/bin/activate

cd ~/OPEN_ARM/teleop_xr

python scripts/lerobot_recorder.py \
    --repo-id 20-wasa/openarm-cube-pickup-20260528 \
    --repo-id-right 20-wasa/openarm-cube-pickup-right-20260528 \
    --resume \
    --fps 30 \
    --root ~/lerobot_datasets \
    --push-to-hub
```

Episode controls at the prompt:
- **ENTER** = reuse last task description
- **s / ENTER** = save episode (success)
- **f** = save as failure (kept but labelled `FAILURE: ...`)
- **d** = discard (bad demo, thrown away)
- **r** at reset prompt = send robot to start pose
- type `quit` to finish and push

---

## Training (workstation — RTX 6000 Blackwell)

```bash
# 1. Download datasets (bypasses xet)
python ~/OPEN_ARM/teleop_xr/scripts/download_datasets.py

# 2. Train bimanual then right-only
conda activate teleop_xr
cd ~/Biswash/lerobot

lerobot-train \
    --dataset.repo_id=20-wasa/openarm-cube-pickup-20260528 \
    --dataset.root=~/Biswash/lerobot_datasets/20-wasa/openarm-cube-pickup-20260528 \
    --policy.type=act \
    --policy.device=cuda \
    --batch_size=32 \
    --steps=50000 \
    --save_freq=5000 \
    --job_name=act_openarm_cube_pickup_20260528 \
    --output_dir=outputs/train/act_openarm_cube_pickup_20260528 \
    --policy.push_to_hub=true \
    --policy.repo_id=20-wasa/act_openarm_cube_pickup_20260528 && \
lerobot-train \
    --dataset.repo_id=20-wasa/openarm-cube-pickup-right-20260528 \
    --dataset.root=~/Biswash/lerobot_datasets/20-wasa/openarm-cube-pickup-right-20260528 \
    --policy.type=act \
    --policy.device=cuda \
    --batch_size=32 \
    --steps=50000 \
    --save_freq=5000 \
    --job_name=act_openarm_cube_pickup_right_20260528 \
    --output_dir=outputs/train/act_openarm_cube_pickup_right_20260528 \
    --policy.push_to_hub=true \
    --policy.repo_id=20-wasa/act_openarm_cube_pickup_right_20260528
```

## Training (laptop — RTX 4050 6 GB, slower)

```bash
source ~/lerobot/.venv/bin/activate
cd ~/lerobot

lerobot-train \
    --dataset.repo_id=20-wasa/openarm-cube-pickup-20260528 \
    --dataset.root=~/lerobot_datasets/20-wasa/openarm-cube-pickup-20260528 \
    --policy.type=act \
    --policy.device=cuda \
    --batch_size=4 \
    --steps=50000 \
    --save_freq=5000 \
    --job_name=act_openarm_cube_pickup_20260528 \
    --output_dir=outputs/train/act_openarm_cube_pickup_20260528 \
    --policy.push_to_hub=true \
    --policy.repo_id=20-wasa/act_openarm_cube_pickup_20260528 && \
lerobot-train \
    --dataset.repo_id=20-wasa/openarm-cube-pickup-right-20260528 \
    --dataset.root=~/lerobot_datasets/20-wasa/openarm-cube-pickup-right-20260528 \
    --policy.type=act \
    --policy.device=cuda \
    --batch_size=4 \
    --steps=50000 \
    --save_freq=5000 \
    --job_name=act_openarm_cube_pickup_right_20260528 \
    --output_dir=outputs/train/act_openarm_cube_pickup_right_20260528 \
    --policy.push_to_hub=true \
    --policy.repo_id=20-wasa/act_openarm_cube_pickup_right_20260528
```

---

## Quick checklist before every session

```bash
# All from a spare terminal:
ros2 topic hz /joint_states                     # hardware alive
ros2 topic hz /camera/head/image_raw            # head cam publishing
ros2 topic hz /camera/right_wrist/image_raw     # right wrist cam
ros2 topic list | grep gripper                  # gripper action servers up
```

---

## Files

| File | Purpose |
|---|---|
| `scripts/camera_publisher.py` | Standalone camera publisher — `--head`, `--left-wrist`, `--right-wrist` device paths. No aiortc/WebRTC needed. |
| `scripts/joint_trajectory_relay.py` | Splits `/joint_trajectory` to per-arm controllers |
| `scripts/lerobot_recorder.py` | Records episodes with save/discard/failure choice |
| `scripts/download_datasets.py` | Downloads HF datasets via direct HTTP (bypasses xet) |
| `scripts/RECORDING_GUIDE.md` | Full recording + xet error troubleshooting guide |
| `scripts/TESTING_GUIDE.md` | This file |
