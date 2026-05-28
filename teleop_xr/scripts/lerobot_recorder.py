#!/usr/bin/env python3
"""
LeRobot dataset recorder for bimanual OpenArm VR teleop.

Records at --fps Hz:
  observation.state            16 joint positions from /joint_states
  action                       16 joint commands  from /joint_trajectory
  observation.images.head         480x640 rgb8 from /camera/head/image_raw
  observation.images.left_wrist   480x640 rgb8
  observation.images.right_wrist  480x640 rgb8

Optionally records a parallel right-arm-only dataset at the same time
(--repo-id-right): 8 joints + head + right_wrist only.

Usage:
    source ~/lerobot/.venv/bin/activate
    source /opt/ros/jazzy/setup.bash
    source ~/OPEN_ARM/packages/install/setup.bash
    cd ~/OPEN_ARM/teleop_xr

    python scripts/lerobot_recorder.py \\
        --repo-id YOUR_HF_USERNAME/openarm-bimanual \\
        --repo-id-right YOUR_HF_USERNAME/openarm-right-only \\
        --fps 30 \\
        --root ~/lerobot_datasets \\
        --push-to-hub

Requirements:
  - Teleop stack running (openarm.bimanual.launch.py + joint_trajectory_relay.py)
  - demo_with_ros2.py running with Quest connected (for camera topics to be active)
  - HF auth: run 'hf auth login' first if not already done
"""

from __future__ import annotations

import argparse
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory

from lerobot.datasets.lerobot_dataset import LeRobotDataset


# Joint names in the order they appear in /joint_states and /joint_trajectory.
# Must match the relay's split order (left 1-7, right 1-7, left finger, right finger).
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

# Right-arm-only subset (8 joints: joints 1-7 + right finger)
RIGHT_JOINT_NAMES: list[str] = [
    "openarm_right_joint1", "openarm_right_joint2", "openarm_right_joint3",
    "openarm_right_joint4", "openarm_right_joint5", "openarm_right_joint6",
    "openarm_right_joint7",
    "openarm_right_finger_joint1",
]
N_RIGHT_JOINTS = len(RIGHT_JOINT_NAMES)
_RIGHT_IDX = [JOINT_NAMES.index(n) for n in RIGHT_JOINT_NAMES]

# Left-arm-only subset (8 joints: joints 1-7 + left finger)
LEFT_JOINT_NAMES: list[str] = [
    "openarm_left_joint1", "openarm_left_joint2", "openarm_left_joint3",
    "openarm_left_joint4", "openarm_left_joint5", "openarm_left_joint6",
    "openarm_left_joint7",
    "openarm_left_finger_joint1",
]
N_LEFT_JOINTS = len(LEFT_JOINT_NAMES)
_LEFT_IDX = [JOINT_NAMES.index(n) for n in LEFT_JOINT_NAMES]

IMG_H, IMG_W = 480, 640   # all three cameras deliver 480x640 (V4L2 fallback)


# ---------------------------------------------------------------------------

class OpenArmRecorder(Node):
    def __init__(
        self,
        dataset: LeRobotDataset,
        fps: int,
        dataset_right: LeRobotDataset | None = None,
        dataset_left: LeRobotDataset | None = None,
    ):
        super().__init__("lerobot_recorder")
        self._dataset = dataset
        self._dataset_right = dataset_right
        self._dataset_left = dataset_left
        self._fps = fps

        # Latest sensor data (written by subscriber callbacks, read by timer)
        self._state: np.ndarray | None = None
        self._action: np.ndarray | None = None
        self._images: dict[str, np.ndarray | None] = {
            "head": None, "left_wrist": None, "right_wrist": None}

        # Episode control
        self._recording = False
        self._task = ""
        self._frame_count = 0
        self._episode_count = 0
        self._episode_lock = threading.Lock()
        # Serializes add_frame vs save_episode — save_episode mutates
        # episode_buffer["episode_index"] from int→array (line 242 in
        # dataset_writer.py); a concurrent add_frame then formats that
        # array as "%06d" and crashes.
        self._add_frame_lock = threading.Lock()

        # Subscribers
        self.create_subscription(
            JointState, "/joint_states", self._cb_state, 10)
        self.create_subscription(
            JointTrajectory, "/joint_trajectory", self._cb_action, 10)
        for key, topic in [
            ("head",        "/camera/head/image_raw"),
            ("left_wrist",  "/camera/left_wrist/image_raw"),
            ("right_wrist", "/camera/right_wrist/image_raw"),
        ]:
            self.create_subscription(
                Image, topic,
                lambda msg, k=key: self._cb_image(msg, k), 10)

        self._reset_pub = self.create_publisher(Bool, '/teleop_xr/reset', 1)

        self.create_timer(1.0 / fps, self._tick)
        extras = ([" + right-only"] if dataset_right else []) + ([" + left-only"] if dataset_left else [])
        self.get_logger().info(
            f"Recorder ready: {fps} fps | {N_JOINTS} joints | 3 cameras{''.join(extras)}")

    def send_reset(self) -> None:
        """Tell demo_with_ros2 to move both arms to their default start pose."""
        self._reset_pub.publish(Bool(data=True))

    # ── ROS2 callbacks (spin thread) ─────────────────────────────────────────

    def _cb_state(self, msg: JointState) -> None:
        lookup = {n: p for n, p in zip(msg.name, msg.position)}
        self._state = np.array(
            [lookup.get(n, 0.0) for n in JOINT_NAMES], dtype=np.float32)

    def _cb_action(self, msg: JointTrajectory) -> None:
        if not msg.points:
            return
        lookup = {n: p for n, p in zip(msg.joint_names, msg.points[0].positions)}
        self._action = np.array(
            [lookup.get(n, 0.0) for n in JOINT_NAMES], dtype=np.float32)

    def _cb_image(self, msg: Image, key: str) -> None:
        # Direct numpy conversion — avoids cv_bridge numpy ABI issue
        self._images[key] = (
            np.frombuffer(msg.data, dtype=np.uint8)
            .reshape(msg.height, msg.width, 3)
            .copy()
        )

    # ── Timer (spin thread) ──────────────────────────────────────────────────

    def _tick(self) -> None:
        if not self._recording:
            return
        with self._episode_lock:
            if not self._recording:
                return
            if (self._state is None or self._action is None or
                    any(v is None for v in self._images.values())):
                return
            # Snapshot all sensor data once under the lock
            state   = self._state.copy()
            action  = self._action.copy()
            head    = self._images["head"].copy()
            l_wrist = self._images["left_wrist"].copy()
            r_wrist = self._images["right_wrist"].copy()
            task    = self._task

        frame = {
            "observation.state":              state,
            "action":                         action,
            "observation.images.head":        head,
            "observation.images.left_wrist":  l_wrist,
            "observation.images.right_wrist": r_wrist,
            "task":                           task,
        }
        # Arm-only frames reuse already-copied image arrays;
        # fancy indexing on state/action always returns a new copy.
        frame_right = {
            "observation.state":              state[_RIGHT_IDX],
            "action":                         action[_RIGHT_IDX],
            "observation.images.head":        head,
            "observation.images.right_wrist": r_wrist,
            "task":                           task,
        } if self._dataset_right is not None else None

        frame_left = {
            "observation.state":             state[_LEFT_IDX],
            "action":                        action[_LEFT_IDX],
            "observation.images.head":       head,
            "observation.images.left_wrist": l_wrist,
            "task":                          task,
        } if self._dataset_left is not None else None

        with self._add_frame_lock:
            self._dataset.add_frame(frame)
            if frame_right is not None:
                self._dataset_right.add_frame(frame_right)
            if frame_left is not None:
                self._dataset_left.add_frame(frame_left)
        self._frame_count += 1

    # ── Episode control (main thread) ────────────────────────────────────────

    def start_episode(self, task: str) -> None:
        self._task = task
        self._frame_count = 0
        self._recording = True
        self.get_logger().info(
            f"▶ Recording episode {self._episode_count + 1}: '{task}'")

    def stop_episode(self) -> int:
        """Stop recording (do not save yet). Returns frame count."""
        with self._episode_lock:
            self._recording = False
            n = self._frame_count

        # Wait for any add_frame already past the recording check to finish
        # before save_episode mutates episode_buffer["episode_index"] int→array.
        with self._add_frame_lock:
            pass

        if n == 0:
            self.get_logger().warning("No frames recorded — episode discarded.")
            self._discard_buffers()
        return n

    def save_episode(self) -> None:
        """Flush the current episode buffer to disk (call after stop_episode)."""
        self._dataset.save_episode(parallel_encoding=True)
        if self._dataset_right is not None:
            self._dataset_right.save_episode(parallel_encoding=True)
        if self._dataset_left is not None:
            self._dataset_left.save_episode(parallel_encoding=True)
        self._episode_count += 1
        self.get_logger().info(
            f"✅ Saved episode {self._episode_count}: "
            f"{self._frame_count} frames ({self._frame_count / self._fps:.1f}s) | task: '{self._task}'"
        )

    def discard_episode(self) -> None:
        """Throw away the current episode buffer without saving."""
        self._discard_buffers()
        self.get_logger().info("🗑  Episode discarded.")

    def _discard_buffers(self) -> None:
        self._dataset.clear_episode_buffer()
        if self._dataset_right is not None:
            self._dataset_right.clear_episode_buffer()
        if self._dataset_left is not None:
            self._dataset_left.clear_episode_buffer()
        self._frame_count = 0

    def data_ready(self) -> bool:
        return (self._state is not None and
                self._action is not None and
                all(v is not None for v in self._images.values()))


# ---------------------------------------------------------------------------

def _reupload_parquets(dataset: LeRobotDataset, repo_id: str) -> None:
    """Re-upload parquet files directly to fix xet pointer corruption from push_to_hub.

    Covers data/*.parquet AND meta/episodes/*.parquet so the lerobot Space
    visualizer can read episode metadata (it fetches meta/episodes/chunk-*/file-*.parquet
    directly and silently fails if it gets an xet pointer instead of real bytes).
    """
    from huggingface_hub import HfApi
    api = HfApi()
    root = Path(dataset.root)
    parquet_paths = sorted(
        list((root / "data").rglob("*.parquet")) +
        list((root / "meta" / "episodes").rglob("*.parquet")) +
        ([root / "meta" / "tasks.parquet"] if (root / "meta" / "tasks.parquet").exists() else [])
    )
    import pyarrow.parquet as pq, pyarrow as pa, tempfile as _tf

    for parquet_path in parquet_paths:
        repo_path = str(parquet_path.relative_to(root))
        # Re-serialise through pyarrow with snappy compression so the bytes
        # differ from the xet-stored copy — this forces HF to accept a new
        # commit instead of skipping with "No files modified".
        table = pq.read_table(str(parquet_path))
        buf = __import__("io").BytesIO()
        pq.write_table(table, buf, compression="snappy")
        with _tf.NamedTemporaryFile(suffix=".parquet", delete=False) as tmp:
            tmp.write(buf.getvalue())
            tmp.flush()
            api.upload_file(
                path_or_fileobj=tmp.name,
                path_in_repo=repo_path,
                repo_id=repo_id,
                repo_type="dataset",
                commit_message=f"Re-upload {repo_path} (real parquet bytes, bypass xet)",
            )

    # Push flat meta/episodes.parquet for lerobot Space visualizer compatibility.
    # Read from dataset.meta.episodes (in memory) — NOT from disk, because
    # push_to_hub() writes an xet pointer blob over the local parquet file.
    import tempfile, pandas as pd
    if dataset.meta.episodes is not None:
        ep = dataset.meta.episodes.to_pandas()
        flat = pd.DataFrame({
            "episode_index": ep["episode_index"].astype("int64"),
            "tasks":         ep["tasks"],
            "length":        ep["length"].astype("int64"),
        })
        with tempfile.NamedTemporaryFile(suffix=".parquet", delete=False) as tmp:
            flat.to_parquet(tmp.name, index=False)
            api.upload_file(
                path_or_fileobj=tmp.name,
                path_in_repo="meta/episodes.parquet",
                repo_id=repo_id,
                repo_type="dataset",
                commit_message="Update flat meta/episodes.parquet (lerobot Space visualizer)",
            )


def _push_readme(dataset: LeRobotDataset, repo_id: str) -> None:
    """Push a lerobot-style README.md so the HF Dataset Viewer works."""
    import json
    from huggingface_hub import HfApi
    info_path = Path(dataset.root) / "meta" / "info.json"
    with open(info_path) as f:
        info = json.load(f)
    readme = (
        "---\n"
        "license: apache-2.0\n"
        "task_categories:\n"
        "- robotics\n"
        "tags:\n"
        "- LeRobot\n"
        "configs:\n"
        "- config_name: default\n"
        "  data_files: data/*/*.parquet\n"
        "---\n\n"
        "This dataset was created using [LeRobot](https://github.com/huggingface/lerobot).\n\n\n"
        f'<a class="flex" href="https://huggingface.co/spaces/lerobot/visualize_dataset?path={repo_id}">\n'
        '<img class="block dark:hidden" src="https://huggingface.co/datasets/huggingface/badges/resolve/main/visualize-this-dataset-xl.svg"/>\n'
        '<img class="hidden dark:block" src="https://huggingface.co/datasets/huggingface/badges/resolve/main/visualize-this-dataset-xl-dark.svg"/>\n'
        "</a>\n\n\n"
        "## Dataset Structure\n\n"
        "[meta/info.json](meta/info.json):\n"
        "```json\n"
        + json.dumps(info, indent=4)
        + "\n```\n"
    )
    HfApi().upload_file(
        path_or_fileobj=readme.encode(),
        path_in_repo="README.md",
        repo_id=repo_id,
        repo_type="dataset",
        commit_message="Add dataset card for Dataset Viewer",
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Record LeRobot episodes from bimanual OpenArm VR teleop")
    parser.add_argument(
        "--repo-id", required=True,
        help="HuggingFace repo id for the full bimanual dataset")
    parser.add_argument(
        "--repo-id-right", default=None,
        help="(Optional) repo id for a parallel right-arm-only dataset "
             "(8 joints + head + right_wrist). Recorded simultaneously.")
    parser.add_argument(
        "--repo-id-left", default=None,
        help="(Optional) repo id for a parallel left-arm-only dataset "
             "(8 joints + head + left_wrist). Recorded simultaneously.")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument(
        "--root", default=None,
        help="Base directory for datasets (default: ~/lerobot_datasets). "
             "The dataset is stored at <root>/<repo_id>/.")
    parser.add_argument(
        "--task", default="pick up cube and place in box",
        help="Default task description (press ENTER each episode to reuse)")
    parser.add_argument(
        "--push-to-hub", action="store_true", default=False,
        help="Push to HuggingFace Hub automatically after quitting")
    parser.add_argument(
        "--resume", action="store_true", default=False,
        help="Resume recording into existing local datasets instead of creating new ones")
    args = parser.parse_args()

    # lerobot uses root as the exact dataset directory (not a parent).
    # We construct root = base / repo_id so each dataset gets its own dir.
    base = Path(args.root) if args.root else Path.home() / "lerobot_datasets"
    root = base / args.repo_id
    root.parent.mkdir(parents=True, exist_ok=True)

    features = {
        "observation.state": {
            "dtype": "float32",
            "shape": (N_JOINTS,),
            "names": JOINT_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (N_JOINTS,),
            "names": JOINT_NAMES,
        },
        "observation.images.head": {
            "dtype": "video",
            "shape": (IMG_H, IMG_W, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.left_wrist": {
            "dtype": "video",
            "shape": (IMG_H, IMG_W, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.right_wrist": {
            "dtype": "video",
            "shape": (IMG_H, IMG_W, 3),
            "names": ["height", "width", "channel"],
        },
    }

    features_right = {
        "observation.state": {
            "dtype": "float32",
            "shape": (N_RIGHT_JOINTS,),
            "names": RIGHT_JOINT_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (N_RIGHT_JOINTS,),
            "names": RIGHT_JOINT_NAMES,
        },
        "observation.images.head": {
            "dtype": "video",
            "shape": (IMG_H, IMG_W, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.right_wrist": {
            "dtype": "video",
            "shape": (IMG_H, IMG_W, 3),
            "names": ["height", "width", "channel"],
        },
    }

    features_left = {
        "observation.state": {
            "dtype": "float32",
            "shape": (N_LEFT_JOINTS,),
            "names": LEFT_JOINT_NAMES,
        },
        "action": {
            "dtype": "float32",
            "shape": (N_LEFT_JOINTS,),
            "names": LEFT_JOINT_NAMES,
        },
        "observation.images.head": {
            "dtype": "video",
            "shape": (IMG_H, IMG_W, 3),
            "names": ["height", "width", "channel"],
        },
        "observation.images.left_wrist": {
            "dtype": "video",
            "shape": (IMG_H, IMG_W, 3),
            "names": ["height", "width", "channel"],
        },
    }

    def _open_dataset(repo_id, repo_root, features, robot_type, label):
        """Create a new dataset or resume an existing one."""
        mode = "resume" if args.resume else "create"
        repo_root.parent.mkdir(parents=True, exist_ok=True)
        if args.resume:
            print(f"\nResuming {label}: {repo_id}")
            print(f"Local root : {repo_root}")
            return LeRobotDataset.resume(repo_id=repo_id, root=str(repo_root))
        else:
            print(f"\nCreating {label}: {repo_id}")
            print(f"Local root : {repo_root}")
            print(f"(delete {repo_root} to start fresh on re-run)")
            return LeRobotDataset.create(
                repo_id=repo_id,
                fps=args.fps,
                features=features,
                root=str(repo_root),
                robot_type=robot_type,
            )

    print(f"FPS : {args.fps}")
    dataset = _open_dataset(args.repo_id, root, features, "bimanual_openarm", "dataset")

    dataset_right = None
    if args.repo_id_right:
        root_right = base / args.repo_id_right
        dataset_right = _open_dataset(
            args.repo_id_right, root_right, features_right, "openarm_right", "right-only dataset")

    dataset_left = None
    if args.repo_id_left:
        root_left = base / args.repo_id_left
        dataset_left = _open_dataset(
            args.repo_id_left, root_left, features_left, "openarm_left", "left-only dataset")

    rclpy.init()
    recorder = OpenArmRecorder(
        dataset, args.fps,
        dataset_right=dataset_right,
        dataset_left=dataset_left,
    )

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(recorder,), daemon=True)
    spin_thread.start()

    print("\n=== OpenArm LeRobot Recorder ===")
    print("Prereqs: teleop stack + Quest connected (cameras must be active)\n")

    # Wait until at least one joint-state message has arrived
    print("Waiting for sensor data", end="", flush=True)
    while not recorder.data_ready():
        time.sleep(0.2)
        print(".", end="", flush=True)
    print(" ready!\n")

    last_task = args.task
    try:
        while True:
            task = input(
                f"\nTask [{last_task}] (ENTER=reuse, 'quit' to finish): ").strip()
            if task.lower() in ("quit", "q", "exit"):
                break
            if task == "":
                task = last_task
            else:
                last_task = task

            print(f"\n  Task : '{task}'")
            input("  Press ENTER to START recording...")
            recorder.start_episode(task)

            input("  [recording...] Press ENTER to STOP...")
            n = recorder.stop_episode()

            if n == 0:
                print("  No frames captured — skipped.\n")
                continue

            print(f"  Captured {n} frames ({n / args.fps:.1f}s)")
            print()
            print("  What to do with this episode?")
            print("    s / ENTER  — save (success)")
            print("    f          — save as FAILURE (keeps data, marks task)")
            print("    d          — discard (bad demo, throw away)")
            print()
            choice = input("  Choice [s/f/d]: ").strip().lower()

            if choice == "d":
                recorder.discard_episode()
                print("  🗑  Discarded.\n")
            elif choice == "f":
                failure_task = f"FAILURE: {task}"
                recorder._task = failure_task
                recorder.save_episode()
                print(f"  ⚠  Saved as failure: '{failure_task}'\n")
            else:
                recorder.save_episode()
                print(f"  ✅ Saved ({recorder._episode_count} total)\n")

            cmd = input("  Reset robot to start pose? [r + ENTER / ENTER to skip]: ").strip().lower()
            if cmd == 'r':
                recorder.send_reset()
                print("  ↺ Reset sent — robot moving to start pose\n")

    except KeyboardInterrupt:
        print("\n\nInterrupted.")
        if recorder._recording:
            recorder.stop_episode()
            recorder.discard_episode()

    finally:
        recorder.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    total = recorder._episode_count
    print(f"\nTotal episodes recorded: {total}")

    if total == 0:
        return

    if args.push_to_hub:
        for ds, rid in [
            (dataset,       args.repo_id),
            (dataset_right, args.repo_id_right),
            (dataset_left,  args.repo_id_left),
        ]:
            if ds is not None:
                print(f"Pushing {rid} to HuggingFace Hub...")
                ds.push_to_hub()
                _reupload_parquets(ds, rid)
                _push_readme(ds, rid)
                print(f"Done: https://huggingface.co/datasets/{rid}")
    else:
        print(f"\nTo push to Hub later:")
        for ds, rid, r in [
            (dataset,       args.repo_id,       root),
            (dataset_right, args.repo_id_right, base / args.repo_id_right if args.repo_id_right else None),
            (dataset_left,  args.repo_id_left,  base / args.repo_id_left  if args.repo_id_left  else None),
        ]:
            if ds is not None:
                print(f"  python -c \"from lerobot.datasets.lerobot_dataset import LeRobotDataset; "
                      f"d = LeRobotDataset('{rid}', root='{r}'); d.push_to_hub()\"")


if __name__ == "__main__":
    main()
