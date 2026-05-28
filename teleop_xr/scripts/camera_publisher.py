#!/usr/bin/env python3
"""
Standalone ROS2 camera publisher for OpenArm.

Publishes /camera/{head,left_wrist,right_wrist}/image_raw at 480x640 rgb8.
Use this during policy evaluation instead of launching the full demo_with_ros2.py
just to get camera topics.

Usage:
    source /opt/ros/jazzy/setup.bash
    source ~/OPEN_ARM/packages/install/setup.bash

    python3 scripts/camera_publisher.py \
        --head       /dev/video59 \
        --left-wrist /dev/video65 \
        --right-wrist /dev/video57

    # Or by integer index:
    python3 scripts/camera_publisher.py --head 0 --left-wrist 2 --right-wrist 4

    # Single camera only:
    python3 scripts/camera_publisher.py --head /dev/video59

Check actual device IDs with:
    v4l2-ctl --list-devices
"""

import argparse
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

W, H = 640, 480
FPS = 30


class CameraPublisherNode(Node):
    def __init__(self, cameras: dict[str, int | str]):
        """
        cameras: dict mapping view name → device spec
                 e.g. {"head": "/dev/video59", "left_wrist": 2}
        """
        super().__init__("openarm_camera_publisher")

        self._pubs: dict[str, any] = {}
        self._caps: dict[str, cv2.VideoCapture] = {}
        self._threads: list[threading.Thread] = []
        self._stop = threading.Event()

        for name, device in cameras.items():
            topic = f"/camera/{name}/image_raw"
            self._pubs[name] = self.create_publisher(Image, topic, 10)

            src = int(device) if str(device).isdigit() else device
            cap = cv2.VideoCapture(src, cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
            cap.set(cv2.CAP_PROP_FPS, FPS)

            if not cap.isOpened():
                self.get_logger().error(f"  [{name}] Failed to open {device}")
            else:
                self.get_logger().info(f"  [{name}] {device} → {topic}")
            self._caps[name] = cap

        for name in cameras:
            t = threading.Thread(
                target=self._reader_loop, args=(name,), daemon=True)
            t.start()
            self._threads.append(t)

        self.get_logger().info(
            f"Camera publisher running — {len(cameras)} camera(s) at {FPS} FPS")

    def _reader_loop(self, name: str) -> None:
        cap = self._caps[name]
        pub = self._pubs[name]
        period = 1.0 / FPS

        while not self._stop.is_set():
            t0 = time.monotonic()
            ok, bgr = cap.read()
            if not ok or bgr is None:
                self.get_logger().warning(
                    f"[{name}] Failed to read frame", throttle_duration_sec=5.0)
                time.sleep(0.05)
                continue

            if bgr.shape[1] != W or bgr.shape[0] != H:
                bgr = cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA)

            rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"{name}_camera"
            msg.height = H
            msg.width = W
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = W * 3
            msg.data = rgb.tobytes()
            pub.publish(msg)

            elapsed = time.monotonic() - t0
            sleep_t = period - elapsed
            if sleep_t > 0:
                time.sleep(sleep_t)

    def destroy_node(self):
        self._stop.set()
        for cap in self._caps.values():
            cap.release()
        super().destroy_node()


def main():
    parser = argparse.ArgumentParser(
        description="Standalone OpenArm camera publisher")
    parser.add_argument("--head",        default=None,
                        help="Head camera device  (e.g. /dev/video59 or 0)")
    parser.add_argument("--left-wrist",  default=None,
                        help="Left-wrist camera   (e.g. /dev/video65 or 2)")
    parser.add_argument("--right-wrist", default=None,
                        help="Right-wrist camera  (e.g. /dev/video57 or 4)")
    args = parser.parse_args()

    cameras = {}
    if args.head:
        cameras["head"] = args.head
    if args.left_wrist:
        cameras["left_wrist"] = args.left_wrist
    if args.right_wrist:
        cameras["right_wrist"] = args.right_wrist

    if not cameras:
        parser.error("Provide at least one of --head, --left-wrist, --right-wrist")

    rclpy.init()
    node = CameraPublisherNode(cameras)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
