#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash

echo "Starting OpenArm RGB camera publishers using v4l2_camera..."
echo "  head/chest   : /dev/video53 -> /camera/head/image_raw"
echo "  left wrist   : /dev/video65 -> /camera/wrist_left/image_raw"
echo "  right wrist  : /dev/video59 -> /camera/wrist_right/image_raw"
echo

cleanup() {
  echo
  echo "Stopping camera nodes..."
  kill ${PIDS[@]} 2>/dev/null || true
}
trap cleanup EXIT INT TERM

PIDS=()

ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __node:=head_camera \
  -r __ns:=/camera/head \
  -p video_device:=/dev/video53 \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8 \
  -p image_size:="[640,480]" &
PIDS+=($!)

sleep 0.5

ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __node:=wrist_left_camera \
  -r __ns:=/camera/wrist_left \
  -p video_device:=/dev/video65 \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8 \
  -p image_size:="[640,480]" &
PIDS+=($!)

sleep 0.5

ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -r __node:=wrist_right_camera \
  -r __ns:=/camera/wrist_right \
  -p video_device:=/dev/video59 \
  -p pixel_format:=YUYV \
  -p output_encoding:=rgb8 \
  -p image_size:="[640,480]" &
PIDS+=($!)

sleep 2

echo
echo "Camera topics:"
ros2 topic list | grep "^/camera/" || true

echo
echo "Check rates with:"
echo "  ros2 topic hz /camera/head/image_raw"
echo "  ros2 topic hz /camera/wrist_left/image_raw"
echo "  ros2 topic hz /camera/wrist_right/image_raw"
echo
echo "All camera nodes started. Press Ctrl+C to stop."

wait
