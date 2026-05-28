#!/bin/bash
# Terminal 1 — TeleopXR IK server with robot visual feedback and camera panels.
#
# Usage:
#   ./scripts/run_openarm_vr_ros2_feedback.sh

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
REPO_ROOT=$(dirname "$SCRIPT_DIR")

PYTHON="/home/air-lab-ncsu/anaconda3/envs/teleop_xr/bin/python"

# System ROS2 jazzy is Python 3.12, matching the teleop_xr conda env.
source /opt/ros/jazzy/setup.bash

# Must match IsaacSim feedback script.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

HOST_IP=$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)
[ -z "$HOST_IP" ] && HOST_IP=$(hostname -I | awk '{print $1}')

echo "======================================================"
echo " OpenArm VR Teleop — ROS2 IK + Feedback Viewer"
echo "======================================================"
echo "Publishes JointTrajectory -> /joint_trajectory"
echo "Subscribes JointState     <- /joint_states"
echo "Camera panels:"
echo "  head        <- /camera/head/image_raw"
echo "  wrist_left  <- /camera/wrist_left/image_raw"
echo "  wrist_right <- /camera/wrist_right/image_raw"
echo "Connect Quest at: https://${HOST_IP}:4443"
echo "Hold BOTH left + right squeeze grips to engage teleop."
echo ""

$PYTHON -m teleop_xr.ros2 \
    --mode ik \
    --robot-class openarm \
    --head-topic /camera/head/image_raw \
    --wrist-left-topic /camera/wrist_left/image_raw \
    --wrist-right-topic /camera/wrist_right/image_raw
