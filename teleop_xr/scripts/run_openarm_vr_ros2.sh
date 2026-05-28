#!/bin/bash
# Terminal 1 — teleop_xr IK server with ROS2 output.
# Uses the teleop_xr conda env (Python 3.12) + system ROS2 jazzy.
#
# Usage:
#   ./scripts/run_openarm_vr_ros2.sh

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
REPO_ROOT=$(dirname "$SCRIPT_DIR")

PYTHON="/home/air-lab-ncsu/anaconda3/envs/teleop_xr/bin/python"

# Source system ROS2 jazzy (Python 3.12 — matches teleop_xr env)
source /opt/ros/jazzy/setup.bash
# RMW: both sides use CycloneDDS (system default for jazzy — no override needed)

HOST_IP=$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)
[ -z "$HOST_IP" ] && HOST_IP=$(hostname -I | awk '{print $1}')

echo "============================================"
echo " OpenArm VR Teleop — ROS2 IK server"
echo "============================================"
echo "Publishes JointTrajectory -> /joint_trajectory"
echo "Connect Quest at: https://${HOST_IP}:4443"
echo "Hold BOTH left + right squeeze grips to engage teleop."
echo ""

$PYTHON -m teleop_xr.ros2 --mode ik --robot-class openarm
