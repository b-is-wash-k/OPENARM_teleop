#!/bin/bash
# Relay /joint_trajectory -> left and right hardware controller topics.
# Run this between TeleopXR and the hardware bringup.

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
REPO_ROOT=$(dirname "$SCRIPT_DIR")

source /opt/ros/jazzy/setup.bash
source /home/air-lab-ncsu/OPEN_ARM/packages/install/setup.bash

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "======================================================"
echo " Joint Trajectory Relay"
echo "======================================================"
echo " SUB: /joint_trajectory"
echo " PUB: /left_joint_trajectory_controller/joint_trajectory"
echo " PUB: /right_joint_trajectory_controller/joint_trajectory"
echo "======================================================"

python3 "$SCRIPT_DIR/joint_trajectory_relay.py"
