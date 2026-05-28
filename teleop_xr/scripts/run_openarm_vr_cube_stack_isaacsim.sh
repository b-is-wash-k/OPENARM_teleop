#!/bin/bash
# Run the OpenArm VR cube-stacking IsaacSim scene.
#
# Usage:
#   ./scripts/run_openarm_vr_cube_stack_isaacsim.sh [--cube-size 0.10] [--cube-spacing 0.10]

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
REPO_ROOT=$(dirname "$SCRIPT_DIR")

ISAAC_SIM_PATH="/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim"
BRIDGE_PATH="$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge"

unset AMENT_PREFIX_PATH
unset AMENT_CURRENT_PREFIX
unset COLCON_PREFIX_PATH
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')

export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$BRIDGE_PATH/jazzy/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_PATH/jazzy/lib"

HOST_IP=$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)
[ -z "$HOST_IP" ] && HOST_IP=$(hostname -I | awk '{print $1}')

echo "======================================================"
echo " OpenArm VR Teleop — Cube Stack IsaacSim"
echo "======================================================"
echo "Subscribes: /joint_trajectory"
echo "Publishes:  /joint_states and camera topics"
echo "Quest URL:  https://${HOST_IP}:4443"
echo ""

/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/bin/python \
    "$REPO_ROOT/scripts/openarm_vr_cube_stack_isaacsim.py" "$@"
