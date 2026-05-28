#!/bin/bash
# Run openarm_vr_isaacsim.py inside the env_isaacsim conda environment.
# Do NOT source /opt/ros before calling this — IsaacSim ships its own ROS2 bridge.
#
# Usage:
#   ./scripts/run_openarm_vr_isaacsim.sh [--usd /path/to/openarm_bimanual.usd] [--headless]

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
REPO_ROOT=$(dirname "$SCRIPT_DIR")

ISAAC_SIM_PATH="/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim"
BRIDGE_PATH="$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge"

# Strip system ROS from environment
unset AMENT_PREFIX_PATH
unset AMENT_CURRENT_PREFIX
unset COLCON_PREFIX_PATH
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')

# Inject IsaacSim internal ROS2 bridge (jazzy, Python 3.11)
# Use CycloneDDS — same default as system ROS2 jazzy, same version (0.10.5) on both sides
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export PYTHONPATH="$BRIDGE_PATH/jazzy/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_PATH/jazzy/lib"

echo "============================================"
echo " OpenArm VR Teleop — IsaacSim"
echo "============================================"
echo "USD default: $REPO_ROOT/../openarm_isaac_lab/source/openarm/openarm/tasks/manager_based/openarm_manipulation/usds/openarm_bimanual/openarm_bimanual.usd"
echo "Input topic: /joint_trajectory (from teleop_xr.ros2)"
echo ""
HOST_IP=$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)
[ -z "$HOST_IP" ] && HOST_IP=$(hostname -I | awk '{print $1}')

echo "Make sure Terminal 1 is running:"
echo "  ./scripts/run_openarm_vr_ros2.sh"
echo "Then connect Quest at: https://${HOST_IP}:4443"
echo ""

/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/bin/python \
    "$REPO_ROOT/scripts/openarm_vr_isaacsim.py" "$@"
