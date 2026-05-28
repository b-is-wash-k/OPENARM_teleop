#!/bin/bash

# -----------------------------
# Project paths
# -----------------------------
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

# Isaac Sim pip installation path
ISAAC_SIM_PATH="/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim"
BRIDGE_PATH="$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge"

# -----------------------------
# Clean environment (very important!)
# -----------------------------
unset AMENT_PREFIX_PATH
unset AMENT_CURRENT_PREFIX
unset COLCON_PREFIX_PATH

# Remove system ROS from PYTHONPATH and LD_LIBRARY_PATH
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')

# -----------------------------
# Isaac Sim + ROS 2 Bridge settings
# -----------------------------
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

export PYTHONPATH="$BRIDGE_PATH/jazzy/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_PATH/jazzy/lib"

echo "Isaac Sim Path: $ISAAC_SIM_PATH"
echo "Starting Panda Teleop (USD Workflow)..."
echo "ROS_DISTRO: $ROS_DISTRO"

# -----------------------------
# Run with conda python (this is the correct way for pip isaacsim)
# -----------------------------
/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/bin/python "$PROJECT_ROOT/src/isaac_panda_teleop.py"