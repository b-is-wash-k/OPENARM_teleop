#!/bin/bash

# Get project root directory (parent of scripts/)
SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

# Isaac Sim pip-based installation path (env_isaacsim conda env)
ISAAC_SIM_PATH="/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim"
BRIDGE_PATH="$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge"

# --- Strip system ROS from environment (causes Python version mismatch) ---
unset AMENT_PREFIX_PATH
unset AMENT_CURRENT_PREFIX
unset COLCON_PREFIX_PATH
# Remove /opt/ros/jazzy entries from PYTHONPATH
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')
# Remove /opt/ros entries from LD_LIBRARY_PATH
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')

# --- Set Isaac Sim internal ROS 2 bridge (jazzy, Python 3.11) ---
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$BRIDGE_PATH/jazzy/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_PATH/jazzy/lib"

echo "Starting OpenArm Bimanual Teleop..."
echo "ROS_DISTRO: $ROS_DISTRO"
echo "Internal rclpy: $BRIDGE_PATH/jazzy/rclpy"

/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/bin/python $PROJECT_ROOT/src/isaac_openarm_teleop.py
