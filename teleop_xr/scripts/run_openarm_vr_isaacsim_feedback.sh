#!/bin/bash
# Run openarm_vr_isaacsim_feedback.py inside env_isaacsim.
# Adds /joint_states feedback and optional IsaacSim camera image topics.
#
# Usage:
#   ./scripts/run_openarm_vr_isaacsim_feedback.sh [--usd /path/to/openarm_bimanual.usd] [--no-cameras]

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
REPO_ROOT=$(dirname "$SCRIPT_DIR")

ISAAC_SIM_PATH="/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/lib/python3.11/site-packages/isaacsim"
BRIDGE_PATH="$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge"

# Strip system ROS from environment. IsaacSim needs its bundled Python 3.11 rclpy.
unset AMENT_PREFIX_PATH
unset AMENT_CURRENT_PREFIX
unset COLCON_PREFIX_PATH
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')

# Inject IsaacSim internal ROS2 bridge (jazzy, Python 3.11).
export ROS_DISTRO=jazzy
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export PYTHONPATH="$BRIDGE_PATH/jazzy/rclpy:$PYTHONPATH"
export LD_LIBRARY_PATH="$LD_LIBRARY_PATH:$BRIDGE_PATH/jazzy/lib"

HOST_IP=$(ip route get 1.1.1.1 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") print $(i+1)}' | head -1)
[ -z "$HOST_IP" ] && HOST_IP=$(hostname -I | awk '{print $1}')

echo "======================================================"
echo " OpenArm VR Teleop — IsaacSim Feedback"
echo "======================================================"
echo "Subscribes: /joint_trajectory"
echo "Publishes:  /joint_states"
echo "Publishes:  /camera/head/image_raw, /camera/wrist_left/image_raw, /camera/wrist_right/image_raw when cameras exist"
echo "Quest URL:  https://${HOST_IP}:4443"
echo ""

/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/bin/python \
    "$REPO_ROOT/scripts/openarm_vr_isaacsim_feedback.py" "$@"
