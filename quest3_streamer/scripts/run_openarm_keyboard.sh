#!/bin/bash
# run_openarm_keyboard.sh
# Launch OpenArm in Isaac Sim with keyboard SE(3) end-effector control.
# No ROS required.

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

# Strip system ROS from environment (prevents Python version conflicts with Isaac Sim)
unset AMENT_PREFIX_PATH
unset AMENT_CURRENT_PREFIX
unset COLCON_PREFIX_PATH
export PYTHONPATH=$(echo "$PYTHONPATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')
export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/ros' | tr '\n' ':' | sed 's/:$//')

echo "Starting OpenArm Keyboard Teleop..."
echo ""
echo "Controls:"
echo "  L       - Reset active arm to home"
echo "  Tab     - Switch active arm (left <-> right)"
echo "  K       - Toggle gripper"
echo "  W/S     - EE X (forward/back)"
echo "  A/D     - EE Y (left/right)"
echo "  Q/E     - EE Z (up/down)"
echo "  Z/X     - Roll +/-"
echo "  T/G     - Pitch +/-"
echo "  C/V     - Yaw +/-"
echo ""

/home/air-lab-ncsu/anaconda3/envs/env_isaacsim/bin/python \
    "$PROJECT_ROOT/src/isaac_openarm_keyboard_teleop.py" "$@"
