#!/bin/bash
# run_cube_stack.sh
# Launches both terminals for Quest3 → MuJoCo cube stacking teleoperation.
# Terminal 1: Quest wireless bridge (.venv + ROS2 Jazzy)
# Terminal 2: MuJoCo cube stack viewer (.venv + ROS2 Jazzy + mujoco pip)
#
# One-time setup (run once before first use):
#   cd ~/OPEN_ARM/quest3_streamer
#   source .venv/bin/activate
#   pip install mujoco
#
# Usage:
#   cd ~/OPEN_ARM/quest3_streamer
#   ./scripts/run_cube_stack.sh
#   MUJOCO_ARGS=--no-deadman ./scripts/run_cube_stack.sh

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

MUJOCO_ARGS="${MUJOCO_ARGS:-}"

echo "============================================"
echo " OpenArm Cube Stack Teleop — Launch"
echo "============================================"
echo "Project root: $PROJECT_ROOT"
echo ""

# ── Terminal 1: Quest wireless bridge ────────────────────────────────────────
gnome-terminal \
  --title="Quest Wireless Bridge" \
  -- bash --norc -c "
    _pause() { echo ''; echo '=== Script ended. Type exit or press Ctrl+D to close ==='; exec bash --norc; }
    trap _pause EXIT

    echo '=== Terminal 1: Quest Wireless Bridge ==='
    cd '$PROJECT_ROOT'
    source '$PROJECT_ROOT/.venv/bin/activate'
    source /opt/ros/jazzy/setup.bash
    echo 'Environment ready. Starting wireless bridge...'
    echo ''
    ./scripts/run_wireless.sh
  "

sleep 1

# ── Terminal 2: MuJoCo Cube Stack Teleop ─────────────────────────────────────
gnome-terminal \
  --title="MuJoCo Cube Stack" \
  -- bash --norc -c "
    _pause() { echo ''; echo '=== Script ended. Type exit or press Ctrl+D to close ==='; exec bash --norc; }
    trap _pause EXIT

    echo '=== Terminal 2: MuJoCo Cube Stack Teleop ==='
    cd '$PROJECT_ROOT'

    echo '[1/3] Activating .venv...'
    source '$PROJECT_ROOT/.venv/bin/activate' || { echo 'ERROR: .venv not found'; exit 1; }

    echo '[2/3] Sourcing ROS2 Jazzy...'
    source /opt/ros/jazzy/setup.bash || { echo 'ERROR: ROS2 Jazzy not found'; exit 1; }

    echo '[3/3] Checking mujoco...'
    python -c 'import mujoco; print(\"mujoco\", mujoco.__version__, \"ok\")' || {
      echo 'ERROR: mujoco not installed. Run: pip install mujoco'
      exit 1
    }

    echo ''
    echo 'Starting cube_stack_teleop.py  (args: $MUJOCO_ARGS)'
    echo 'Scene: table + red/blue/green cubes. Cubes obey physics.'
    echo ''
    python '$PROJECT_ROOT/src/cube_stack_teleop.py' $MUJOCO_ARGS
  "

echo ""
echo "Both terminals launched."
echo ""
echo "Steps:"
echo "  1. Wait for Terminal 1 to print the Quest URL"
echo "  2. On Quest browser: open the URL shown in Terminal 1"
echo "  3. Accept certificate → Start AR Session"
echo "  4. MuJoCo viewer opens — you'll see the robot + table + 3 cubes"
echo "  5. Hold both controllers still (~2s) → CALIBRATION COMPLETE"
echo "  6. Hold GRIP to enable each arm, move hands to control"
echo "  7. Use INDEX TRIGGER to close gripper and grasp cubes"
echo "  8. Cubes auto-reset if they fall off the table"
