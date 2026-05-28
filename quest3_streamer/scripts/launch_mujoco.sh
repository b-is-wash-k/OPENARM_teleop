#!/bin/bash
# launch_mujoco.sh
# Launches both terminals for Quest3 → MuJoCo teleoperation.
# Terminal 1: Quest wireless bridge (.venv + ROS2 Jazzy)
# Terminal 2: MuJoCo teleop viewer (.venv + ROS2 Jazzy + mujoco pip)
#
# One-time setup (run once before first use):
#   cd ~/OPEN_ARM/quest3_streamer
#   source .venv/bin/activate
#   pip install mujoco
#
# Usage:
#   cd ~/OPEN_ARM/quest3_streamer
#   ./scripts/launch_mujoco.sh

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

# Pass --no-deadman if you want arms always enabled:
#   MUJOCO_ARGS=--no-deadman ./scripts/launch_mujoco.sh
MUJOCO_ARGS="${MUJOCO_ARGS:-}"

echo "============================================"
echo " OpenArm Bimanual Teleop — MuJoCo Launch"
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
    echo 'Open on Quest: https://192.168.1.191:8000/web/webxr_streamer.html'
    echo ''
    ./scripts/run_wireless.sh
  "

sleep 1

# ── Terminal 2: MuJoCo Teleop ─────────────────────────────────────────────────
gnome-terminal \
  --title="MuJoCo Teleop Viewer" \
  -- bash --norc -c "
    # Always keep terminal open on any exit (trap before anything else)
    _pause() { echo ''; echo '=== Script ended. Type exit or press Ctrl+D to close ==='; exec bash --norc; }
    trap _pause EXIT

    echo '=== Terminal 2: MuJoCo Teleop ==='
    cd '$PROJECT_ROOT'

    echo '[1/3] Activating .venv...'
    source '$PROJECT_ROOT/.venv/bin/activate' || { echo 'ERROR: .venv not found'; exit 1; }

    echo '[2/3] Sourcing ROS2 Jazzy...'
    source /opt/ros/jazzy/setup.bash || { echo 'ERROR: ROS2 Jazzy not found at /opt/ros/jazzy'; exit 1; }

    echo '[3/3] Checking mujoco...'
    python -c 'import mujoco; print(\"mujoco\", mujoco.__version__, \"ok\")' || {
      echo ''
      echo 'ERROR: mujoco not installed in .venv. Fix with:'
      echo '    source .venv/bin/activate && pip install mujoco'
      exit 1
    }

    echo ''
    echo 'Starting mujoco_teleop.py  (args: $MUJOCO_ARGS)'
    echo 'The MuJoCo viewer window will open on your desktop.'
    echo ''
    python '$PROJECT_ROOT/src/mujoco_teleop.py' $MUJOCO_ARGS
  "

echo ""
echo "Both terminals launched."
echo ""
echo "Steps:"
echo "  1. Wait for Terminal 1 to print the Quest URL"
echo "  2. On Quest browser: https://192.168.1.191:8000/web/webxr_streamer.html"
echo "  3. Accept certificate → Start AR Session"
echo "  4. A MuJoCo viewer window will open on your desktop"
echo "  5. Hold both controllers still (~2s) → CALIBRATION COMPLETE"
echo "  6. Move hands to control the robot in the viewer"
