#!/bin/bash
# launch_all.sh
# Launches both the Quest wireless bridge and Isaac Sim teleop in separate terminals.
#
# Usage:
#   cd ~/OPEN_ARM/quest3_streamer
#   ./scripts/launch_all.sh

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

echo "============================================"
echo " OpenArm Bimanual Teleop — Full Launch"
echo "============================================"
echo "Project root: $PROJECT_ROOT"
echo ""
echo "Launching Terminal 1: Quest Wireless Bridge"
echo "Launching Terminal 2: Isaac Sim Teleop"
echo ""

# ── Terminal 1: Quest wireless bridge ────────────────────────────────────────
# Needs: .venv + system ROS2 jazzy
gnome-terminal \
  --title="Quest Wireless Bridge" \
  -- bash -c "
    echo '=== Terminal 1: Quest Wireless Bridge ==='
    cd '$PROJECT_ROOT'

    # Activate venv
    source '$PROJECT_ROOT/.venv/bin/activate'

    # Source system ROS2 jazzy
    source /opt/ros/jazzy/setup.bash

    echo 'Environment ready. Starting wireless bridge...'
    echo 'Open on Quest: https://192.168.1.191:8000/web/webxr_streamer.html'
    echo ''

    ./scripts/run_wireless.sh

    # Keep terminal open if it exits so you can read errors
    echo ''
    echo 'Bridge exited. Press Enter to close.'
    read
  "

# Small delay so both windows don't spawn at the exact same time
sleep 1

# ── Terminal 2: Isaac Sim Teleop ──────────────────────────────────────────────
# Needs: env_isaacsim conda — NO system ROS sourced
gnome-terminal \
  --title="Isaac Sim Teleop" \
  -- bash -c "
    echo '=== Terminal 2: Isaac Sim Teleop ==='
    cd '$PROJECT_ROOT'

    # Activate conda env_isaacsim (no system ROS)
    source '$HOME/anaconda3/etc/profile.d/conda.sh'
    conda activate env_isaacsim

    echo 'Conda env_isaacsim active.'
    echo 'Starting Isaac Sim... (takes ~60s to load)'
    echo ''

    ./scripts/run_openarm_teleop.sh

    echo ''
    echo 'Isaac Sim exited. Press Enter to close.'
    read
  "

echo ""
echo "Both terminals launched."
echo ""
echo "Steps:"
echo "  1. Wait for Terminal 1 to print the Quest URL"
echo "  2. On Quest browser: https://192.168.1.191:8000/web/webxr_streamer.html"
echo "  3. Accept certificate warning → Start AR Session"
echo "  4. Wait for Terminal 2 to finish loading Isaac Sim (~60s)"
echo "  5. Hold both controllers still → CALIBRATION COMPLETE"
echo "  6. Move hands to control the robot"