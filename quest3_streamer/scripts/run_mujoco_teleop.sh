#!/bin/bash
# run_mujoco_teleop.sh
# Runs Quest3 → MuJoCo bimanual teleop (local viewer, no browser, no Isaac Sim).
#
# Requirements (one-time setup):
#   source .venv/bin/activate
#   pip install mujoco
#
# Usage (Terminal 2 — run alongside Terminal 1 which runs run_wireless.sh):
#   source .venv/bin/activate
#   source /opt/ros/jazzy/setup.bash
#   ./scripts/run_mujoco_teleop.sh

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" &> /dev/null && pwd)
PROJECT_ROOT=$(dirname "$SCRIPT_DIR")

# Verify mujoco is installed
if ! python -c "import mujoco" 2>/dev/null; then
    echo "ERROR: mujoco not installed in current Python environment."
    echo "Install it with:"
    echo "    source .venv/bin/activate && pip install mujoco"
    exit 1
fi

echo "Starting Quest3 → MuJoCo Teleop..."
echo "Model: $PROJECT_ROOT/../openarm/openarm/simulation/models/scene_openarm.xml"
echo ""

python "$PROJECT_ROOT/src/mujoco_teleop.py"
