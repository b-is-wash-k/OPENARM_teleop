#!/bin/bash
# launch_isaacsim.sh — Launch Isaac Sim with OpenArm (no VR)
# Usage: ./launch_isaacsim.sh [--vr] [--headless]

ISAAC_ENV=/home/vision/workspace/simlab/activate-isaacsim.sh
OPENARM_SIM=/home/vision/humanoids/openarm_module/scripts/isaacsim/openarm_sim.py
VR_KIT=/home/vision/workspace/simlab/.venv-isaacsim/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.base.xr.vr.kit

VR=false
HEADLESS=false

for arg in "$@"; do
    case $arg in
        --vr) VR=true ;;
        --headless) HEADLESS=true ;;
    esac
done

source $ISAAC_ENV

if [ "$HEADLESS" = true ]; then
    echo "Launching Isaac Sim headless..."
    python $OPENARM_SIM --headless
elif [ "$VR" = true ]; then
    echo "Launching Isaac Sim in VR mode..."
    echo "Make sure SteamVR and ALVR are running first (run launch_alvr.sh)"
    isaacsim --experience $VR_KIT
else
    echo "Launching Isaac Sim with OpenArm..."
    python $OPENARM_SIM
fi