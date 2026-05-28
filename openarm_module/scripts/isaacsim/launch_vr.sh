#!/bin/bash
# launch_vr.sh — Full VR teleoperation session launcher for OpenArm Isaac Sim
# Usage: ./launch_vr.sh [--headless] [--no-robot]


ISAAC_ENV=/home/vision/workspace/simlab/activate-isaacsim.sh
ALVR_DASHBOARD=/home/vision/alvr/alvr_streamer_linux/bin/alvr_dashboard
OPENARM_SIM=/home/vision/humanoids/openarm_module/scripts/isaacsim/openarm_sim.py
VR_KIT=/home/vision/workspace/simlab/.venv-isaacsim/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.base.xr.vr.kit

HEADLESS=false
NO_ROBOT=false

for arg in "$@"; do
    case $arg in
        --headless) HEADLESS=true ;;
        --no-robot) NO_ROBOT=true ;;
    esac
done

echo "=== OpenArm VR Launcher ==="

# Step 1: Check ADB / Quest 2 connection
echo "[1/4] Checking Quest 2 connection..."
DEVICES=$(adb devices | grep -v "List of devices" | grep "device$")
if [ -z "$DEVICES" ]; then
    echo "    WARNING: Quest 2 not detected via ADB."
    echo "    Make sure USB cable is connected and USB debugging is accepted in the headset."
else
    echo "    Quest 2 detected: $DEVICES"
fi

# Step 2: Start ALVR dashboard (background)
echo "[2/4] Starting ALVR dashboard..."
if pgrep -f "alvr_dashboard" > /dev/null; then
    echo "    ALVR dashboard already running."
else
    $ALVR_DASHBOARD &
    sleep 2
    echo "    ALVR dashboard started."
fi

# Step 3: Start SteamVR (background)
echo "[3/4] Starting SteamVR..."
if pgrep -f "vrmonitor" > /dev/null; then
    echo "    SteamVR already running."
else
    steam steam://run/250820 &
    echo "    Waiting for SteamVR to start..."
    sleep 5
    echo "    SteamVR started."
fi

echo ""
echo "    --> Put on your Quest 2 and launch ALVR from App Library > Unknown Sources"
echo "    --> Wait for the SteamVR environment to appear in the headset"
echo ""
read -p "Press ENTER when Quest 2 is connected to SteamVR..."

# Step 4: Launch Isaac Sim
echo "[4/4] Launching Isaac Sim..."
source $ISAAC_ENV

if [ "$HEADLESS" = true ]; then
    echo "    Launching in headless mode..."
    python $OPENARM_SIM --headless
elif [ "$NO_ROBOT" = true ]; then
    echo "    Launching Isaac Sim VR (no robot)..."
    isaacsim --experience $VR_KIT
else
    echo "    Launching Isaac Sim VR with OpenArm..."
    python $OPENARM_SIM
fi