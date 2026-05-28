#!/bin/bash
# launch_alvr.sh — Start ALVR + SteamVR only (without Isaac Sim)
# Useful for testing VR connection before launching the full sim

ALVR_DASHBOARD=/home/vision/alvr/alvr_streamer_linux/bin/alvr_dashboard

echo "=== Starting ALVR + SteamVR ==="

# Start ALVR
if pgrep -f "alvr_dashboard" > /dev/null; then
    echo "ALVR dashboard already running."
else
    $ALVR_DASHBOARD &
    sleep 2
    echo "ALVR dashboard started."
fi

# Start SteamVR
if pgrep -f "vrmonitor" > /dev/null; then
    echo "SteamVR already running."
else
    steam steam://run/250820 &
    echo "SteamVR starting..."
fi

echo ""
echo "On your Quest 2:"
echo "  1. Open App Library -> Unknown Sources -> ALVR"
echo "  2. Launch ALVR"
echo "  3. In the ALVR dashboard, click Trust next to your device"