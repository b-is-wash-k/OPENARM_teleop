#!/bin/bash
# System setup for OpenArm bimanual + leader arm module
set -e

echo "=== OpenArm System Setup ==="

echo "Installing OpenArm system packages..."
sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository -y ppa:openarm/main
sudo apt-get update
sudo apt-get install -y \
    can-utils \
    iproute2 \
    libopenarm-can-dev \
    openarm-can-utils

echo "OpenArm CLI tools installed"

echo "Setting up systemd-networkd CAN configuration..."

sudo tee /etc/systemd/network/can0.network > /dev/null <<'EOF'
[Match]
Name=can0

[CAN]
BitRate=1000000
DataBitRate=5000000
FDMode=yes
RestartSec=100ms
EOF

sudo tee /etc/systemd/network/can1.network > /dev/null <<'EOF'
[Match]
Name=can1

[CAN]
BitRate=1000000
DataBitRate=5000000
FDMode=yes
RestartSec=100ms
EOF

sudo tee /etc/systemd/network/can2.network > /dev/null <<'EOF'
[Match]
Name=can2

[CAN]
BitRate=1000000
DataBitRate=5000000
FDMode=yes
RestartSec=100ms
EOF

sudo tee /etc/systemd/network/can3.network > /dev/null <<'EOF'
[Match]
Name=can3

[CAN]
BitRate=1000000
DataBitRate=5000000
FDMode=yes
RestartSec=100ms
EOF

echo "Enabling systemd-networkd..."
sudo systemctl enable systemd-networkd
sudo systemctl start systemd-networkd

echo "Reloading networkd config..."
sudo networkctl reload

echo ""
echo "=== CAN Interface Status ==="
for iface in can0 can1 can2 can3; do
    ip link show $iface 2>/dev/null && echo "$iface is up" || echo "$iface not found (plug in USB adapter)"
done

echo ""
echo "Setup complete! CAN interfaces will now auto-configure on every boot and USB plug."
echo ""
echo "Next steps:"
echo "  1. Verify CAN: ip link show can0 can1 can2 can3"
echo "  2. Calibrate hardware:"
echo "     openarm-can-zero-position-calibration --canport can0 --arm-side right_arm  # follower right"
echo "     openarm-can-zero-position-calibration --canport can1 --arm-side left_arm   # follower left"
echo "     openarm-can-zero-position-calibration --canport can2 --arm-side right_arm  # leader right"
echo "     openarm-can-zero-position-calibration --canport can3 --arm-side left_arm   # leader left"