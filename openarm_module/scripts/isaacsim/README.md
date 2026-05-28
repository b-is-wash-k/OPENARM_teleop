# VR Teleoperation Setup — OpenArm Isaac Sim

This document describes how to set up VR teleoperation for the OpenArm bimanual robot in Isaac Sim using a Meta Quest 2 on Ubuntu 22.04.

## System Requirements

- Ubuntu 22.04
- NVIDIA GPU (RTX 3060 or better)
- NVIDIA driver 535.129+ (tested with 580.x)
- CUDA 12.x
- Meta Quest 2 with developer mode enabled
- Steam installed

## Overview

The VR pipeline is:

```
Meta Quest 2 → ALVR (WiFi streaming) → SteamVR (OpenXR runtime) → Isaac Sim XR extensions → Robot visualization
```

---

## 1. Enable Developer Mode on Quest 2

1. Install the **Meta app** on your phone
2. Go to **Menu → Devices → [your Quest 2]**
3. Enable **Developer Mode**
4. On the Quest 2, accept the USB debugging prompt when connecting via USB-C

Verify ADB connection:
```bash
sudo apt install android-tools-adb
adb devices
# Should show your device serial number
```

Add udev rules for Quest 2 USB:
```bash
cat << 'EOF' | sudo tee /etc/udev/rules.d/51-oculus.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", ATTR{idProduct}=="5010", MODE="0666", GROUP="plugdev"
SUBSYSTEM=="usb", ATTR{idVendor}=="2833", ATTR{idProduct}=="5011", MODE="0666", GROUP="plugdev"
EOF
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG plugdev $USER
```

---

## 2. Install Steam and SteamVR

```bash
sudo apt install steam -y
```

Launch Steam, log in, then install **SteamVR** from the Steam store (App ID: 250820).

Launch SteamVR once to initialize its config files, then close it:
```bash
steam steam://run/250820
```

Add the following to SteamVR's launch options in Steam (right-click SteamVR → Properties → Launch Options):
```
/home/vision/.steam/debian-installation/steamapps/common/SteamVR/bin/vrmonitor.sh %command%
```

---

## 3. Install ALVR

ALVR streams Quest 2 over WiFi and registers as a SteamVR driver.

### Download ALVR streamer (PC side)
```bash
wget --no-check-certificate \
  https://github.com/alvr-org/ALVR/releases/download/v20.14.1/alvr_streamer_linux.tar.gz \
  -O ~/Downloads/alvr_streamer.tar.gz
mkdir -p ~/alvr
tar -xzf ~/Downloads/alvr_streamer.tar.gz -C ~/alvr
```

### Install ALVR client on Quest 2 (via ADB)
```bash
wget --no-check-certificate \
  https://github.com/alvr-org/ALVR/releases/download/v20.14.1/alvr_client_android.apk \
  -O ~/Downloads/alvr_client.apk
adb install ~/Downloads/alvr_client.apk
```

### Configure ALVR

Open the ALVR dashboard:
```bash
chmod +x ~/alvr/alvr_streamer_linux/bin/alvr_dashboard
~/alvr/alvr_streamer_linux/bin/alvr_dashboard
```

In the dashboard:
1. Go to **Installation** tab
2. Click **Register ALVR driver**
3. Click **Set firewall rules**

Also open firewall ports manually:
```bash
sudo ufw allow 9943/udp
sudo ufw allow 9944/udp
sudo ufw allow 9943/tcp
sudo ufw allow 9944/tcp
```

---

## 4. Configure OpenXR Runtime

Set SteamVR as the system OpenXR runtime:

```bash
mkdir -p ~/.config/openxr/1
cat > ~/.config/openxr/1/active_runtime.json << 'EOF'
{
    "file_format_version": "1.0.0",
    "runtime": {
        "library_path": "/home/vision/.steam/debian-installation/steamapps/common/SteamVR/bin/linux64/libopenxr_loader.so",
        "name": "SteamVR"
    }
}
EOF
```

---

## 5. Launch VR Session

Every time you want to use VR:

### Step 1 — Start ALVR dashboard
```bash
~/alvr/alvr_streamer_linux/bin/alvr_dashboard &
```

### Step 2 — Start SteamVR
Launch SteamVR from Steam GUI or:
```bash
steam steam://run/250820
```

### Step 3 — Connect Quest 2
- On the Quest 2, open **App Library → Unknown Sources → ALVR**
- Launch ALVR on the headset
- In the ALVR dashboard on theia, click **Trust** next to the device entry

### Step 4 — Launch Isaac Sim in VR mode
```bash
source ~/workspace/simlab/activate-isaacsim.sh
isaacsim --experience ~/workspace/simlab/.venv-isaacsim/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.base.xr.vr.kit
```

### Step 5 — Enable XR extensions in Isaac Sim
In Isaac Sim:
1. Go to **Window → Extensions**
2. Search for `xr`
3. Enable:
   - `omni.kit.xr.profile.vr`
   - `omni.kit.xr.core`
   - `omni.kit.xr.system.openxr`

### Step 6 — Load OpenArm robot
Open the bimanual USD:
```
~/humanoids/openarm_isaac_lab/source/openarm/openarm/tasks/manager_based/openarm_manipulation/usds/openarm_bimanual/openarm_bimanual.usd
```

Or run the sim script:
```bash
source ~/workspace/simlab/activate-isaacsim.sh
python ~/humanoids/openarm_module/scripts/isaacsim/openarm_sim.py
```

---

## File Locations

| Item | Path |
|------|------|
| ALVR streamer | `~/alvr/alvr_streamer_linux/` |
| ALVR client APK | `~/Downloads/alvr_client.apk` |
| OpenXR runtime config | `~/.config/openxr/1/active_runtime.json` |
| Isaac Sim VR kit | `~/.venv-isaacsim/lib/python3.11/site-packages/isaacsim/apps/isaacsim.exp.base.xr.vr.kit` |
| OpenArm bimanual USD | `~/humanoids/openarm_isaac_lab/source/openarm/.../openarm_bimanual.usd` |
| Sim scripts | `~/humanoids/openarm_module/scripts/isaacsim/` |

---

## Troubleshooting

**Quest 2 not detected by ADB**
- Unplug and replug USB cable
- Put on headset and accept the "Allow USB debugging" prompt
- Run `adb kill-server && adb devices`

**ALVR connection timeout**
- Ensure Quest 2 and theia are on the same WiFi network
- Re-run firewall rules in ALVR dashboard
- Check `sudo ufw status`

**SteamVR not starting properly**
- Make sure launch option is set: `vrmonitor.sh %command%`
- Launch SteamVR from Steam GUI first to initialize config

**OpenXR not found by Isaac Sim**
- Verify `~/.config/openxr/1/active_runtime.json` exists
- Make sure SteamVR is running before launching Isaac Sim

**rclpy not loading in Isaac Sim**
- Source the activate script before launching: `source ~/workspace/simlab/activate-isaacsim.sh`
- The activate script sets the required `LD_LIBRARY_PATH` for the ROS2 bridge

---

## Next Steps

- Write VR teleoperation script using Isaac Sim XR input APIs to read controller poses
- Implement IK (using cuRobo) to convert end-effector poses to joint angles
- Bridge joint commands to real OpenArm hardware via ROS2 → CAN bus