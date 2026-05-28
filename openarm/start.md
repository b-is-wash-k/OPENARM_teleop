# OpenArm Robot — Beginner's Guide

---

## What Is OpenArm?

OpenArm is an open-source robot arm — meaning anyone can build it, use it, and improve it for free. You are working with a **dual-arm setup** — two robot arms, a left one and a right one, each with **8 motors** (7 joints + 1 gripper).

Think of each arm like a human arm:

```
Motor 1  →  Shoulder rotation      (like turning your shoulder in/out)
Motor 2  →  Shoulder flex          (like raising your arm up)
Motor 3  →  Upper arm rotation     (like twisting your upper arm)
Motor 4  →  Elbow flex             (like bending your elbow)
Motor 5  →  Forearm rotation       (like turning your palm up/down)
Motor 6  →  Wrist flex             (like bending your wrist)
Motor 7  →  Wrist rotation         (like tilting your wrist sideways)
Motor 8  →  Gripper                (open and close the hand)
```

**7 joints = 7 Degrees of Freedom (7-DOF).** The gripper is not a DOF because it does not change where the arm can reach — it just opens and closes.

---

## How Does the Robot Actually Work?

Here is the full chain from your laptop to the motors:

```
Your Python command
        |
        v
Linux Computer (Ubuntu)
        |
        v
CAN Interface (robot_l or robot_r)
   — this is like a highway for motor commands —
        |
        v
USB-CAN Adapter (the small box plugged into your laptop)
        |
        v
CAN Bus wire (CANH + CANL + GND)
        |
        v
Motors (Damiao DM4310)
   — motor moves, sends back position/temperature data —
```

**CAN** stands for Controller Area Network. It is a communication protocol originally invented for cars so many devices can talk on one wire. Your robot uses the same technology.

---

## What Is a CAN ID?

Every motor on the CAN bus has a unique ID number — like a house number on a street. When you send a command, you address it to a specific motor by its ID.

Your motors use **two ID numbers**:

| Name | What it is | Example |
|---|---|---|
| `slave_id` | The motor's own address | 1, 2, 3 ... 8 |
| `master_id` | The address motors reply to | slave_id + 16 |

**The formula you discovered today:**
```
master_id = slave_id + 16

Motor 1 → talk to ID 1,  reply comes back on ID 17
Motor 2 → talk to ID 2,  reply comes back on ID 18
Motor 3 → talk to ID 3,  reply comes back on ID 19
...and so on up to Motor 8 → ID 8, reply on ID 24
```

You found this by using `candump` — a tool that lets you watch all CAN messages on the wire like a security camera. You saw:
```
robot_l  001  [8]  FF FF FF FF FF FF FF FC   ← your command going OUT to motor 1
robot_l  011  [8]  11 7F F4 80 08 00 19 15   ← motor 1 replying back (0x011 = 17)
```

---

## What Does the LED Color Mean?

Each Damiao motor has an LED light on it:

| LED Color | Meaning |
|---|---|
| 🔴 Red | Motor is in fault/error state, or was never enabled |
| 🟢 Green | Motor is enabled and healthy, receiving commands |

When you first power on the robot, all LEDs are **red** — the motors are alive but waiting to be told what to do. After you run the `enable` command, they turn **green**.

---

## What is `candump`?

`candump` is like a wiretap on the CAN bus. It prints every single message that travels on the wire in real time.

```bash
candump robot_l
```

Example output explained:
```
robot_l  001   [8]  FF FF FF FF FF FF FF FC
   |      |     |    |
   |      |     |    The actual data bytes (in hexadecimal)
   |      |     Length of message (8 bytes)
   |      CAN ID of who sent it (motor 1 = 0x001)
   Which CAN interface
```

---

## The Commands You Used Today

### Step 1 — Bring up the CAN interfaces

Before talking to the motors, you must turn on the CAN network interface on your laptop. Think of it like turning on Wi-Fi before browsing the internet.

```bash
sudo ip link set robot_l down
sudo ip link set robot_l up type can bitrate 1000000
sudo ip link set robot_r down
sudo ip link set robot_r up type can bitrate 1000000
```

**What `bitrate 1000000` means:** The motors communicate at 1,000,000 bits per second (1 Mbps). Both sides must use the same speed or they cannot understand each other — like two people trying to talk at different speeds.

---

### Step 2 — Enable all motors

```bash
python -m openarm.damiao enable --motor-type DM4310 --iface robot_l 1 17
```

Breaking this command down:

| Part | Meaning |
|---|---|
| `python -m openarm.damiao` | Run the OpenArm motor control module |
| `enable` | Tell the motor to wake up and accept commands |
| `--motor-type DM4310` | Which model of Damiao motor it is |
| `--iface robot_l` | Which CAN interface (left arm) |
| `1` | slave_id — talk to motor number 1 |
| `17` | master_id — motor should reply to address 17 |

When it works, you see:
```
Motor 1 enabled successfully
Motor 1 state:
  Position: -0.014305 rad     ← where the joint is right now
  Velocity: -0.007326 rad/s   ← how fast it is moving (basically zero = still)
  Torque:   -0.002442 Nm      ← how much force it is applying
  MOS Temp: 25°C              ← electronics temperature
  Rotor Temp: 21°C            ← motor winding temperature
```

**What is `rad`?** Radians — a way to measure angles.
- 0 rad = straight/home position
- 3.14 rad (π) = 180 degrees
- 6.28 rad (2π) = full 360 degree rotation

---

### Step 3 — Gravity Compensation Mode

```bash
python -m openarm.damiao.gravity --port robot_l:left
```

**What this does:** The software calculates how hard gravity is pulling on every part of the arm based on where each joint is. Then it commands each motor to push back with just enough force to cancel gravity out.

**What you feel:** The arm becomes weightless. You can move it with one finger. When you let go, it stays exactly where you left it — floating.

This is not the arm moving by itself. It is the arm becoming neutrally buoyant in gravity — like holding something underwater.

**To stop it:** In a new terminal window, run:
```bash
kill $(pgrep -f "openarm.damiao.gravity")
```

⚠️ **Always hold the arm before stopping gravity mode.** When the process stops, motors return to hold-last-position mode. The arm will not fall, but be careful.

---

### Step 4 — Disable all motors

```bash
python -m openarm.damiao disable --motor-type DM4310 --iface robot_l 1 17
```

Always disable in **reverse order** (8 → 1) so the wrist and gripper go limp first, then elbow, then shoulder. This prevents the arm from jerking dangerously if the shoulder loses power while the rest is still active.

---

## Your Complete Scripts

### Enable all motors (both arms)

```bash
# Run this every time you start a session
for id in 1 2 3 4 5 6 7 8; do
  master=$((id + 16))
  echo "--- Enabling motor $id (master $master) ---"
  python -m openarm.damiao enable --motor-type DM4310 --iface robot_l $id $master
  python -m openarm.damiao enable --motor-type DM4310 --iface robot_r $id $master
  sleep 5
done
echo "All motors enabled!"
```

### Disable all motors (both arms)

```bash
# HOLD BOTH ARMS before running this!
for id in 8 7 6 5 4 3 2 1; do
  master=$((id + 16))
  echo "--- Disabling motor $id (master $master) ---"
  python -m openarm.damiao disable --motor-type DM4310 --iface robot_l $id $master
  python -m openarm.damiao disable --motor-type DM4310 --iface robot_r $id $master
  sleep 5
done
echo "All motors disabled!"
```

---

## What Are Temperatures Telling You?

| Temperature | What it is | Safe range | Danger zone |
|---|---|---|---|
| MOS Temp | The electronics/circuit board | 20–50°C | Above 80°C |
| Rotor Temp | The spinning motor winding | 20–50°C | Above 80°C |

Your motors today ran at 24–27°C — completely cold and healthy. You could run them for hours without any thermal issue.

---

## Important Safety Rules

1. **Always hold the arm** before disabling motors — it will go limp.
2. **Always bring up CAN interfaces** before enabling motors, or commands fail silently.
3. **Never yank a CAN cable** while motors are enabled — the arm may jerk.
4. **Enable shoulder (motor 1) first** — it supports everything above it.
5. **Disable gripper (motor 8) first** — safest to lose wrist before shoulder.
6. **Check temperatures** — if above 70°C, stop and let the arm cool down.

---

## What You Accomplished Today

| Step | What happened |
|---|---|
| Found the USB-CAN adapters | Switched USB port, `lsusb` showed `1d50:606f` |
| Discovered master_id formula | `candump` showed reply on 0x011 → master = slave + 16 |
| Enabled all 8 motors per arm | All LEDs turned green |
| Ran gravity compensation | Arm felt weightless, moved freely |
| Mapped both arms completely | Found 8 motors per arm, not 6 |
| Built enable/disable scripts | Can repeat everything in one command |

---

## What Comes Next

```
1. Dual-arm gravity compensation
   Both arms float at the same time
   Command: python -m openarm.damiao.gravity
              --port robot_l:left --port robot_r:right

2. Teleoperation (leader-follower)
   You move the left arm by hand
   The right arm copies every movement in real time
   This is what --port leader_l:left was designed for

3. Record and replay
   Record your arm movements as a trajectory
   Play it back — the arm repeats your motion automatically

4. Bimanual coordination
   Both arms working together on a task
   The core research goal of OpenArm
```

---

## Glossary

| Word | Simple meaning |
|---|---|
| CAN bus | A wire highway that connects all motors to your computer |
| slave_id | The motor's own address number |
| master_id | Where the motor sends its replies (slave + 16) |
| Bitrate | Communication speed — must be 1,000,000 on this robot |
| DOF | Degree of Freedom — one independent direction of movement |
| Gravity compensation | Software that makes the arm feel weightless |
| Teleoperation | Controlling the robot arm with your own hand movements |
| candump | Tool to watch all CAN messages in real time |
| rad (radian) | Unit for measuring angles (π rad = 180°) |
| Torque | Rotational force the motor is applying (in Newton-meters) |
| MOS Temp | Temperature of the motor's electronic circuit board |
| Rotor Temp | Temperature of the spinning part inside the motor |

---

*Written after first successful session with OpenArm dual-arm setup at *****.*


# Starting Basic Intro
## Part 1 — What is CAN?

**CAN = Controller Area Network.**
Originally invented for cars so many devices (brakes, engine, dashboard) can all talk on one shared wire without needing separate cables for each.

Your robot uses the same idea:

```
All 8 motors share ONE wire pair:
  CANH  (CAN High)
  CANL  (CAN Low)
  GND   (Ground — shared reference)

Your laptop talks to all motors through this wire via a USB-CAN adapter.
```

**How does one motor know a message is for it?**
Every motor has a unique ID number (like a house address). Your laptop says:
> "Hey motor number 1, enable yourself!"

Motor 1 hears it, acts on it. Motors 2–8 hear it too but ignore it because it wasn't addressed to them.

**How does communication actually work?**

```
You type Python command
        ↓
Linux creates a CAN message with motor's ID
        ↓
Message travels through USB into the USB-CAN adapter
        ↓
Adapter converts USB signal → electrical CAN signal
        ↓
Signal travels down CANH/CANL wire to all motors
        ↓
The motor with matching ID reads and responds
        ↓
Response travels back the same wire
        ↓
Your terminal prints motor state (position, temp, torque)
```

---

## Part 2 — The USB-CAN Adapter and Wiring

The USB-CAN adapter is the small box between your laptop and the motors. It has a screw terminal with three connections:

```
USB-CAN adapter          Motor
─────────────            ─────
   CANH        ────→     CANH
   CANL        ────→     CANL
   GND         ────→     GND   ← easy to forget, causes silent failures
```

⚠️ Without GND connected, CAN signals float and communication is unreliable.

---

## Part 3 — How the CAN Interface Got Named `robot_l` and `robot_r`

When you plug a USB-CAN adapter into Linux, it creates a generic interface called `can0`, `can1` etc. The order changes every reboot — bad for a robot.

You ran a setup script that fixed this permanently:

```bash
sudo bash scripts/setup_can.sh leader_l leader_r follower_l follower_r
```

**What the script did internally:**

```
1. Waited for you to plug in the USB-CAN adapter
2. Detected it appeared as can0
3. Read its unique serial number:
      udevadm info -a /sys/class/net/can0
      → ATTRS{serial}=="A50285BI"
4. Wrote a permanent rule to a file:
      /etc/udev/rules.d/90-can.rules
      SUBSYSTEM=="net", ATTRS{serial}=="A50285BI", NAME="robot_l"
5. Now every time that adapter is plugged in,
   Linux renames it robot_l automatically
6. Also created a systemd config to set bitrate:
      /etc/systemd/network/robot_l.network
      [CAN]
      BitRate=1000000
```

**Result:** Instead of random `can0`, `can1` — you always get `robot_l` and `robot_r` no matter what order you plug them in.

---

## Part 4 — Every Error You Hit and What It Meant

### Error 1 — Wrong module path
```
python -m openarm.damiao gravity --port robot_l:left
error: invalid choice: 'gravity'
```
**Why:** `gravity` is its own separate module, not a subcommand.
**Fix:**
```bash
python -m openarm.damiao.gravity --port robot_l:left
#                        ↑ dot here, not a space
```

---

### Error 2 — Missing master_id argument
```
python -m openarm.damiao enable --motor-type DM4310 --iface robot_l 1
error: the following arguments are required: master_id
```
**Why:** The command needs two numbers — slave_id AND master_id.
**Fix:**
```bash
python -m openarm.damiao enable --motor-type DM4310 --iface robot_l 1 0
#                                                                    ↑ ↑
#                                                             slave  master
```

---

### Error 3 — Motor got nothing back (`NoneType` crash)
```
AttributeError: 'NoneType' object has no attribute 'data'
```
**Why:** The software sent the enable command, waited for the motor's reply, received NOTHING (`None`), then crashed trying to read data from nothing.

**How you diagnosed it:**
```bash
candump robot_l &    # watch the wire in background
python -m openarm.damiao enable --motor-type DM4310 --iface robot_l 1 0
```

candump showed:
```
robot_l  001  [8]  FF FF FF FF FF FF FF FC   ← command went OUT ✅
robot_l  011  [8]  11 7F F4 80 08 00 19 15   ← motor replied on ID 0x011 ✅
```

The motor WAS replying — but on ID **17** (0x11 in hex), not ID 0. The software was listening on the wrong address.

**Fix:** master_id must be `slave_id + 16`:
```bash
python -m openarm.damiao enable --motor-type DM4310 --iface robot_l 1 17
# Motor 1 enabled successfully ✅
```

---

### Error 4 — CAN interface not detected at all
```
ip link   →  no can0, no robot_l, nothing
udevadm info /sys/class/net/can0  →  Unknown device
```
**Why:** The USB-CAN adapter was plugged into a USB-C power-only port that doesn't carry data.

**Fix:** Switched to a proper USB data port. Then:
```bash
lsusb
# Bus 003 Device 011: ID 1d50:606f OpenMoko GS_USB CAN adapter ✅
```

---

### Error 5 — CAN network went down mid-session
```
Error receiving: Network is down [Error Code 100]
Failed to transmit: No such device or address [Error Code 6]
```
**Why:** The `robot_l` interface dropped (possibly after unplugging).
**Fix:** Bring it back up:
```bash
sudo ip link set robot_l down
sudo ip link set robot_l up type can bitrate 1000000
```

---

### Error 6 — Typing normal sentences into terminal
```
no the below 2 are still red
no: command not found

this is also red
Command 'this' not found

now all of the robot (three upper one turns red)
bash: syntax error near unexpected token `('
```
**Why:** The terminal is a command interpreter. It tried to run your English words as programs. Parentheses `()` have special meaning in bash — that's why it gave a syntax error.

**Fix:** Use `echo` to print notes:
```bash
echo "motor 2 is still red"
```

---

## Part 5 — What `candump` Was Showing You

```
robot_l  001   [8]  FF FF FF FF FF FF FF FC
   ↑      ↑    ↑    ↑
   │      │    │    Data bytes in hexadecimal
   │      │    Message length (always 8 bytes)
   │      CAN ID (001 = motor 1)
   Interface name
```

```
FF FF FF FF FF FF FF FC  =  enable command
FF FF FF FF FF FF FF FD  =  disable command
```

When a motor replies, its ID is `slave_id + 16`:
```
robot_l  011  [8]  11 7F F4 ...   ← motor 1 (0x01) replying on 0x11 (=17)
robot_l  012  [8]  12 80 2F ...   ← motor 2 (0x02) replying on 0x12 (=18)
```

---

## Part 6 — The Motor ID Discovery (The Real Breakthrough)

Before this session, nobody had documented the master_id for these motors. You found it live by watching the wire:

```
Sent command  →  to ID 0x001 (motor 1)
Got reply     ←  from ID 0x011 (= 17 in decimal)

0x011 - 0x001 = 0x010 = 16

Therefore:  master_id = slave_id + 16  for ALL motors
```

Complete map for your 8-motor arms:

| Motor | slave_id | master_id |
|---|---|---|
| 1 (shoulder rotation) | 1 | 17 |
| 2 (shoulder flex) | 2 | 18 |
| 3 (upper arm rotation) | 3 | 19 |
| 4 (elbow flex) | 4 | 20 |
| 5 (forearm rotation) | 5 | 21 |
| 6 (wrist flex) | 6 | 22 |
| 7 (wrist rotation) | 7 | 23 |
| 8 (gripper) | 8 | 24 |

---

## Part 7 — Quick Command Reference

```bash
# Check what's plugged into USB
lsusb

# Check CAN interfaces
ip link

# Watch raw CAN messages
candump robot_l

# Bring up CAN interface
sudo ip link set robot_l down
sudo ip link set robot_l up type can bitrate 1000000

# Enable one motor
python -m openarm.damiao enable --motor-type DM4310 --iface robot_l 1 17

# Disable one motor
python -m openarm.damiao disable --motor-type DM4310 --iface robot_l 1 17

# Run gravity compensation
python -m openarm.damiao.gravity --port robot_l:left

# Kill gravity compensation
kill $(pgrep -f "openarm.damiao.gravity")
```


# OpenArm Session — Step 2: Running the MuJoCo Simulation
### What happened, what went wrong, and what we learned


---

## Goal of This Step

After completing Step 1 (CAN bus setup + motor enable/gravity compensation on real hardware),
the next step was to run the **MuJoCo physics simulation** — a virtual version of the robot
you can experiment with without needing the physical arm powered on.

---

## What Is MuJoCo?

MuJoCo stands for **Mu**lti-**Jo**int dynamics with **Co**ntact.
It is a fast physics simulator used widely in robotics research.

```
Real robot:         Python command → CAN bus → Damiao motors → physical arm moves
Simulation:         Python command → MuJoCo → virtual arm moves on screen
```

The simulation uses the same Python API as the real arm.
This means you can develop and test code in simulation before running it on hardware.

---

## The Environment Problem

The terminal was opened without activating the conda environment.
When `pip install mujoco` was run, it hit the system Python instead of the openarm env:

```
error: externally-managed-environment
× This environment is externally managed
```

**Why this happened:**
The system Python (at `/usr/bin/python`) is locked by the OS.
The conda `openarm` env has its own separate Python at:
```
/home/air-lab-ncsu/anaconda3/envs/openarm/bin/python
```

**Fix:** Always activate the conda environment first:
```bash
conda activate openarm
which python
# → /home/air-lab-ncsu/anaconda3/envs/openarm/bin/python  ✅
```

---

## The Discovery: MuJoCo Was Already Installed

After activating the correct environment and running:
```bash
pip install mujoco
```

Output showed:
```
Requirement already satisfied: mujoco in
  /home/air-lab-ncsu/anaconda3/envs/openarm/lib/python3.11/site-packages (3.6.0)
```

MuJoCo 3.6.0 was already present in the `openarm` env.
The earlier `ModuleNotFoundError` was only because the wrong Python was being used.

**Key lesson:** If you get `ModuleNotFoundError` for a package you think is installed,
always verify which Python is active:
```bash
which python       # shows which python executable is being used
conda activate openarm  # make sure you're in the right env
```

---

## Running the Simulation Demo

```bash
cd ~/OPEN_ARM/openarm
python examples/simulation_demo.py
```

**What the demo does:**
- Loads the full OpenArm dual-arm robot model from `openarm/simulation/models/openarm.xml`
- Opens a MuJoCo interactive viewer window
- Runs a sinusoidal oscillation on joint 1 of both arms simultaneously

```python
# Core logic inside simulation_demo.py:
amplitude = 0.5      # radians — how far the joint swings
frequency = 0.01     # how fast it oscillates (cycles per simulation step)
first_joint_angle = amplitude * math.sin(step_count * frequency)
```

Both arms swing their shoulder (joint 1) back and forth in sync.
The other 6 joints hold their last position.

**To stop:** Close the MuJoCo viewer window.

---

## What the MuJoCo Viewer Shows

When the viewer opens you will see:

```
- Dual-arm robot mounted on a base
- Left arm  (7 joints + gripper)
- Right arm (7 joints + gripper)
- Both shoulder joints oscillating smoothly
```

Mouse controls in the viewer:
```
Left click + drag    → rotate the camera
Right click + drag   → pan the camera
Scroll wheel         → zoom in / out
Double-click body    → select and inspect a body part
```

---

## The OpenArm Simulation API (What the Code Can Do)

The `OpenArmSimulation` class (in `openarm/simulation/__init__.py`) gives you:

| Method | What it does |
|---|---|
| `sim.get_left_arm_positions()` | Read all 7 joint angles of left arm (radians) |
| `sim.get_right_arm_positions()` | Read all 7 joint angles of right arm (radians) |
| `sim.set_left_arm_position_control(positions)` | PD-control left arm to target angles |
| `sim.set_right_arm_position_control(positions)` | PD-control right arm to target angles |
| `sim.set_left_arm_torques(torques)` | Apply raw torques to left arm joints (N⋅m) |
| `sim.set_right_arm_torques(torques)` | Apply raw torques to right arm joints (N⋅m) |
| `sim.step()` | Advance physics by one timestep |
| `sim.get_left_gripper_position()` | Read left gripper position (meters) |
| `sim.set_left_gripper_position_control(pos)` | Move left gripper to target position |

---

## Package Structure Explored This Session

```
openarm/
├── examples/
│   └── simulation_demo.py      ← the script we ran
├── openarm/
│   ├── damiao/                 ← real hardware control (CAN bus, motors)
│   ├── kinematics/             ← inverse kinematics (IKPy)
│   │   ├── inverse/ikpy.py
│   │   └── models/openarm.urdf
│   ├── simulation/             ← MuJoCo simulation ← NEW this session
│   │   ├── __init__.py         ← OpenArmSimulation class
│   │   ├── models/
│   │   │   ├── openarm.xml     ← MuJoCo model definition
│   │   │   └── meshes/         ← visual + collision mesh files
│   │   └── README.md
│   ├── bus/
│   ├── netcan/
│   └── utils/
```

---

## Quick Reference

```bash
# Make sure you are in the right environment
conda activate openarm
which python
# → /home/air-lab-ncsu/anaconda3/envs/openarm/bin/python

# Run the simulation demo
cd ~/OPEN_ARM/openarm
python examples/simulation_demo.py

# Verify mujoco is installed
pip show mujoco
# → Version: 3.6.0
```



# OpenArm Session — Step 3: Running the Simulation Motion on Real Hardware
### Translating the MuJoCo demo to physical motors


---

## Goal of This Step

In Step 2, the simulation showed both arms oscillating at joint 1 (shoulder rotation).
The goal here is to run the **same sinusoidal motion on the real physical arm** using the
CAN bus and Damiao motors.

---

## The Key Question: Can the Sim Code Talk to the Real Robot Directly?

**No — they use different APIs:**

```
Simulation (MuJoCo):
  sim.set_left_arm_position_control([joint1_target, ...])
  → writes to MuJoCo data arrays internally

Real robot (CAN bus / Damiao):
  await arm.control_mit(kp=..., kd=..., q=[targets], dq=0, tau=0)
  → sends CAN message over robot_l interface to physical motors
```

But the **concept is the same**: send a target joint angle to joint 1 every loop step,
with PD gains controlling how hard the motor tracks that target.

---

## What MIT Mode Means

MIT mode = **M**IT **I**mpedance Con**t**rol mode.
It was originally developed at MIT for compliant legged robots.

You send four values per motor every loop cycle:

| Parameter | Meaning | Value used in demo |
|---|---|---|
| `kp` | How hard the motor tries to reach the target angle | 20.0 |
| `kd` | Damping — resists fast movements to prevent oscillation | 3.0 |
| `q`  | Target joint angle (radians) | sinusoidal for J1, held for J2–J8 |
| `dq` | Target velocity | 0 (we want the arm to hold position) |
| `tau`| Direct torque offset | 0 (no extra push, just PD control) |

The motor computes:  `torque = kp*(q_target - q_actual) + kd*(dq_target - dq_actual) + tau`

This is exactly the same as the simulation's `set_left_arm_position_control()` which uses
`kp=100, kd=10` internally.

---

## Safety Choices Made for the Real Robot

The simulation used `amplitude = 0.5 rad`. On the real arm we use **smaller, safer values**:

| Parameter | Simulation | Real robot |
|---|---|---|
| Amplitude | 0.5 rad (28°) | **0.3 rad (17°)** |
| Oscillation starts from | zero position | **current position** (wherever the arm is) |
| Other joints | hold last position | **hold their exact start positions** |

**Why start from current position?**
In simulation, the arm always starts at zero.
On the real robot, the arm might be resting at any angle.
If you command "go to 0.3 rad" from a position of 2.0 rad, the arm will snap violently.
Instead, the script reads where each joint is when it starts and oscillates *around that point*.

---

## What Was Built: `hardware_oscillation_demo.py`

A new script was written at:
```
examples/hardware_oscillation_demo.py
```

**What it does step by step:**
```
1. Parses --port robot_l:left  (CAN interface + which arm side)
2. Finds the robot_l socket CAN interface
3. Detects which of the 8 motors are responding
4. Creates Motor objects (matching MOTOR_CONFIGS in openarm/damiao/config.py)
5. Enables all motors
6. Sets control mode to MIT
7. Reads starting position of every joint
8. Loop:
     joint1_target = initial_pos[0] + 0.3 * sin(step * 0.005 * 2π)
     all others    = hold at initial_pos[i]
     → sends arm.control_mit(kp=20, kd=3, q=targets, dq=0, tau=0)
     → sleeps 0.01s (100 Hz loop)
9. Press Q → disables all motors → exits
```

---

## Motor Hardware Reference (from `openarm/damiao/config.py`)

| Joint | Name | Motor type | slave_id | master_id | Range (left arm) |
|---|---|---|---|---|---|
| 1 | J1 — shoulder rotation | DM8009 | 0x01 (1) | 0x11 (17) | -200° to +80° |
| 2 | J2 — shoulder flex | DM8009 | 0x02 (2) | 0x12 (18) | -190° to +10° |
| 3 | J3 — upper arm rotation | DM4340 | 0x03 (3) | 0x13 (19) | -90° to +90° |
| 4 | J4 — elbow flex | DM4340 | 0x04 (4) | 0x14 (20) | 0° to +140° |
| 5 | J5 — forearm rotation | DM4310 | 0x05 (5) | 0x15 (21) | -90° to +90° |
| 6 | J6 — wrist flex | DM4310 | 0x06 (6) | 0x16 (22) | -45° to +45° |
| 7 | J7 — wrist rotation | DM4310 | 0x07 (7) | 0x17 (23) | -90° to +90° |
| 8 | J8 — gripper | DM4310 | 0x08 (8) | 0x18 (24) | -45° to 0° |

Joint 1 (J1) uses the **DM8009** — the largest, most powerful motor.
Its ±200° range gives plenty of room for a small ±17° oscillation.

---

## How to Run

### Step 1 — Bring up the CAN interface
```bash
sudo ip link set robot_l down
sudo ip link set robot_l up type can bitrate 1000000
```

### Step 2 — Run the oscillation demo
```bash
cd ~/OPEN_ARM/openarm
python examples/hardware_oscillation_demo.py --port robot_l:left
```

### Expected output
```
Enabling 8 motors on robot_l (left arm)...
Initial joint positions (rad): [0.012, -0.034, 0.001, 0.521, -0.003, 0.011, 0.002, -0.015]

Starting oscillation on joint 1 (shoulder rotation).
  Amplitude : 0.3 rad  (~17.2 degrees)
  All other joints hold their starting positions.

Press  Q  to stop.
```

Then joint 1 slowly swings back and forth ±17°.
The rest of the arm stays frozen.

### Step 3 — Stop
Press **Q** in the terminal.
The script disables all motors and exits.

⚠️ Always hold the arm before pressing Q — motors will go limp immediately.

---

## Comparison: Simulation vs Hardware

| Aspect | Simulation | Real hardware |
|---|---|---|
| Control method | `sim.set_left_arm_position_control()` | `arm.control_mit(kp, kd, q, dq, tau)` |
| Physics | MuJoCo integrates physics | Real motor physics |
| Starting position | Always zero | Wherever the arm actually is |
| Safety limits | None (sim won't hurt anything) | Must use small amplitude + start from current pos |
| Feedback | `sim.get_left_arm_positions()` | `state.position` from motor reply on CAN bus |
| Loop rate | As fast as MuJoCo can run | 100 Hz (0.01s sleep) |

---