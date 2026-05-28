"""Hardware + Simulation mirror demo.

Runs the sinusoidal oscillation on the real robot AND simultaneously shows
the actual joint positions inside the MuJoCo viewer — so you can watch both
the physical arm and its virtual twin move together in real time.

How it works:
    - A background thread runs the asyncio CAN loop:
        * Sends MIT position commands to the hardware (oscillate joint 1)
        * Reads back the actual joint positions from each motor's reply
        * Writes those positions into a shared state object
    - The main thread runs the MuJoCo viewer loop:
        * Reads the shared state each frame
        * Sets those positions directly on the sim model
        * Calls viewer.sync() to update the display

Usage:
    # Left arm only
    python examples/hardware_sim_mirror.py --port robot_l:left

    # Both arms
    python examples/hardware_sim_mirror.py --port robot_l:left --port robot_r:right

Press Q in the terminal (not the viewer) to stop.
"""

import asyncio
import math
import select
import sys
import termios
import threading
import tty
import argparse

import can
import mujoco.viewer

from openarm.bus import Bus
from openarm.damiao import Arm, ControlMode, Motor, detect_motors
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.simulation import OpenArmSimulation


# ── Safety parameters ──────────────────────────────────────────────────────────
AMPLITUDE = 0.3    # radians (~17 degrees)
FREQUENCY = 0.005  # cycles per loop step
KP        = 20.0
KD        = 3.0
LOOP_DT   = 0.01   # 100 Hz CAN loop
# ───────────────────────────────────────────────────────────────────────────────

NUM_ARM_JOINTS = 7  # joints 1–7 (sim expects 7; gripper is separate)


class SharedState:
    """Thread-safe container for live joint positions from hardware."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.left_positions:  list[float] = [0.0] * NUM_ARM_JOINTS
        self.right_positions: list[float] = [0.0] * NUM_ARM_JOINTS
        self.left_gripper:  float = 0.0
        self.right_gripper: float = 0.0
        self.stop: bool = False  # set True to signal shutdown

    def write_arm(self, side: str, positions: list[float]) -> None:
        with self._lock:
            if side == "left":
                self.left_positions  = positions[:NUM_ARM_JOINTS]
                self.left_gripper    = positions[NUM_ARM_JOINTS] if len(positions) > NUM_ARM_JOINTS else 0.0
            else:
                self.right_positions = positions[:NUM_ARM_JOINTS]
                self.right_gripper   = positions[NUM_ARM_JOINTS] if len(positions) > NUM_ARM_JOINTS else 0.0

    def read(self) -> tuple[list[float], list[float], float, float]:
        with self._lock:
            return (
                list(self.left_positions),
                list(self.right_positions),
                self.left_gripper,
                self.right_gripper,
            )


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Mirror real OpenArm joint positions into MuJoCo viewer"
    )
    parser.add_argument(
        "--port",
        action="append",
        required=True,
        help="CAN port:side, e.g. --port robot_l:left --port robot_r:right",
    )
    return parser.parse_args()


def check_key() -> str | None:
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return None


# ── CAN / asyncio side (runs in background thread) ────────────────────────────

async def setup_arm(port_spec: str) -> tuple[Arm, list[float], str, can.BusABC] | None:
    """Connect, enable, return (arm, initial_positions, side, can_bus)."""
    parts = port_spec.split(":")
    if len(parts) != 2 or parts[1] not in ("left", "right"):
        print(f"Invalid port spec '{port_spec}'. Use PORT:left or PORT:right")
        return None

    port_name, side = parts
    all_cfgs = can.detect_available_configs("socketcan")
    bus_cfg = next((c for c in all_cfgs if port_name in str(c.get("channel", ""))), None)

    if bus_cfg is None:
        print(f"CAN interface '{port_name}' not found.")
        print(f"  Run: sudo ip link set {port_name} up type can bitrate 1000000")
        return None

    can_bus = can.Bus(channel=bus_cfg["channel"], interface=bus_cfg["interface"])
    slave_ids = [cfg.slave_id for cfg in MOTOR_CONFIGS]
    detected  = {info.slave_id for info in detect_motors(can_bus, slave_ids, timeout=0.1)}
    missing   = [cfg.name for cfg in MOTOR_CONFIGS if cfg.slave_id not in detected]
    if missing:
        print(f"[{port_name}] Warning: motors not detected: {missing}")

    motors = [
        Motor(Bus(can_bus), slave_id=cfg.slave_id, master_id=cfg.master_id, motor_type=cfg.type)
        for cfg in MOTOR_CONFIGS if cfg.slave_id in detected
    ]
    if not motors:
        print(f"[{port_name}] No motors found.")
        can_bus.shutdown()
        return None

    arm = Arm(motors)
    print(f"[{port_name}] Enabling {len(motors)} motors ({side} arm)...")
    states = await arm.enable()
    await arm.set_control_mode(ControlMode.MIT)

    initial = [s.position if s else 0.0 for s in states]
    print(f"[{port_name}] Initial positions: {[round(p, 3) for p in initial]}")
    return arm, initial, side, can_bus


async def arm_loop(
    arm: Arm,
    initial: list[float],
    side: str,
    shared: SharedState,
) -> None:
    """Oscillate joint 1 and write actual positions into shared state."""
    step = 0
    while not shared.stop:
        joint1_target = initial[0] + AMPLITUDE * math.sin(step * FREQUENCY * 2 * math.pi)
        targets = list(initial)
        targets[0] = joint1_target

        states = await arm.control_mit(kp=KP, kd=KD, q=targets, dq=0.0, tau=0.0)

        # Write actual hardware positions into shared state for the viewer
        actual = [s.position if s else targets[i] for i, s in enumerate(states)]
        shared.write_arm(side, actual)

        step += 1
        await asyncio.sleep(LOOP_DT)


async def keyboard_watcher(shared: SharedState) -> None:
    while not shared.stop:
        if check_key() == "q":
            shared.stop = True
            break
        await asyncio.sleep(0.05)


async def can_main(port_specs: list[str], shared: SharedState) -> None:
    results = [await setup_arm(spec) for spec in port_specs]
    active  = [r for r in results if r is not None]

    if not active:
        print("No arms ready.")
        shared.stop = True
        return

    # Pre-fill shared state with initial positions so viewer starts correctly
    for arm, initial, side, _ in active:
        shared.write_arm(side, initial)

    try:
        await asyncio.gather(
            keyboard_watcher(shared),
            *[arm_loop(arm, init, side, shared) for arm, init, side, _ in active],
        )
    finally:
        for arm, _, _, bus in active:
            await arm.disable()
            bus.shutdown()


def can_thread(port_specs: list[str], shared: SharedState) -> None:
    asyncio.run(can_main(port_specs, shared))


# ── MuJoCo viewer side (runs in main thread) ──────────────────────────────────

def run_viewer(shared: SharedState, has_left: bool, has_right: bool) -> None:
    sim = OpenArmSimulation()

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running() and not shared.stop:
            left, right, left_grip, right_grip = shared.read()

            if has_left:
                sim.set_left_arm_positions(left)
                sim.set_left_gripper_position(left_grip)
            if has_right:
                sim.set_right_arm_positions(right)
                sim.set_right_gripper_position(right_grip)

            viewer.sync()

    # If user closes viewer, also stop the CAN thread
    shared.stop = True


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    args = parse_arguments()

    sides = set()
    for spec in args.port:
        parts = spec.split(":")
        if len(parts) == 2:
            sides.add(parts[1])

    has_left  = "left"  in sides
    has_right = "right" in sides

    shared = SharedState()

    # Start CAN loop in background thread
    t = threading.Thread(target=can_thread, args=(args.port, shared), daemon=True)
    t.start()

    print()
    print("MuJoCo viewer opening — real arm positions will appear there.")
    print("Press  Q  in this terminal to stop.")
    print()

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        run_viewer(shared, has_left, has_right)
    except KeyboardInterrupt:
        shared.stop = True
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    t.join(timeout=3.0)
    print("\r\nDone.")


if __name__ == "__main__":
    main()
