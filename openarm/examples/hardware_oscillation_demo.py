"""Real hardware oscillation demo — mirrors the simulation_demo on the physical arm.

Oscillates joint 1 (shoulder rotation) with a small sinusoidal motion while
holding all other joints at their initial positions using MIT position control.
Supports one or both arms simultaneously.

Usage:
    # Single arm
    python examples/hardware_oscillation_demo.py --port robot_l:left

    # Both arms at once
    python examples/hardware_oscillation_demo.py --port robot_l:left --port robot_r:right

Safety:
    - Amplitude is limited to 0.3 rad (~17 degrees) from the starting position
    - All joints are held in place with PD control; only joint 1 oscillates
    - Press Q to stop cleanly (motors are disabled on exit)
    - Always hold the arm before running — it will resist motion while active
"""

import asyncio
import math
import select
import sys
import termios
import tty
import argparse

import can

from openarm.bus import Bus
from openarm.damiao import Arm, ControlMode, Motor, detect_motors
from openarm.damiao.config import MOTOR_CONFIGS


# ── Safety parameters ──────────────────────────────────────────────────────────
AMPLITUDE   = 0.3    # radians (~17 degrees) — how far joint 1 swings each way
FREQUENCY   = 0.005  # cycles per loop step — keep this slow
KP          = 20.0   # position gain for MIT control
KD          = 3.0    # damping gain for MIT control
LOOP_DT     = 0.01   # seconds per loop step (100 Hz)
# ───────────────────────────────────────────────────────────────────────────────


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Oscillate joint 1 on the real OpenArm hardware (one or both arms)"
    )
    parser.add_argument(
        "--port",
        action="append",
        required=True,
        help="CAN port:side pair. Repeat for both arms: --port robot_l:left --port robot_r:right",
    )
    return parser.parse_args()


def check_key() -> str | None:
    """Non-blocking keyboard read (Unix only)."""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1).lower()
    return None


async def setup_arm(port_spec: str) -> tuple[Arm, list[float], can.BusABC] | None:
    """Connect to one arm, enable motors, return (Arm, initial_positions, can_bus)."""
    parts = port_spec.split(":")
    if len(parts) != 2 or parts[1] not in ("left", "right"):
        print(f"Error: invalid port spec '{port_spec}'. Use format PORT:left or PORT:right")
        return None

    port_name, arm_side = parts

    all_configs = can.detect_available_configs("socketcan")
    bus_cfg = next((c for c in all_configs if port_name in str(c.get("channel", ""))), None)

    if bus_cfg is None:
        print(f"Error: CAN interface '{port_name}' not found.")
        print(f"  Run: sudo ip link set {port_name} up type can bitrate 1000000")
        return None

    can_bus = can.Bus(channel=bus_cfg["channel"], interface=bus_cfg["interface"])

    slave_ids = [cfg.slave_id for cfg in MOTOR_CONFIGS]
    detected = {info.slave_id for info in detect_motors(can_bus, slave_ids, timeout=0.1)}
    missing = [cfg.name for cfg in MOTOR_CONFIGS if cfg.slave_id not in detected]
    if missing:
        print(f"  [{port_name}] Warning: motors not detected: {missing}")

    motors = [
        Motor(Bus(can_bus), slave_id=cfg.slave_id, master_id=cfg.master_id, motor_type=cfg.type)
        for cfg in MOTOR_CONFIGS if cfg.slave_id in detected
    ]

    if not motors:
        print(f"  [{port_name}] No motors found.")
        can_bus.shutdown()
        return None

    arm = Arm(motors)
    print(f"  [{port_name}] Enabling {len(motors)} motors ({arm_side} arm)...")
    states = await arm.enable()
    await arm.set_control_mode(ControlMode.MIT)

    initial_positions = [s.position if s else 0.0 for s in states]
    print(f"  [{port_name}] Initial positions (rad): {[round(p, 3) for p in initial_positions]}")

    return arm, initial_positions, can_bus


async def oscillate_arm(arm: Arm, initial_positions: list[float], stop_event: asyncio.Event) -> None:
    """Oscillate joint 1 on one arm until stop_event is set."""
    step = 0
    while not stop_event.is_set():
        joint1_target = initial_positions[0] + AMPLITUDE * math.sin(
            step * FREQUENCY * 2 * math.pi
        )
        targets = list(initial_positions)
        targets[0] = joint1_target

        await arm.control_mit(kp=KP, kd=KD, q=targets, dq=0.0, tau=0.0)
        step += 1
        await asyncio.sleep(LOOP_DT)


async def keyboard_watcher(stop_event: asyncio.Event) -> None:
    """Watch for Q keypress and set stop_event."""
    while not stop_event.is_set():
        key = check_key()
        if key == "q":
            stop_event.set()
            break
        await asyncio.sleep(0.05)


async def run(port_specs: list[str]) -> None:
    """Set up all arms and run oscillation concurrently."""
    print("Setting up arms...")
    results = [await setup_arm(spec) for spec in port_specs]
    active = [(arm, pos, bus) for r in results if r is not None for arm, pos, bus in [r]]

    if not active:
        print("No arms ready. Exiting.")
        return

    print()
    print("Starting oscillation on joint 1 (shoulder rotation) — all active arms.")
    print(f"  Amplitude : {AMPLITUDE} rad  (~{math.degrees(AMPLITUDE):.1f} degrees)")
    print("  All other joints hold their starting positions.")
    print()
    print("Press  Q  to stop.")
    print()

    stop_event = asyncio.Event()

    old_settings = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    try:
        await asyncio.gather(
            keyboard_watcher(stop_event),
            *[oscillate_arm(arm, pos, stop_event) for arm, pos, _ in active],
        )
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    print("\r\nStopping — disabling all motors...")
    for arm, _, bus in active:
        await arm.disable()
        bus.shutdown()
    print("Done. All motors disabled.")


def main() -> None:
    args = parse_arguments()
    try:
        asyncio.run(run(args.port))
    except KeyboardInterrupt:
        print("\r\nInterrupted.")
        sys.exit(0)


if __name__ == "__main__":
    main()
