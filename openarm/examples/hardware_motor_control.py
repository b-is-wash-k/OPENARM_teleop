"""Interactive motor position controller with live MuJoCo viewer.

Enables all motors, shows their positions in the MuJoCo simulation viewer,
and lets you type a motor number + target position to move any single joint.

Usage:
    python examples/hardware_motor_control.py --port robot_l:left
    python examples/hardware_motor_control.py --port robot_r:right

In the interactive prompt:
    > 1 0.5        move motor 1 to 0.5 rad
    > 3 -0.3       move motor 3 to -0.3 rad
    > 1 0          move motor 1 back to 0
    > q            quit (disables all motors)

Safety:
    - Targets are clamped to the motor's ACTUAL physical joint limits (from config)
    - All motors hold with PD control at all times
    - The MuJoCo viewer shows live actual positions from hardware
    - Close the viewer window OR type q to stop
"""

import asyncio
import math
import sys
import argparse
import threading

import can
import mujoco.viewer

from openarm.bus import Bus
from openarm.damiao import Arm, ControlMode, Motor, detect_motors
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.simulation import OpenArmSimulation


KP           = 20.0
KD           = 3.0
LOOP_DT      = 0.02   # 50 Hz hold loop
NUM_ARM_JOINTS = 7    # joints 1-7 for sim arm (8th is gripper)


def joint_limits_rad(motor_idx: int, side: str) -> tuple[float, float]:
    """Return (min_rad, max_rad) for a motor from MOTOR_CONFIGS."""
    cfg = MOTOR_CONFIGS[motor_idx]
    if side == "left":
        lo = math.radians(cfg.min_angle_left)
        hi = math.radians(cfg.max_angle_left)
    else:
        lo = math.radians(cfg.min_angle_right)
        hi = math.radians(cfg.max_angle_right)
    # ensure lo <= hi
    return (min(lo, hi), max(lo, hi))


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Interactively move any motor — with live MuJoCo viewer"
    )
    parser.add_argument("--port", required=True, help="CAN port:side  e.g. robot_l:left")
    return parser.parse_args()


# ── Shared state (CAN thread ↔ viewer thread) ─────────────────────────────────

class SharedState:
    def __init__(self, initial: list[float]) -> None:
        self._lock = threading.Lock()
        self._targets  = list(initial)   # what we want to send to hardware
        self._actual   = list(initial)   # what hardware actually reports back
        self.stop      = False

    def set_target(self, idx: int, pos: float) -> None:
        with self._lock:
            self._targets[idx] = pos

    def get_targets(self) -> list[float]:
        with self._lock:
            return list(self._targets)

    def set_actual(self, positions: list[float]) -> None:
        with self._lock:
            self._actual = list(positions)

    def get_actual(self) -> list[float]:
        with self._lock:
            return list(self._actual)


# ── Input thread ──────────────────────────────────────────────────────────────

def input_loop(shared: SharedState, initial: list[float], num_motors: int,
               limits: list[tuple[float, float]]) -> None:
    print()
    print("─" * 52)
    print("Interactive motor control ready.")
    print("  Type:  <motor_id> <position_rad>   then Enter")
    print("  Type:  q                            to quit")
    print(f"  Motors available: 1 – {num_motors}")
    print()
    print("  Motor  Name    Range (rad)")
    for i in range(num_motors):
        lo, hi = limits[i]
        print(f"    {i+1}     {MOTOR_CONFIGS[i].name}    [{lo:.2f}, {hi:.2f}]")
    print("─" * 52)
    print()

    while not shared.stop:
        try:
            line = input("> ").strip()
        except EOFError:
            shared.stop = True
            break

        if line.lower() == "q":
            shared.stop = True
            break

        parts = line.split()
        if len(parts) != 2:
            print("  Usage: <motor_id> <position_rad>  e.g.  3 -0.5")
            continue

        try:
            motor_id = int(parts[0])
            position = float(parts[1])
        except ValueError:
            print("  Both values must be numbers.  e.g.  2 0.8")
            continue

        if not 1 <= motor_id <= num_motors:
            print(f"  Motor ID must be between 1 and {num_motors}.")
            continue

        idx = motor_id - 1
        lo, hi = limits[idx]
        clamped = max(lo, min(hi, position))
        if abs(clamped - position) > 1e-6:
            print(f"  ⚠ Clamped {position:.3f} → {clamped:.3f} rad  "
                  f"(joint limit: [{lo:.2f}, {hi:.2f}])")

        shared.set_target(idx, clamped)
        print(f"  → Motor {motor_id} ({MOTOR_CONFIGS[idx].name}): {clamped:.3f} rad")


# ── CAN asyncio loop ──────────────────────────────────────────────────────────

async def hold_loop(arm: Arm, shared: SharedState) -> None:
    while not shared.stop:
        targets = shared.get_targets()
        try:
            states = await arm.control_mit(kp=KP, kd=KD, q=targets, dq=0.0, tau=0.0)
            actual = [s.position if s else targets[i] for i, s in enumerate(states)]
            shared.set_actual(actual)
        except Exception as e:
            print(f"\n[CAN error] {e}")
            shared.stop = True
            break
        await asyncio.sleep(LOOP_DT)


async def can_main(port_spec: str, shared_ref: list) -> None:
    parts = port_spec.split(":")
    port_name, arm_side = parts[0], parts[1]

    all_cfgs = can.detect_available_configs("socketcan")
    bus_cfg  = next((c for c in all_cfgs if port_name in str(c.get("channel", ""))), None)
    if bus_cfg is None:
        print(f"CAN interface '{port_name}' not found.")
        print(f"  Run: sudo ip link set {port_name} up type can bitrate 1000000")
        return

    can_bus = can.Bus(channel=bus_cfg["channel"], interface=bus_cfg["interface"])
    try:
        slave_ids = [cfg.slave_id for cfg in MOTOR_CONFIGS]
        detected  = {info.slave_id for info in detect_motors(can_bus, slave_ids, timeout=0.1)}
        missing   = [cfg.name for cfg in MOTOR_CONFIGS if cfg.slave_id not in detected]
        if missing:
            print(f"Warning: motors not detected: {missing}")

        motors = [
            Motor(Bus(can_bus), slave_id=cfg.slave_id, master_id=cfg.master_id, motor_type=cfg.type)
            for cfg in MOTOR_CONFIGS if cfg.slave_id in detected
        ]
        if not motors:
            print("No motors found.")
            return

        arm  = Arm(motors)
        num  = len(motors)
        print(f"Enabling {num} motors on {port_name} ({arm_side} arm)...")
        states = await arm.enable()
        await arm.set_control_mode(ControlMode.MIT)

        initial = [s.position if s else 0.0 for s in states]
        print(f"Initial positions (rad): {[round(p, 3) for p in initial]}")

        limits = [joint_limits_rad(i, arm_side) for i in range(num)]

        shared = SharedState(initial)
        shared_ref.append(shared)   # pass back to main thread

        # Start input thread
        input_thread = threading.Thread(
            target=input_loop,
            args=(shared, initial, num, limits),
            daemon=True,
        )
        input_thread.start()

        await hold_loop(arm, shared)

        input_thread.join(timeout=1.0)
        print("\nDisabling all motors...")
        await arm.disable()
        print("Done.")

    finally:
        can_bus.shutdown()


def can_thread_fn(port_spec: str, shared_ref: list) -> None:
    asyncio.run(can_main(port_spec, shared_ref))


# ── MuJoCo viewer (main thread) ───────────────────────────────────────────────

def run_viewer(shared: SharedState, side: str) -> None:
    sim = OpenArmSimulation()
    has_left  = (side == "left")
    has_right = (side == "right")

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running() and not shared.stop:
            actual = shared.get_actual()
            arm_pos = actual[:NUM_ARM_JOINTS]
            grip    = actual[NUM_ARM_JOINTS] if len(actual) > NUM_ARM_JOINTS else 0.0

            if has_left:
                sim.set_left_arm_positions(arm_pos)
                sim.set_left_gripper_position(grip)
            else:
                sim.set_right_arm_positions(arm_pos)
                sim.set_right_gripper_position(grip)

            viewer.sync()

    shared.stop = True  # closing the viewer also stops everything


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    args = parse_arguments()
    parts = args.port.split(":")
    if len(parts) != 2 or parts[1] not in ("left", "right"):
        print("Error: use format PORT:left or PORT:right  e.g. robot_l:left")
        sys.exit(1)

    shared_ref: list = []

    t = threading.Thread(target=can_thread_fn, args=(args.port, shared_ref), daemon=True)
    t.start()

    # Wait until the CAN thread has built the SharedState object
    import time
    for _ in range(50):
        if shared_ref:
            break
        time.sleep(0.1)

    if not shared_ref:
        print("Could not connect to arm. Exiting.")
        sys.exit(1)

    try:
        run_viewer(shared_ref[0], parts[1])
    except KeyboardInterrupt:
        if shared_ref:
            shared_ref[0].stop = True

    t.join(timeout=3.0)


if __name__ == "__main__":
    main()
