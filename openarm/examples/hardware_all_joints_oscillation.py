"""All-joints oscillation demo with live MuJoCo viewer.

Every motor oscillates simultaneously with a 45° phase offset between each joint,
creating a ripple wave from shoulder to gripper.
The MuJoCo viewer shows the actual hardware positions in real time.

Usage:
    python examples/hardware_all_joints_oscillation.py --port robot_l:left
    python examples/hardware_all_joints_oscillation.py --port robot_l:left --port robot_r:right
    python examples/hardware_all_joints_oscillation.py --port robot_l:left --frequency 0.008

Close the viewer window (or Ctrl+C) to stop.

Safety:
    - Per-joint amplitudes are automatically clamped to physical joint limits
    - Gripper (J8) has a very small amplitude (0.05 rad) by default
"""

import argparse
import asyncio
import math
import sys
import threading
import time

import can
import mujoco.viewer

from openarm.bus import Bus
from openarm.damiao import Arm, ControlMode, Motor, detect_motors
from openarm.damiao.config import MOTOR_CONFIGS
from openarm.simulation import OpenArmSimulation


KP           = 20.0
KD           = 3.0
LOOP_DT      = 0.01
NUM_ARM_JOINTS = 7

# Per-joint desired amplitudes (radians) — conservative for safety
AMPLITUDES = [0.30, 0.20, 0.20, 0.15, 0.20, 0.15, 0.15, 0.05]

# 45° phase offset between joints → ripple wave through the arm
PHASES = [i * math.pi / 4 for i in range(8)]


def joint_limits_rad(motor_idx: int, side: str) -> tuple[float, float]:
    cfg = MOTOR_CONFIGS[motor_idx]
    if side == "left":
        lo, hi = cfg.min_angle_left, cfg.max_angle_left
    else:
        lo, hi = cfg.min_angle_right, cfg.max_angle_right
    lo_r, hi_r = math.radians(lo), math.radians(hi)
    return (min(lo_r, hi_r), max(lo_r, hi_r))


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Oscillate ALL joints as a wave — with live MuJoCo viewer"
    )
    parser.add_argument("--port",      action="append", required=True,
                        help="CAN port:side  e.g. --port robot_l:left --port robot_r:right")
    parser.add_argument("--frequency", type=float, default=0.005,
                        help="Oscillation speed in cycles/step (default 0.005)")
    return parser.parse_args()


# ── Shared state ──────────────────────────────────────────────────────────────

class SharedState:
    def __init__(self) -> None:
        self._lock  = threading.Lock()
        self._left:  list[float] = [0.0] * 8
        self._right: list[float] = [0.0] * 8
        self.stop   = False

    def set(self, side: str, positions: list[float]) -> None:
        with self._lock:
            if side == "left":
                self._left  = list(positions)
            else:
                self._right = list(positions)

    def get(self) -> tuple[list[float], list[float]]:
        with self._lock:
            return list(self._left), list(self._right)


# ── CAN asyncio loop ──────────────────────────────────────────────────────────

async def arm_wave(port_spec: str, frequency: float, shared: SharedState) -> None:
    parts     = port_spec.split(":")
    port_name = parts[0]
    arm_side  = parts[1]

    all_cfgs = can.detect_available_configs("socketcan")
    bus_cfg  = next((c for c in all_cfgs if port_name in str(c.get("channel", ""))), None)
    if bus_cfg is None:
        print(f"CAN interface '{port_name}' not found.")
        print(f"  Run: sudo ip link set {port_name} up type can bitrate 1000000")
        shared.stop = True
        return

    can_bus = can.Bus(channel=bus_cfg["channel"], interface=bus_cfg["interface"])
    try:
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
            return

        n   = len(motors)
        arm = Arm(motors)
        print(f"[{port_name}] Enabling {n} motors ({arm_side} arm)...")
        states = await arm.enable()
        await arm.set_control_mode(ControlMode.MIT)

        initial = [s.position if s else 0.0 for s in states]
        shared.set(arm_side, initial)
        print(f"[{port_name}] Initial positions: {[round(p, 3) for p in initial]}")

        # Compute safe per-joint amplitudes (clamped to physical limits)
        safe_amps = []
        for i in range(n):
            lo, hi = joint_limits_rad(i, arm_side)
            desired = AMPLITUDES[i] if i < len(AMPLITUDES) else 0.1
            max_down = initial[i] - lo
            max_up   = hi - initial[i]
            safe     = min(desired, max_down, max_up)
            safe_amps.append(max(safe, 0.0))
            if safe < desired:
                print(f"[{port_name}] J{i+1} amplitude clamped: {desired:.2f} → {safe:.2f} rad")

        phases = PHASES[:n]

        print()
        print(f"[{port_name}] Wave oscillation — all {n} joints moving:")
        for i in range(n):
            print(f"   J{i+1} ({MOTOR_CONFIGS[i].name})"
                  f"  ±{safe_amps[i]:.2f} rad  phase {math.degrees(phases[i]):.0f}°")
        print()

        step = 0
        while not shared.stop:
            targets = [
                initial[i] + safe_amps[i] * math.sin(step * frequency * 2 * math.pi + phases[i])
                for i in range(n)
            ]
            try:
                states = await arm.control_mit(kp=KP, kd=KD, q=targets, dq=0.0, tau=0.0)
                actual = [s.position if s else targets[i] for i, s in enumerate(states)]
                shared.set(arm_side, actual)
            except Exception as e:
                print(f"\n[{port_name}] CAN error: {e}")
                shared.stop = True
                break
            step += 1
            await asyncio.sleep(LOOP_DT)

        print(f"\n[{port_name}] Disabling motors...")
        await arm.disable()

    finally:
        can_bus.shutdown()


async def can_main(port_specs: list[str], frequency: float, shared: SharedState) -> None:
    await asyncio.gather(*[arm_wave(spec, frequency, shared) for spec in port_specs])


def can_thread_fn(port_specs: list[str], frequency: float, shared: SharedState) -> None:
    asyncio.run(can_main(port_specs, frequency, shared))


# ── MuJoCo viewer (main thread) ───────────────────────────────────────────────

def run_viewer(shared: SharedState, has_left: bool, has_right: bool) -> None:
    sim = OpenArmSimulation()

    # Wait for first hardware positions
    for _ in range(50):
        left, right = shared.get()
        if any(left) or any(right):
            break
        time.sleep(0.1)

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running() and not shared.stop:
            left, right = shared.get()

            if has_left:
                sim.set_left_arm_positions(left[:NUM_ARM_JOINTS])
                sim.set_left_gripper_position(left[NUM_ARM_JOINTS] if len(left) > NUM_ARM_JOINTS else 0.0)
            if has_right:
                sim.set_right_arm_positions(right[:NUM_ARM_JOINTS])
                sim.set_right_gripper_position(right[NUM_ARM_JOINTS] if len(right) > NUM_ARM_JOINTS else 0.0)

            viewer.sync()

    shared.stop = True


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    args = parse_arguments()

    sides = set()
    for spec in args.port:
        p = spec.split(":")
        if len(p) != 2 or p[1] not in ("left", "right"):
            print(f"Invalid port spec '{spec}'. Use PORT:left or PORT:right")
            sys.exit(1)
        sides.add(p[1])

    shared = SharedState()

    t = threading.Thread(
        target=can_thread_fn,
        args=(args.port, args.frequency, shared),
        daemon=True,
    )
    t.start()

    print("Close the viewer window or press Ctrl+C to stop.")

    try:
        run_viewer(shared, "left" in sides, "right" in sides)
    except KeyboardInterrupt:
        shared.stop = True

    t.join(timeout=3.0)


if __name__ == "__main__":
    main()
