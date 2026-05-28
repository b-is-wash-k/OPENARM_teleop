"""Single-motor oscillation demo with live MuJoCo viewer.

Oscillates ONE chosen motor while holding all others at their starting positions.
The MuJoCo viewer shows the actual hardware positions in real time.

Usage:
    python examples/hardware_single_motor_oscillation.py --port robot_l:left --motor 3

    # Custom amplitude and speed:
    python examples/hardware_single_motor_oscillation.py --port robot_l:left --motor 4 --amplitude 0.2 --frequency 0.003

Motor IDs:
    1 = shoulder rotation   5 = forearm rotation
    2 = shoulder flex       6 = wrist flex
    3 = upper arm rotation  7 = wrist rotation
    4 = elbow flex          8 = gripper

Close the viewer window (or Ctrl+C) to stop.

Safety:
    - Amplitude is automatically clamped so the motion stays within the
      motor's physical joint limits (from MOTOR_CONFIGS)
    - All other motors hold with PD control
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
LOOP_DT      = 0.01   # 100 Hz
NUM_ARM_JOINTS = 7


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
        description="Oscillate one motor — with live MuJoCo viewer"
    )
    parser.add_argument("--port",      required=True, help="CAN port:side  e.g. robot_l:left")
    parser.add_argument("--motor",     type=int, required=True, help="Motor ID to oscillate (1–8)")
    parser.add_argument("--amplitude", type=float, default=0.3, help="Swing in radians (default 0.3)")
    parser.add_argument("--frequency", type=float, default=0.005, help="Speed cycles/step (default 0.005)")
    return parser.parse_args()


# ── Shared state ──────────────────────────────────────────────────────────────

class SharedState:
    def __init__(self) -> None:
        self._lock   = threading.Lock()
        self._actual: list[float] = []
        self.stop    = False

    def set_actual(self, positions: list[float]) -> None:
        with self._lock:
            self._actual = list(positions)

    def get_actual(self) -> list[float]:
        with self._lock:
            return list(self._actual)


# ── CAN asyncio loop ──────────────────────────────────────────────────────────

async def can_main(port_spec: str, motor_id: int, amplitude: float,
                   frequency: float, shared: SharedState) -> None:
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
            print(f"Warning: motors not detected: {missing}")

        motors = [
            Motor(Bus(can_bus), slave_id=cfg.slave_id, master_id=cfg.master_id, motor_type=cfg.type)
            for cfg in MOTOR_CONFIGS if cfg.slave_id in detected
        ]
        if not motors:
            print("No motors found.")
            shared.stop = True
            return

        num = len(motors)
        osc_idx = motor_id - 1
        if not 0 <= osc_idx < num:
            print(f"Motor {motor_id} not available. Detected motors: 1–{num}")
            shared.stop = True
            return

        arm = Arm(motors)
        print(f"Enabling {num} motors on {port_name} ({arm_side} arm)...")
        states = await arm.enable()
        await arm.set_control_mode(ControlMode.MIT)

        initial = [s.position if s else 0.0 for s in states]
        shared.set_actual(initial)
        print(f"Initial positions (rad): {[round(p, 3) for p in initial]}")

        # Clamp amplitude to stay within physical joint limits
        lo, hi = joint_limits_rad(osc_idx, arm_side)
        max_amp_lo = initial[osc_idx] - lo
        max_amp_hi = hi - initial[osc_idx]
        safe_amplitude = min(amplitude, max_amp_lo, max_amp_hi)
        if safe_amplitude < amplitude:
            print(f"⚠ Amplitude clamped: {amplitude:.3f} → {safe_amplitude:.3f} rad "
                  f"(joint {motor_id} limit: [{lo:.2f}, {hi:.2f}])")

        print()
        print(f"Oscillating motor {motor_id} ({MOTOR_CONFIGS[osc_idx].name})")
        print(f"  Amplitude : {safe_amplitude:.3f} rad  (~{math.degrees(safe_amplitude):.1f}°)")
        print(f"  All other motors hold position.")
        print("Close the viewer window or press Ctrl+C to stop.")
        print()

        step = 0
        while not shared.stop:
            targets = list(initial)
            targets[osc_idx] = (
                initial[osc_idx]
                + safe_amplitude * math.sin(step * frequency * 2 * math.pi)
            )
            try:
                states = await arm.control_mit(kp=KP, kd=KD, q=targets, dq=0.0, tau=0.0)
                actual = [s.position if s else targets[i] for i, s in enumerate(states)]
                shared.set_actual(actual)
            except Exception as e:
                print(f"\n[CAN error] {e}")
                shared.stop = True
                break
            step += 1
            await asyncio.sleep(LOOP_DT)

        print("\nDisabling all motors...")
        await arm.disable()
        print("Done.")

    finally:
        can_bus.shutdown()


def can_thread_fn(port_spec: str, motor_id: int, amplitude: float,
                  frequency: float, shared: SharedState) -> None:
    asyncio.run(can_main(port_spec, motor_id, amplitude, frequency, shared))


# ── MuJoCo viewer (main thread) ───────────────────────────────────────────────

def run_viewer(shared: SharedState, side: str) -> None:
    sim      = OpenArmSimulation()
    has_left = (side == "left")

    # Wait until first hardware positions arrive
    for _ in range(50):
        if shared.get_actual():
            break
        time.sleep(0.1)

    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running() and not shared.stop:
            actual  = shared.get_actual()
            if actual:
                arm_pos = actual[:NUM_ARM_JOINTS]
                grip    = actual[NUM_ARM_JOINTS] if len(actual) > NUM_ARM_JOINTS else 0.0
                if has_left:
                    sim.set_left_arm_positions(arm_pos)
                    sim.set_left_gripper_position(grip)
                else:
                    sim.set_right_arm_positions(arm_pos)
                    sim.set_right_gripper_position(grip)
            viewer.sync()

    shared.stop = True


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    args  = parse_arguments()
    parts = args.port.split(":")
    if len(parts) != 2 or parts[1] not in ("left", "right"):
        print("Error: use format PORT:left or PORT:right")
        sys.exit(1)

    shared = SharedState()

    t = threading.Thread(
        target=can_thread_fn,
        args=(args.port, args.motor, args.amplitude, args.frequency, shared),
        daemon=True,
    )
    t.start()

    try:
        run_viewer(shared, parts[1])
    except KeyboardInterrupt:
        shared.stop = True

    t.join(timeout=3.0)


if __name__ == "__main__":
    main()
