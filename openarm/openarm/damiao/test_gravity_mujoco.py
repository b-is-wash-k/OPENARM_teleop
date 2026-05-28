#!/usr/bin/env python3
"""
MuJoCo-only gravity compensation debugger for OpenArm.

What this script does:
- Loads the same OpenArm MuJoCo model used by gravity.py
- Prints joint names and indices
- Lets you test left or right arm gravity torques without touching hardware
- Opens a MuJoCo viewer so you can see which joints are actually moving
- Lets you optionally compare left/right torques

Run examples:
    python test_gravity_mujoco.py --side left
    python test_gravity_mujoco.py --side right
    python test_gravity_mujoco.py --side left --q 0 0.3 0 1.0 0 0.2 0 0
    python test_gravity_mujoco.py --compare
"""

from __future__ import annotations

import argparse
import time

import mujoco
import mujoco.viewer
import numpy as np

from openarm.simulation.models import OPENARM_MODEL_PATH


class MuJoCoKDL:
    """Same MuJoCo inverse-dynamics helper idea as gravity.py."""

    def __init__(self) -> None:
        self.model = mujoco.MjModel.from_xml_path(str(OPENARM_MODEL_PATH))
        self.model.opt.gravity = np.array([0.0, 0.0, -9.81], dtype=float)

        self.data = mujoco.MjData(self.model)

        # Match gravity.py behavior
        self.model.geom_contype[:] = 0
        self.model.geom_conaffinity[:] = 0
        self.model.jnt_limited[:] = 0

    def joint_slice(self, side: str, length: int) -> slice:
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'")
        # Matches current gravity.py exactly
        return slice(0, length) if side == "left" else slice(9, 9 + length)

    def compute_inverse_dynamics(
        self,
        q: np.ndarray,
        qdot: np.ndarray,
        qdotdot: np.ndarray,
        side: str = "left",
    ) -> np.ndarray:
        if not (len(q) == len(qdot) == len(qdotdot)):
            raise ValueError("q, qdot, qdotdot must have same length")

        joint_indices = self.joint_slice(side, len(q))

        self.data.qpos[:] = 0
        self.data.qvel[:] = 0
        self.data.qacc[:] = 0

        self.data.qpos[joint_indices] = q
        self.data.qvel[joint_indices] = qdot
        self.data.qacc[joint_indices] = qdotdot

        mujoco.mj_inverse(self.model, self.data)
        return self.data.qfrc_inverse[joint_indices].copy()


class GravityCompensator:
    """Same structure as gravity.py, but MuJoCo-only."""

    def __init__(self) -> None:
        self.kdl = MuJoCoKDL()
        self.tuning_factors = [0.8, 0.8, 1.0, 1.0, 1.0, 1.0, 1.0, 0.0]

    def compute(self, angles: list[float], position: str = "left") -> list[float]:
        q = np.array(angles, dtype=float)
        gravity_torques = self.kdl.compute_inverse_dynamics(
            q=q,
            qdot=np.zeros_like(q),
            qdotdot=np.zeros_like(q),
            side=position,
        )

        return [
            torque * factor
            for torque, factor in zip(
                gravity_torques, self.tuning_factors, strict=False
            )
        ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="MuJoCo-only gravity compensation debugger"
    )
    parser.add_argument(
        "--side",
        choices=["left", "right"],
        default="left",
        help="Which arm side to test",
    )
    parser.add_argument(
        "--q",
        nargs=8,
        type=float,
        default=[0.0, 0.3, 0.0, 1.0, 0.0, 0.2, 0.0, 0.0],
        metavar=("Q1", "Q2", "Q3", "Q4", "Q5", "Q6", "Q7", "Q8"),
        help="8 joint angles in radians",
    )
    parser.add_argument(
        "--compare",
        action="store_true",
        help="Print both left and right torques for the same q",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Viewer update rate",
    )
    return parser.parse_args()


def print_joint_info(model: mujoco.MjModel) -> None:
    print("\n=== Joint list from MuJoCo model ===")
    print(f"nq={model.nq}, nv={model.nv}, njnt={model.njnt}")
    for j in range(model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        # qpos and dof addresses help debug indexing
        qpos_adr = model.jnt_qposadr[j]
        dof_adr = model.jnt_dofadr[j]
        print(f"joint[{j:2d}] name={name!r:25s} qpos_adr={qpos_adr:2d} dof_adr={dof_adr:2d}")
    print()


def print_body_info(model: mujoco.MjModel) -> None:
    print("=== Body list from MuJoCo model ===")
    for b in range(model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, b)
        print(f"body[{b:2d}] name={name!r}")
    print()


def set_side_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    side: str,
    q: np.ndarray,
) -> None:
    """Set only the chosen side's joints, leave everything else at zero."""
    data.qpos[:] = 0
    data.qvel[:] = 0
    data.qacc[:] = 0

    joint_slice = slice(0, len(q)) if side == "left" else slice(9, 9 + len(q))
    data.qpos[joint_slice] = q

    mujoco.mj_forward(model, data)


def main() -> None:
    args = parse_args()
    q = np.array(args.q, dtype=float)

    gc = GravityCompensator()
    model = gc.kdl.model
    data = gc.kdl.data

    print_joint_info(model)
    print_body_info(model)

    print(f"Model path: {OPENARM_MODEL_PATH}")
    print(f"Testing side: {args.side}")
    print(f"Input q: {q.tolist()}")

    tau = gc.compute(q.tolist(), position=args.side)
    print(f"\nGravity torques for {args.side}:")
    for i, t in enumerate(tau):
        print(f"  joint {i+1}: {t:+.6f} Nm")

    if args.compare:
        tau_left = gc.compute(q.tolist(), position="left")
        tau_right = gc.compute(q.tolist(), position="right")
        print("\nComparison for same q on both sides:")
        for i, (tl, tr) in enumerate(zip(tau_left, tau_right, strict=False)):
            print(
                f"  joint {i+1}: left={tl:+.6f} Nm   right={tr:+.6f} Nm"
            )

    print(
        "\nViewer controls:\n"
        "  - close window to exit\n"
        "  - this is visualization only; no hardware commands are sent\n"
    )

    set_side_pose(model, data, args.side, q)

    dt = 1.0 / max(args.fps, 1.0)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last_print = 0.0

        while viewer.is_running():
            # Keep the chosen pose fixed
            set_side_pose(model, data, args.side, q)

            now = time.time()
            if now - last_print > 1.0:
                tau_live = gc.compute(q.tolist(), position=args.side)
                print(f"[live] {args.side} tau = {[round(x, 6) for x in tau_live]}")
                last_print = now

            viewer.sync()
            time.sleep(dt)


if __name__ == "__main__":
    main()