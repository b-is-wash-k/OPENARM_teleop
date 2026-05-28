"""OpenArm simulation demonstration script.

This example demonstrates basic usage of the OpenArm simulation environment
by creating a simple oscillating motion on the first joint of both arms
using position control while visualizing the result in MuJoCo's viewer.
"""
import argparse

import math

import mujoco.viewer

from openarm.simulation import OpenArmSimulation
from openarm.simulation.models import OPENARM_SCENE_PATH

parser = argparse.ArgumentParser(description="OpenArm simulation demo.")
parser.add_argument("--xml", type=str, default=str(OPENARM_SCENE_PATH),
                    help="Path to MuJoCo XML scene")
args = parser.parse_args()

# Create simulation instance with OpenArm scene (floor, lighting, sky)
sim = OpenArmSimulation(model_path=args.xml)

with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
    step_count = 0
    while viewer.is_running():
        # Generate sinusoidal motion for demonstration
        amplitude = 0.5  # Joint angle amplitude in radians
        frequency = 0.01  # Oscillation frequency (cycles per step)
        first_joint_angle = amplitude * math.sin(step_count * frequency)

        # Read current joint positions for both arms
        left_positions = sim.get_left_arm_positions()
        right_positions = sim.get_right_arm_positions()

        # Apply sinusoidal motion to the first joint of each arm
        left_positions[0] = first_joint_angle
        right_positions[0] = first_joint_angle

        # Apply position control to both arms
        sim.set_left_arm_position_control(left_positions)
        sim.set_right_arm_position_control(right_positions)

        # Advance physics simulation and update viewer
        sim.step()
        viewer.sync()
        step_count += 1
