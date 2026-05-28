#!/usr/bin/env python3
"""Test custom Jacobian-based IK solver."""

import sys
sys.path.insert(0, '/home/claude')

import numpy as np
from openarm_jacobian_ik import OpenArmIKWrapper

print("=== Testing Custom Jacobian IK ===\n")

# Initialize wrapper
ik_wrapper = OpenArmIKWrapper()

# Test 1: From arms down (J2=0° physical = J2=-90° URDF)
print("=== Test 1: Right Arm from Arms Down ===")
current_physical = np.array([0, 0, 0, 0, 0, 0, 0])  # Arms down
print(f"Starting position (physical): {current_physical}")

# Convert to URDF to get current Cartesian position
urdf_joints = ik_wrapper.physical_to_urdf_right(current_physical)
print(f"Starting position (URDF rad): {urdf_joints}")

# FK to find current position
config_full = np.zeros(12)
config_full[1:8] = urdf_joints
fk_result = ik_wrapper.ik_solver._right_chain.forward_kinematics(config_full)
current_pos = fk_result[:3, 3]

print(f"Current end-effector: [{current_pos[0]:.3f}, {current_pos[1]:.3f}, {current_pos[2]:.3f}]")

# Target: 10cm up
target_pos = current_pos + np.array([0, 0, 0.10])
print(f"Target position:      [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
print("\nSolving IK...")

# Solve!
solution_physical = ik_wrapper.solve_right_arm_physical(
    target_pos,
    current_physical,
    max_iterations=100,
    position_tolerance=0.001,
    step_size=0.3,
)

print(f"\nSolution (physical degrees): {solution_physical}")

# Verify
urdf_solution = ik_wrapper.physical_to_urdf_right(solution_physical)
verify_full = np.zeros(12)
verify_full[1:8] = urdf_solution
fk_verify = ik_wrapper.ik_solver._right_chain.forward_kinematics(verify_full)
verify_pos = fk_verify[:3, 3]
error = np.linalg.norm(verify_pos - target_pos)

print(f"\nVerification:")
print(f"  Expected: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
print(f"  Got:      [{verify_pos[0]:.3f}, {verify_pos[1]:.3f}, {verify_pos[2]:.3f}]")
print(f"  Error:    {error*1000:.2f}mm")

if error < 0.010:  # 10mm tolerance
    print("  ✓ SUCCESS!")
else:
    print("  ✗ FAILED - large error")

# Test 2: Multiple movements
print("\n\n=== Test 2: Multiple Sequential Movements ===")

movements = [
    ("5cm forward", np.array([0.05, 0, 0])),
    ("5cm right", np.array([0, -0.05, 0])),
    ("5cm down", np.array([0, 0, -0.05])),
    ("Return to start", -np.array([0.05, -0.05, -0.05])),
]

current_physical = np.array([0, 0, 0, 0, 0, 0, 0])

for name, offset in movements:
    # Get current position
    urdf_joints = ik_wrapper.physical_to_urdf_right(current_physical)
    config_full = np.zeros(12)
    config_full[1:8] = urdf_joints
    fk_result = ik_wrapper.ik_solver._right_chain.forward_kinematics(config_full)
    current_pos = fk_result[:3, 3]
    
    # Target
    target_pos = current_pos + offset
    
    print(f"\n{name}:")
    print(f"  Target: [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
    
    # Solve
    solution_physical = ik_wrapper.solve_right_arm_physical(
        target_pos,
        current_physical,
        max_iterations=50,
        position_tolerance=0.005,
        step_size=0.3,
    )
    
    # Verify
    urdf_solution = ik_wrapper.physical_to_urdf_right(solution_physical)
    verify_full = np.zeros(12)
    verify_full[1:8] = urdf_solution
    fk_verify = ik_wrapper.ik_solver._right_chain.forward_kinematics(verify_full)
    verify_pos = fk_verify[:3, 3]
    error = np.linalg.norm(verify_pos - target_pos)
    
    print(f"  Error: {error*1000:.2f}mm", end="")
    if error < 0.010:
        print(" ✓")
        current_physical = solution_physical  # Update for next movement
    else:
        print(" ✗")

print("\n=== Test Complete! ===")
print("\nIf all tests passed, the custom IK solver works!")
print("Next step: Integrate into robot control script")