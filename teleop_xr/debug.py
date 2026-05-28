#!/usr/bin/env python3
"""
COMPREHENSIVE DEBUG SCRIPT FOR LEFT_JOINT4 LOCK ISSUE
Fixed version for teleop_xr architecture
"""

import sys
import os
import traceback

# ============================================================================
# STEP 1: Find robot class correctly
# ============================================================================

print("=" * 80)
print("STEP 1: Finding OpenArm / Default robot class definition")
print("=" * 80)

try:
    from teleop_xr.ik.loader import load_robot_class
    from teleop_xr.ik_utils import list_available_robots

    robots = list_available_robots()
    print(f"\nAvailable robots: {list(robots.keys())}")

    # Try OpenArm first if it exists, otherwise fallback
    if "openarm" in robots:
        print(f"✓ Found OpenArm entry point: {robots['openarm']}")
        robot_cls = load_robot_class("openarm")
    else:
        print("⚠ OpenArm NOT found in entry points")
        print("→ Falling back to default robot (None)")
        robot_cls = load_robot_class(None)

    print(f"✓ Loaded class: {robot_cls.__name__}")
    print(f"✓ Module file: {sys.modules[robot_cls.__module__].__file__}")

except Exception as e:
    print(f"✗ Error in step 1: {e}")
    traceback.print_exc()
    sys.exit(1)

# ============================================================================
# STEP 2: Instantiate robot
# ============================================================================

print("\n" + "=" * 80)
print("STEP 2: Instantiate robot and inspect joints")
print("=" * 80)

try:
    robot = robot_cls()
    print("✓ Robot instantiated")

    joints = robot.actuated_joint_names

    print(f"\nActuated joints ({len(joints)}):")
    for i, name in enumerate(joints):
        tag = "← JOINT4" if "joint4" in name.lower() else ""
        print(f"  [{i}] {name} {tag}")

    default_config = robot.get_default_config()

    print("\nDefault configuration:")
    for i, (name, value) in enumerate(zip(joints, default_config)):
        tag = "← JOINT4" if "joint4" in name.lower() else ""
        print(f"  [{i}] {name:30} = {value:+8.4f} rad {tag}")

except Exception as e:
    print(f"✗ Error in step 2: {e}")
    traceback.print_exc()
    sys.exit(1)

# ============================================================================
# STEP 3: Solver inspection (safe)
# ============================================================================

print("\n" + "=" * 80)
print("STEP 3: Inspect IK solver (if available)")
print("=" * 80)

try:
    from teleop_xr.ik.solver import PyrokiSolver

    solver = PyrokiSolver(robot)
    print("✓ PyrokiSolver created")

    if hasattr(solver, "robot"):
        pk = solver.robot

        print("\nJoint limits:")
        for i, name in enumerate(joints):
            try:
                lower = pk.joints.lower_limits[i]
                upper = pk.joints.upper_limits[i]
                tag = "← JOINT4" if "joint4" in name.lower() else ""
                print(f"  [{i}] {name:30} [{lower:+6.3f}, {upper:+6.3f}] {tag}")
            except Exception:
                print(f"  [{i}] {name:30} [limit read error]")

except Exception as e:
    print(f"⚠ Solver inspection skipped: {e}")

# ============================================================================
# STEP 4: Locate source code
# ============================================================================

print("\n" + "=" * 80)
print("STEP 4: Locate source files")
print("=" * 80)

try:
    import inspect
    from teleop_xr.ik.controller import IKController

    print("✓ IKController file:")
    print(inspect.getfile(IKController))

    robot_file = sys.modules[robot_cls.__module__].__file__
    print("\n✓ Robot class file:")
    print(robot_file)

    print("\n👉 THIS is the file you must inspect for JOINT4 cost locking")

except Exception as e:
    print(f"✗ Source inspection error: {e}")

# ============================================================================
# STEP 5: Helpful grep commands
# ============================================================================

print("\n" + "=" * 80)
print("STEP 5: Commands to find JOINT4 issue")
print("=" * 80)

robot_file = sys.modules[robot_cls.__module__].__file__

print("\nRun these in terminal:\n")

print(f"grep -n 'joint4' {robot_file}")
print(f"grep -n 'rest_cost' {robot_file}")
print(f"grep -n 'limit_cost' {robot_file}")
print(f"grep -n 'weight' {robot_file}")
print(f"cat {robot_file}")

print("\n" + "=" * 80)
print("DIAGNOSIS HINT")
print("=" * 80)

print("""
If left_joint4 is locked, it is almost always:

1) REST COST TOO HIGH
   → joint pulled back to default pose

2) JOINT EXCLUDED FROM IK
   → not in optimization variables

3) STRONG LIMIT COST
   → penalizing movement near bounds

4) SYMMETRY BUG IN URDF
   → left arm mirrored incorrectly

MOST LIKELY FIX:
→ reduce rest_cost weight for left_joint4
→ or remove it from rest_cost entirely
""")
