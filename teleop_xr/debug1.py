#!/usr/bin/env python3
"""
Debug script for teleop_xr IK robot loading and joint inspection
"""

import sys
import traceback

from teleop_xr.ik.loader import load_robot_class
from teleop_xr.ik_utils import list_available_robots


def main():
    try:
        print("===================================================")
        print("STEP 1: Check teleop_xr installation")
        print("===================================================")

        print("✓ teleop_xr imported successfully")

        print("\n===================================================")
        print("STEP 2: List available robots (entry points)")
        print("===================================================")

        robots = list_available_robots()

        if not robots:
            print("⚠ No robots found via entry points")
        else:
            print(f"✓ Found {len(robots)} robots:")
            for name, path in robots.items():
                print(f"  - {name}: {path}")

        print("\n===================================================")
        print("STEP 3: Load default robot class")
        print("===================================================")

        # IMPORTANT: safest option (works even if entry points are broken)
        robot_cls = load_robot_class(None)

        print(f"✓ Loaded robot class: {robot_cls}")
        print(f"  Module: {robot_cls.__module__}")

        module_file = sys.modules[robot_cls.__module__].__file__
        print(f"  File: {module_file}")

        print("\n===================================================")
        print("STEP 4: Instantiate robot")
        print("===================================================")

        robot = robot_cls()

        print("✓ Robot instantiated successfully")

        # These attributes may or may not exist depending on implementation
        if hasattr(robot, "actuated_joint_names"):
            print("\n✓ Actuated joints:")
            print(robot.actuated_joint_names)
        else:
            print("⚠ No 'actuated_joint_names' attribute found")

        if hasattr(robot, "get_default_config"):
            print("\n✓ Default config:")
            print(robot.get_default_config())
        else:
            print("⚠ No 'get_default_config' method found")

        print("\n===================================================")
        print("DEBUG COMPLETE")
        print("===================================================")

    except Exception as e:
        print("\n✗ ERROR OCCURRED")
        print("===================================================")
        print(e)
        traceback.print_exc()


if __name__ == "__main__":
    main()
