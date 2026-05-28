#!/usr/bin/env python
"""
OpenArm Wave Demo - Right arm waves by rotating wrist (J6)

Usage:
    python wave_demo.py                          # Default: 20 waves, 30°, 60°/s
    python wave_demo.py --waves 10               # 10 waves
    python wave_demo.py --angle 40 --speed 90    # Faster, wider waves
"""

import time
import argparse
import numpy as np
from lerobot.robots.bi_openarm_follower import BiOpenArmFollower
from lerobot.robots.bi_openarm_follower.config_bi_openarm_follower import BiOpenArmFollowerConfig
from lerobot.robots.openarm_follower.config_openarm_follower import OpenArmFollowerConfig

# Raised position for right arm
RAISED_POSITION = {
    'joint_1': 13.34,
    'joint_2': 84.02,
    'joint_3': 62.29,
    'joint_4': 90.57,
    'joint_5': -78.25,
    'joint_6': 0.00,
    'joint_7': 0.00,
    'gripper': 0.00,
}

# Zero/rest position
ZERO_POSITION = {
    'joint_1': 0.0,
    'joint_2': 0.0,
    'joint_3': 0.0,
    'joint_4': 0.0,
    'joint_5': 0.0,
    'joint_6': 0.0,
    'joint_7': 0.0,
    'gripper': 0.0,
}

def move_smooth(robot, arm, start_pos, end_pos, duration=1.0, fps=60):
    """Move arm smoothly from start to end position."""
    num_steps = int(duration * fps)
    dt = 1.0 / fps
    other_arm = 'left' if arm == 'right' else 'right'
    
    # Get state
    state = robot.get_observation()
    
    # Build arrays
    start = np.array([start_pos[f'joint_{i}'] for i in range(1, 8)])
    start = np.append(start, start_pos['gripper'])
    
    end = np.array([end_pos[f'joint_{i}'] for i in range(1, 8)])
    end = np.append(end, end_pos['gripper'])
    
    for step in range(num_steps + 1):
        t = step / num_steps
        smooth_t = 0.5 - 0.5 * np.cos(t * np.pi)
        pos = start + smooth_t * (end - start)
        
        action = {}
        # Keep other arm still
        for i in range(1, 8):
            action[f"{other_arm}_joint_{i}.pos"] = state[f"{other_arm}_joint_{i}.pos"]
        action[f"{other_arm}_gripper.pos"] = state[f"{other_arm}_gripper.pos"]
        
        # Move active arm
        for i in range(7):
            action[f"{arm}_joint_{i+1}.pos"] = float(pos[i])
        action[f"{arm}_gripper.pos"] = float(pos[7])
        
        robot.send_action(action)
        time.sleep(dt)

def wave_j6(robot, base_position, wave_angle=30.0, num_waves=20, speed=60):
    """
    Wave by oscillating J6 (wrist pitch).
    
    Args:
        robot: BiOpenArmFollower instance
        base_position: Base position dict
        wave_angle: Max angle to wave (degrees)
        num_waves: Number of complete waves
        speed: Wave speed in degrees/second
    """
    # Calculate duration for one half-wave
    half_wave_duration = wave_angle / speed
    
    # Create wave positions
    wave_left = base_position.copy()
    wave_left['joint_6'] = -wave_angle
    
    wave_right = base_position.copy()
    wave_right['joint_6'] = wave_angle
    
    print(f"Waving J6 between -{wave_angle}° and +{wave_angle}° ({num_waves} waves)...")
    
    for i in range(num_waves):
        # Wave left
        move_smooth(robot, 'right', base_position if i == 0 else wave_right, wave_left, duration=half_wave_duration)
        # Wave right
        move_smooth(robot, 'right', wave_left, wave_right, duration=half_wave_duration)
        
        if (i + 1) % 5 == 0:
            print(f"  Completed {i + 1}/{num_waves} waves")
    
    # Return to center
    move_smooth(robot, 'right', wave_right, base_position, duration=half_wave_duration)

def main(num_waves=20, wave_angle=30.0, speed=60):
    """Run wave demo with specified parameters."""
    print("="*60)
    print("OpenArm Simple Wave Demo - Right Arm J6 Wave")
    print("="*60)
    print(f"Parameters: {num_waves} waves, ±{wave_angle}°, {speed}°/s")
    print("="*60)
    
    # Create robot configuration
    left_config = OpenArmFollowerConfig(port='can1', side='left')
    right_config = OpenArmFollowerConfig(port='can0', side='right')
    config = BiOpenArmFollowerConfig(
        left_arm_config=left_config,
        right_arm_config=right_config
    )
    
    # Initialize robot
    print("\nConnecting to robot...")
    robot = BiOpenArmFollower(config)
    robot.connect()
    print("✓ Robot connected")
    
    try:
        # Simple wave demo
        print("\n[1/3] Raising right arm to wave position...")
        state = robot.get_observation()
        current_pos = {f'joint_{i}': state[f"right_joint_{i}.pos"] for i in range(1, 8)}
        current_pos['gripper'] = state['right_gripper.pos']
        
        move_smooth(robot, 'right', current_pos, RAISED_POSITION, duration=3.0)
        print("✓ Right arm raised")
        time.sleep(0.5)
        
        print(f"\n[2/3] Waving {num_waves} times at {speed}°/s...")
        wave_j6(robot, RAISED_POSITION, wave_angle=wave_angle, num_waves=num_waves, speed=speed)
        print("✓ Wave complete")
        time.sleep(0.5)
        
        print("\n[3/3] Returning to zero position...")
        move_smooth(robot, 'right', RAISED_POSITION, ZERO_POSITION, duration=3.0)
        print("✓ Returned to zero")
        
        print("\n" + "="*60)
        print("✓ Demo complete!")
        print("="*60)
        
    except KeyboardInterrupt:
        print("\n\nDemo interrupted by user")
    
    finally:
        print("\nDisconnecting robot...")
        robot.disconnect()
        print("✓ Robot disconnected")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='OpenArm wave demo - makes right arm wave using J6',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python wave_demo.py                           # Default: 20 waves, 30°, 60°/s
  python wave_demo.py --waves 10                # Quick demo with 10 waves
  python wave_demo.py --angle 40 --speed 90     # Faster, wider waves
        """
    )
    
    parser.add_argument('--waves', type=int, default=5, help='Number of complete waves (default: 5)')
    parser.add_argument('--angle', type=float, default=30.0, help='Wave angle in degrees (default: 30.0)')
    parser.add_argument('--speed', type=float, default=30.0, help='Wave speed in degrees/second (default: 30.0)')
    
    args = parser.parse_args()
    
    # Validate parameters
    if args.waves < 1:
        parser.error("--waves must be at least 1")
    if args.angle <= 0 or args.angle > 40:
        parser.error("--angle must be between 0 and 40 degrees (J6 limit is ±40°)")
    if args.speed <= 0 or args.speed > 120:
        parser.error("--speed must be between 0 and 120 degrees/second")
    
    main(num_waves=args.waves, wave_angle=args.angle, speed=args.speed)