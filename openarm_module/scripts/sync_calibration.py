#!/usr/bin/env python3
import openarm_can as oa
import time

print("=== OpenArm Hardware-Synced Calibration ===")

# Step 1: Move to hardware zero
print("\nStep 1: Moving arms to hardware-calibrated zero...")

right_arm = oa.OpenArm("can2", True)
left_arm = oa.OpenArm("can3", True)

motor_types = [
    oa.MotorType.DM8009, oa.MotorType.DM8009,
    oa.MotorType.DM4340, oa.MotorType.DM4340,
    oa.MotorType.DM4310, oa.MotorType.DM4310,
    oa.MotorType.DM4310
]
send_ids = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
recv_ids = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

right_arm.init_arm_motors(motor_types, send_ids, recv_ids)
right_arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)
left_arm.init_arm_motors(motor_types, send_ids, recv_ids)
left_arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)

right_arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
left_arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
right_arm.enable_all()
left_arm.enable_all()
right_arm.recv_all()
left_arm.recv_all()

right_arm.set_callback_mode_all(oa.CallbackMode.STATE)
left_arm.set_callback_mode_all(oa.CallbackMode.STATE)

zero_params = [oa.MITParam(10.0, 2.0, 0.0, 0, 0) for _ in range(7)]
gripper_zero = [oa.MITParam(10.0, 2.0, 0.0, 0, 0)]

for step in range(300):
    right_arm.get_arm().mit_control_all(zero_params)
    right_arm.get_gripper().mit_control_all(gripper_zero)
    left_arm.get_arm().mit_control_all(zero_params)
    left_arm.get_gripper().mit_control_all(gripper_zero)
    right_arm.recv_all()
    left_arm.recv_all()
    time.sleep(1.0/60.0)

print("✓ Arms at hardware zero!")

# Disable C++ interface
print("\nDisabling C++ interface...")
right_arm.disable_all()
left_arm.disable_all()
right_arm.recv_all(1000)
left_arm.recv_all(1000)
time.sleep(0.5)

# Step 2: Run LeRobot calibration
print("\nStep 2: Running LeRobot calibration at hardware zero...")

from lerobot.robots.bi_openarm_follower.config_bi_openarm_follower import BiOpenArmFollowerConfig
from lerobot.robots.openarm_follower.config_openarm_follower import OpenArmFollowerConfig
from lerobot.robots import make_robot_from_config

# Create config
left_config = OpenArmFollowerConfig(port='can1', side='left')
right_config = OpenArmFollowerConfig(port='can0', side='right')
config = BiOpenArmFollowerConfig(left_arm_config=left_config, right_arm_config=right_config)

# Create robot
robot = make_robot_from_config(config)

# Connect WITHOUT calibration (arms already at zero)
robot.connect(calibrate=False)

try:
    # Run calibration (will set current position as zero and save)
    robot.calibrate()
finally:
    robot.disconnect()

print("\n✓ COMPLETE! LeRobot calibrated at hardware zero.")
print("Test with: python ~/movet_to_zero.py")