#!/usr/bin/env python3
import openarm_can as oa
import time

print("=== Moving arms to zero (slow and smooth) ===")

# Reduced gains for slower, smoother movement
# Original LeRobot: kp=[240, 240, 240, 240, 24, 31, 25, 25], kd=[5, 5, 3, 5, 0.3, 0.3, 0.3, 0.3]
# Slower version: reduce kp by ~75%, keep kd proportional
kp_values = [60.0, 60.0, 60.0, 60.0, 6.0, 8.0, 6.0, 6.0]  # ~25% of original
kd_values = [2.0, 2.0, 1.5, 2.0, 0.2, 0.2, 0.2, 0.2]      # Reduced proportionally

right_arm = oa.OpenArm("can0", True)
left_arm = oa.OpenArm("can1", True)

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

print("Moving to zero position (slowly)...")
right_arm.set_callback_mode_all(oa.CallbackMode.STATE)
left_arm.set_callback_mode_all(oa.CallbackMode.STATE)

zero_params = [
    oa.MITParam(kp_values[i], kd_values[i], 0.0, 0, 0) 
    for i in range(7)
]
gripper_zero = [oa.MITParam(kp_values[7], kd_values[7], 0.0, 0, 0)]

for step in range(600):  # Double the time (10 seconds instead of 5)
    right_arm.get_arm().mit_control_all(zero_params)
    right_arm.get_gripper().mit_control_all(gripper_zero)
    left_arm.get_arm().mit_control_all(zero_params)
    left_arm.get_gripper().mit_control_all(gripper_zero)
    right_arm.recv_all()
    left_arm.recv_all()
    time.sleep(1.0/60.0)

print("✓ At zero! Disabling...")
right_arm.disable_all()
left_arm.disable_all()
right_arm.recv_all(1000)
left_arm.recv_all(1000)
