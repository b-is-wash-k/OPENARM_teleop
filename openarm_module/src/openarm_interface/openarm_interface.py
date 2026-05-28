#!/usr/bin/env python3
"""
OpenArm Robot Interface
=======================
Clean interface layer for the OpenArm bimanual robot over CAN bus.
Wraps the openarm_can Python bindings with higher-level methods.

Supported methods:
    home(speed)          - Move all joints to zero position
    moveJ(angles, speed) - Move to joint configuration (radians)
    getJ()               - Get current joint positions (radians)
    moveL(pose, speed)   - [STUB] Move end-effector to Cartesian pose (requires IK)
    getL()               - [STUB] Get end-effector Cartesian pose (requires FK)

Speed parameter (0.0–1.0):
    Motion duration is interpolated between MAX_MOVE_DURATION (slow, speed=0)
    and MIN_MOVE_DURATION (fast, speed=1). All moves use cosine easing so the
    arm accelerates and decelerates smoothly rather than jumping to target.
"""

import time
import numpy as np
import openarm_can as oa


# ---------------------------------------------------------------------------
# Motor configuration constants (matches LeRobot OpenArm setup)
# ---------------------------------------------------------------------------

MOTOR_TYPES = [
    oa.MotorType.DM8009,
    oa.MotorType.DM8009,
    oa.MotorType.DM4340,
    oa.MotorType.DM4340,
    oa.MotorType.DM4310,
    oa.MotorType.DM4310,
    oa.MotorType.DM4310,
]

ARM_SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
ARM_RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

GRIPPER_MOTOR_TYPE = oa.MotorType.DM4310
GRIPPER_SEND_ID = 0x08
GRIPPER_RECV_ID = 0x18

NUM_ARM_JOINTS = 7

# MIT gains — static, tuned for smooth movement. Do not use as a speed knob.
# To adjust stiffness/compliance change these, not the speed parameter.
DEFAULT_KP = [60.0, 60.0, 60.0, 60.0, 6.0, 8.0, 6.0, 6.0]  # index 7 = gripper
DEFAULT_KD = [2.0,  2.0,  1.5,  2.0,  0.2, 0.2, 0.2, 0.2]

# Control loop
CONTROL_RATE_HZ = 60
CONTROL_DT = 1.0 / CONTROL_RATE_HZ

# Motion duration bounds (seconds). Speed parameter maps linearly between these.
MIN_MOVE_DURATION = 2.0   # speed = 1.0 (fast)
MAX_MOVE_DURATION = 10.0  # speed = 0.0 (slow)
DEFAULT_SPEED = 0.3       # conservative default


# ---------------------------------------------------------------------------
# IK / FK not yet implemented
# ---------------------------------------------------------------------------

class IKNotImplementedError(NotImplementedError):
    """Raised when Cartesian-space methods are called before IK is available."""


# ---------------------------------------------------------------------------
# Motion helpers
# ---------------------------------------------------------------------------

def _speed_to_duration(speed: float) -> float:
    """Convert a speed value [0.0, 1.0] to a motion duration in seconds."""
    speed = float(np.clip(speed, 0.0, 1.0))
    return MAX_MOVE_DURATION + speed * (MIN_MOVE_DURATION - MAX_MOVE_DURATION)


def _cosine_interpolate(start: np.ndarray, end: np.ndarray, progress: float) -> np.ndarray:
    """
    Cosine-eased interpolation between start and end.
    progress in [0.0, 1.0]. Matches the zero-return profile in the teleop code.
    """
    t = 0.5 - 0.5 * np.cos(progress * np.pi)
    return start + t * (end - start)


# ---------------------------------------------------------------------------
# Single arm wrapper
# ---------------------------------------------------------------------------

class OpenArmSingle:
    """
    Wraps one OpenArm instance (one CAN interface) and exposes
    joint-space control methods.
    """

    def __init__(
        self,
        can_interface: str,
        is_right: bool,
        kp: list[float] | None = None,
        kd: list[float] | None = None,
        enable_debug: bool = True,
    ):
        """
        Args:
            can_interface: CAN interface name, e.g. "can0" or "can1".
            is_right:      True for the right arm, False for left. Used for
                           reference only — the underlying library does not
                           distinguish arms, both use identical CAN IDs.
            kp:            MIT position gains for [joint0..6, gripper].
                           Defaults to DEFAULT_KP.
            kd:            MIT derivative gains for [joint0..6, gripper].
                           Defaults to DEFAULT_KD.
            enable_debug:  Pass True to the openarm_can library for debug
                           output. Note: the second argument to OpenArm() is
                           enable_debug, NOT is_right.
        """
        self.can_interface = can_interface
        self.is_right = is_right
        self.kp = kp if kp is not None else list(DEFAULT_KP)
        self.kd = kd if kd is not None else list(DEFAULT_KD)

        self._arm = oa.OpenArm(can_interface, enable_debug)
        self._initialized = False

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def initialize(self, warmup_steps: int = 30):
        """
        Initialize motors and enable the arm. Must be called before motion.

        Sequence matches official openarm_init.cpp:
          1. STATE callback mode, enable_all()
          2. sleep 100ms → recv_all() → sleep 100ms
          3. Short MIT warmup loop to prime motor state
        """
        self._arm.init_arm_motors(MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
        self._arm.init_gripper_motor(GRIPPER_MOTOR_TYPE, GRIPPER_SEND_ID, GRIPPER_RECV_ID)

        self._arm.set_callback_mode_all(oa.CallbackMode.STATE)
        self._arm.enable_all()
        time.sleep(0.1)
        self._arm.recv_all(2000)
        time.sleep(0.1)

        # Prime motor state with zero-gain commands so motors respond without moving.
        # kp=0, kd=0 means no position/damping force — purely solicits state feedback.
        zero_arm = [oa.MITParam(0.0, 0.0, 0.0, 0, 0) for _ in range(NUM_ARM_JOINTS)]
        zero_gripper = [oa.MITParam(0.0, 0.0, 0.0, 0, 0)]
        for _ in range(warmup_steps):
            self._arm.get_arm().mit_control_all(zero_arm)
            self._arm.get_gripper().mit_control_all(zero_gripper)
            self._arm.recv_all(500)
            time.sleep(CONTROL_DT)

        self._initialized = True

    def shutdown(self):
        """Disable all motors gracefully."""
        self._require_initialized()
        self._arm.disable_all()
        self._arm.recv_all(1000)
        self._initialized = False

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def home(self, speed: float = DEFAULT_SPEED):
        """
        Move all joints (including gripper) to zero using cosine easing.
        Reads current joint positions first so the arm never jumps.
        Blocking — returns when motion is complete.

        Args:
            speed: Motion speed in [0.0, 1.0]. 0 = slowest, 1 = fastest.
                   Defaults to DEFAULT_SPEED.
        """
        self._require_initialized()
        state = self.getJ()  # sets STATE callback internally
        start = np.array(state["positions"] + [state["gripper"]])  # shape (8,)
        end = np.zeros(NUM_ARM_JOINTS + 1)
        self._run_interpolated_move(start, end, speed)

    def moveJ(
        self,
        joint_angles: list[float],
        gripper_angle: float = 0.0,
        speed: float = DEFAULT_SPEED,
    ):
        """
        Move arm to specified joint configuration using cosine easing.
        Reads current positions first so motion always starts from where the arm is.
        Blocking — returns when motion is complete.

        Args:
            joint_angles:  Target positions for the 7 arm joints, in radians.
                           Must have exactly 7 elements.
            gripper_angle: Target gripper position in radians. Defaults to 0.0.
            speed:         Motion speed in [0.0, 1.0]. Defaults to DEFAULT_SPEED.

        Raises:
            ValueError: If joint_angles does not have exactly 7 elements.
        """
        self._require_initialized()
        if len(joint_angles) != NUM_ARM_JOINTS:
            raise ValueError(
                f"joint_angles must have {NUM_ARM_JOINTS} elements, got {len(joint_angles)}"
            )

        state = self.getJ()  # sets STATE callback internally
        start = np.array(state["positions"] + [state["gripper"]])
        end = np.array(list(joint_angles) + [gripper_angle])
        self._run_interpolated_move(start, end, speed)

    def getJ(self) -> dict:
        """
        Read current joint state from the arm.

        Returns:
            dict with keys:
                "positions"  (list[float]): Joint positions in radians [joint0..6].
                "velocities" (list[float]): Joint velocities in rad/s [joint0..6].
                "torques"    (list[float]): Joint torques in Nm [joint0..6].
                "gripper"    (float):       Gripper position in radians.
        """
        self._require_initialized()
        # Outside a control loop we need to explicitly request state.
        # Switch to IGNORE so the request frame isn't misread, then STATE to parse response.
        self._arm.set_callback_mode_all(oa.CallbackMode.IGNORE)
        self._arm.refresh_all()
        self._arm.set_callback_mode_all(oa.CallbackMode.STATE)
        self._arm.recv_all(2000)

        arm_motors     = self._arm.get_arm().get_motors()
        gripper_motors = self._arm.get_gripper().get_motors()

        positions  = [m.get_position() for m in arm_motors]
        velocities = [m.get_velocity() for m in arm_motors]
        torques    = [m.get_torque()   for m in arm_motors]
        gripper_pos = gripper_motors[0].get_position() if gripper_motors else 0.0

        return {
            "positions":  positions,
            "velocities": velocities,
            "torques":    torques,
            "gripper":    gripper_pos,
        }

    def moveL(self, pose: list[float], speed: float = DEFAULT_SPEED):
        """
        [STUB] Move end-effector to a Cartesian pose.
        Requires an IK solver — not yet implemented.

        Args:
            pose:  Target end-effector pose [x, y, z, roll, pitch, yaw].
            speed: Motion speed in [0.0, 1.0].

        Raises:
            IKNotImplementedError: Always, until IK is implemented.
        """
        raise IKNotImplementedError(
            "moveL requires an IK solver which is not yet implemented. "
            "Use moveJ for joint-space control."
        )

    def getL(self) -> list[float]:
        """
        [STUB] Get current end-effector Cartesian pose.
        Requires a forward kinematics model — not yet implemented.

        Raises:
            IKNotImplementedError: Always, until FK is implemented.
        """
        raise IKNotImplementedError(
            "getL requires a forward kinematics model which is not yet implemented. "
            "Use getJ for joint-space feedback."
        )

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _require_initialized(self):
        if not self._initialized:
            raise RuntimeError(
                "Arm is not initialized. Call initialize() before issuing commands."
            )

    def _run_interpolated_move(
        self,
        start: np.ndarray,
        end: np.ndarray,
        speed: float,
    ):
        """
        Drive the arm from start to end using cosine easing over the duration
        determined by speed. start and end are shape-(8,) arrays: [joint0..6, gripper].
        Caller is responsible for setting callback mode before calling this.
        """
        duration = _speed_to_duration(speed)
        steps = max(1, int(duration * CONTROL_RATE_HZ))

        for step in range(steps):
            progress = step / steps
            current = _cosine_interpolate(start, end, progress)

            arm_params = [
                oa.MITParam(self.kp[i], self.kd[i], float(current[i]), 0, 0)
                for i in range(NUM_ARM_JOINTS)
            ]
            gripper_params = [oa.MITParam(self.kp[7], self.kd[7], float(current[7]), 0, 0)]

            self._arm.get_arm().mit_control_all(arm_params)
            self._arm.get_gripper().mit_control_all(gripper_params)
            self._arm.recv_all(500)
            time.sleep(CONTROL_DT)

        # Hold final target exactly
        arm_params = [
            oa.MITParam(self.kp[i], self.kd[i], float(end[i]), 0, 0)
            for i in range(NUM_ARM_JOINTS)
        ]
        gripper_params = [oa.MITParam(self.kp[7], self.kd[7], float(end[7]), 0, 0)]
        self._arm.get_arm().mit_control_all(arm_params)
        self._arm.get_gripper().mit_control_all(gripper_params)
        self._arm.recv_all(500)


# ---------------------------------------------------------------------------
# Bimanual interface (convenience wrapper around two OpenArmSingle instances)
# ---------------------------------------------------------------------------

class OpenArmBimanual:
    """
    Convenience wrapper for simultaneous control of both arms.
    Exposes the same interface as OpenArmSingle but operates on one or both arms.
    """

    def __init__(
        self,
        right_can: str = "can0",
        left_can: str = "can1",
        kp: list[float] | None = None,
        kd: list[float] | None = None,
    ):
        self.right = OpenArmSingle(right_can, is_right=True,  kp=kp, kd=kd)
        self.left  = OpenArmSingle(left_can,  is_right=False, kp=kp, kd=kd)

    def initialize(self):
        """Initialize both arms."""
        self.right.initialize()
        self.left.initialize()

    def shutdown(self, right: bool = True, left: bool = True):
        """Disable one or both arms."""
        if right and self.right._initialized:
            self.right.shutdown()
        if left and self.left._initialized:
            self.left.shutdown()

    def home(self, right: bool = True, left: bool = True, speed: float = DEFAULT_SPEED):
        """
        Move one or both arms to zero using cosine easing.
        Reads current positions first so the arms never jump.
        Blocking — returns when complete.

        Args:
            right: Home the right arm. Defaults to True.
            left:  Home the left arm. Defaults to True.
            speed: Motion speed in [0.0, 1.0]. Defaults to DEFAULT_SPEED.
        """
        if not right and not left:
            raise ValueError("At least one arm must be selected.")
        self._require_initialized(right=right, left=left)

        arms = self._active_arms(right, left)
        duration = _speed_to_duration(speed)
        steps = max(1, int(duration * CONTROL_RATE_HZ))

        # Read starting positions (getJ sets STATE callback internally)
        starts = {arm: self._read_start(arm) for arm in arms}
        end = np.zeros(NUM_ARM_JOINTS + 1)

        for step in range(steps):
            progress = step / steps
            for arm in arms:
                current = _cosine_interpolate(starts[arm], end, progress)
                arm_params = [
                    oa.MITParam(arm.kp[i], arm.kd[i], float(current[i]), 0, 0)
                    for i in range(NUM_ARM_JOINTS)
                ]
                gripper_params = [oa.MITParam(arm.kp[7], arm.kd[7], float(current[7]), 0, 0)]
                arm._arm.get_arm().mit_control_all(arm_params)
                arm._arm.get_gripper().mit_control_all(gripper_params)
            for arm in arms:
                arm._arm.recv_all(500)
            time.sleep(CONTROL_DT)

        # Hold final zero exactly
        for arm in arms:
            arm_params = [oa.MITParam(arm.kp[i], arm.kd[i], 0.0, 0, 0) for i in range(NUM_ARM_JOINTS)]
            gripper_params = [oa.MITParam(arm.kp[7], arm.kd[7], 0.0, 0, 0)]
            arm._arm.get_arm().mit_control_all(arm_params)
            arm._arm.get_gripper().mit_control_all(gripper_params)
        for arm in arms:
            arm._arm.recv_all(500)

    def moveJ(
        self,
        right_angles: list[float] | None = None,
        left_angles: list[float] | None = None,
        right_gripper: float = 0.0,
        left_gripper: float = 0.0,
        speed: float = DEFAULT_SPEED,
    ):
        """
        Move one or both arms to specified joint configurations simultaneously,
        using cosine easing. Reads current positions first.
        Blocking — returns when complete.

        Args:
            right_angles:  7-element target joint positions (rad) for right arm, or None to skip.
            left_angles:   7-element target joint positions (rad) for left arm, or None to skip.
            right_gripper: Right gripper target (rad). Used only if right_angles is set.
            left_gripper:  Left gripper target (rad). Used only if left_angles is set.
            speed:         Motion speed in [0.0, 1.0]. Defaults to DEFAULT_SPEED.

        Raises:
            ValueError: If both arms are None, or an angle list has the wrong length.
        """
        if right_angles is None and left_angles is None:
            raise ValueError("At least one of right_angles or left_angles must be provided.")
        if right_angles is not None and len(right_angles) != NUM_ARM_JOINTS:
            raise ValueError(f"right_angles must have {NUM_ARM_JOINTS} elements, got {len(right_angles)}.")
        if left_angles is not None and len(left_angles) != NUM_ARM_JOINTS:
            raise ValueError(f"left_angles must have {NUM_ARM_JOINTS} elements, got {len(left_angles)}.")

        self._require_initialized(right=right_angles is not None, left=left_angles is not None)

        active: list[tuple[OpenArmSingle, np.ndarray]] = []
        if right_angles is not None:
            active.append((self.right, np.array(list(right_angles) + [right_gripper])))
        if left_angles is not None:
            active.append((self.left, np.array(list(left_angles) + [left_gripper])))

        # Read starting positions (getJ sets STATE callback internally)
        starts = {arm: self._read_start(arm) for arm, _ in active}
        ends   = {arm: end for arm, end in active}

        duration = _speed_to_duration(speed)
        steps = max(1, int(duration * CONTROL_RATE_HZ))

        for step in range(steps):
            progress = step / steps
            for arm, _ in active:
                current = _cosine_interpolate(starts[arm], ends[arm], progress)
                arm_params = [
                    oa.MITParam(arm.kp[i], arm.kd[i], float(current[i]), 0, 0)
                    for i in range(NUM_ARM_JOINTS)
                ]
                gripper_params = [oa.MITParam(arm.kp[7], arm.kd[7], float(current[7]), 0, 0)]
                arm._arm.get_arm().mit_control_all(arm_params)
                arm._arm.get_gripper().mit_control_all(gripper_params)
            for arm, _ in active:
                arm._arm.recv_all(500)
            time.sleep(CONTROL_DT)

        # Hold final targets exactly
        for arm, _ in active:
            target = ends[arm]
            arm_params = [
                oa.MITParam(arm.kp[i], arm.kd[i], float(target[i]), 0, 0)
                for i in range(NUM_ARM_JOINTS)
            ]
            gripper_params = [oa.MITParam(arm.kp[7], arm.kd[7], float(target[7]), 0, 0)]
            arm._arm.get_arm().mit_control_all(arm_params)
            arm._arm.get_gripper().mit_control_all(gripper_params)
        for arm, _ in active:
            arm._arm.recv_all(500)

    def getJ(self, right: bool = True, left: bool = True) -> dict:
        """
        Read current joint state from one or both arms.

        Args:
            right: Include right arm state. Defaults to True.
            left:  Include left arm state. Defaults to True.

        Returns:
            dict with keys "right" and/or "left", each containing the same
            structure as OpenArmSingle.getJ().
        """
        result = {}
        if right:
            result["right"] = self.right.getJ()
        if left:
            result["left"] = self.left.getJ()
        return result

    def moveL(self, right_pose: list[float] | None = None, left_pose: list[float] | None = None, speed: float = DEFAULT_SPEED):
        """[STUB] Not implemented — requires IK solver."""
        raise IKNotImplementedError("moveL is not yet implemented.")

    def getL(self) -> dict:
        """[STUB] Not implemented — requires forward kinematics."""
        raise IKNotImplementedError("getL is not yet implemented.")

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _require_initialized(self, right: bool = True, left: bool = True):
        if right and not self.right._initialized:
            raise RuntimeError("Right arm is not initialized. Call initialize() first.")
        if left and not self.left._initialized:
            raise RuntimeError("Left arm is not initialized. Call initialize() first.")

    def _active_arms(self, right: bool, left: bool) -> list:
        arms = []
        if right:
            arms.append(self.right)
        if left:
            arms.append(self.left)
        return arms

    def _read_start(self, arm: OpenArmSingle) -> np.ndarray:
        """Read current arm state into a shape-(8,) array [joint0..6, gripper]."""
        state = arm.getJ()
        return np.array(state["positions"] + [state["gripper"]])


# ---------------------------------------------------------------------------
# Example usage
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    robot = OpenArmBimanual(right_can="can0", left_can="can1")

    try:
        print("Initializing arms...")
        robot.initialize()

        # Home both arms at default speed
        print("Homing both arms...")
        robot.home()

        # Home only the right arm, faster
        print("Homing right arm only at speed 0.7...")
        robot.home(right=True, left=False, speed=0.7)

        # Read state from both arms
        state = robot.getJ()
        print("Right arm positions:", state["right"]["positions"])
        print("Left arm positions: ", state["left"]["positions"])

        # Move only the right arm
        right_target = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        print("Moving right arm only...")
        robot.moveJ(right_angles=right_target, speed=0.5)

        # Move both arms simultaneously
        left_target = [0.0, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0]
        print("Moving both arms...")
        robot.moveJ(right_angles=right_target, left_angles=left_target, speed=0.3)

        # Home both arms to finish
        print("Homing both arms...")
        robot.home(speed=0.5)

    finally:
        print("Shutting down...")
        robot.shutdown()