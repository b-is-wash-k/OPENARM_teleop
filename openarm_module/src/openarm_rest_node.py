#!/usr/bin/env python3
"""REST node for the OpenArm bimanual robot."""

from typing import Annotated, Optional

from madsci.common.types.action_types import ActionFailed
from madsci.common.types.node_types import RestNodeConfig
from madsci.node_module.helpers import action
from madsci.node_module.rest_node_module import RestNode

from openarm_interface.openarm_interface import OpenArmBimanual


class OpenArmNodeConfig(RestNodeConfig):
    """Configuration for the OpenArm node module."""

    right_can: str = "can0"
    """CAN interface for the right arm."""
    left_can: str = "can1"
    """CAN interface for the left arm."""


class OpenArmNode(RestNode):
    """A Rest Node object to control the OpenArm bimanual robot."""

    robot: Optional[OpenArmBimanual] = None
    config: OpenArmNodeConfig = OpenArmNodeConfig()
    config_model = OpenArmNodeConfig

    def startup_handler(self) -> None:
        """Called to (re)initialize the node. Opens CAN connections and enables both arms."""
        self.robot = OpenArmBimanual(
            right_can=self.config.right_can,
            left_can=self.config.left_can,
        )
        self.robot.initialize()
        self.logger.log_info("OpenArm Node initialized.")

    def shutdown_handler(self) -> None:
        """Called to shutdown the node. Disables both arms and releases CAN resources."""
        try:
            if self.robot is not None:
                self.robot.shutdown()
                del self.robot
                self.robot = None
        except Exception as err:
            self.logger.log_error(f"Error shutting down the OpenArm Node: {err}")
            raise err

    def state_handler(self) -> None:
        """Periodically called to update the current state of the node."""
        if self.robot is not None:
            try:
                state = self.robot.getJ()
                self.node_state = {
                    "right_arm": {
                        "positions":  state["right"]["positions"],
                        "velocities": state["right"]["velocities"],
                        "torques":    state["right"]["torques"],
                        "gripper":    state["right"]["gripper"],
                    },
                    "left_arm": {
                        "positions":  state["left"]["positions"],
                        "velocities": state["left"]["velocities"],
                        "torques":    state["left"]["torques"],
                        "gripper":    state["left"]["gripper"],
                    },
                }
            except Exception as err:
                self.logger.log_error(f"Error reading arm state: {err}")

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------

    @action(name="home", description="Move one or both arms to the zero position using cosine easing.")
    def home(
        self,
        right: Annotated[bool, "Home the right arm."] = True,
        left: Annotated[bool, "Home the left arm."] = True,
        speed: Annotated[Optional[float], "Motion speed [0.0-1.0]. 0 = slowest, 1 = fastest. Defaults to interface default."] = None,
    ) -> Optional[ActionFailed]:
        """Move one or both arms smoothly to their zero (home) position."""
        if not right and not left:
            return ActionFailed(errors=["At least one arm must be selected (right and/or left)."])
        try:
            kwargs = {"right": right, "left": left}
            if speed is not None:
                kwargs["speed"] = speed
            self.robot.home(**kwargs)
        except Exception as err:
            return ActionFailed(errors=[f"Home failed: {err}"])
        return None

    @action(name="moveJ", description="Move one or both arms to specified joint configurations.")
    def moveJ(
        self,
        right_angles: Annotated[
            Optional[list[float]],
            "7-element list of target joint positions in radians for the right arm. Pass null to skip.",
        ] = None,
        left_angles: Annotated[
            Optional[list[float]],
            "7-element list of target joint positions in radians for the left arm. Pass null to skip.",
        ] = None,
        right_gripper: Annotated[float, "Right gripper target position in radians."] = 0.0,
        left_gripper: Annotated[float, "Left gripper target position in radians."] = 0.0,
        speed: Annotated[Optional[float], "Motion speed [0.0-1.0]. 0 = slowest, 1 = fastest. Defaults to interface default."] = None,
    ) -> Optional[ActionFailed]:
        """Move one or both arms to the specified joint configuration using cosine easing."""
        if right_angles is None and left_angles is None:
            return ActionFailed(errors=["At least one of right_angles or left_angles must be provided."])
        try:
            kwargs = {
                "right_angles": right_angles,
                "left_angles": left_angles,
                "right_gripper": right_gripper,
                "left_gripper": left_gripper,
            }
            if speed is not None:
                kwargs["speed"] = speed
            self.robot.moveJ(**kwargs)
        except ValueError as err:
            return ActionFailed(errors=[f"Invalid joint angles: {err}"])
        except Exception as err:
            return ActionFailed(errors=[f"moveJ failed: {err}"])
        return None

    @action(name="getJ", description="Read current joint state from one or both arms.")
    def getJ(
        self,
        right: Annotated[bool, "Read right arm state."] = True,
        left: Annotated[bool, "Read left arm state."] = True,
    ) -> dict:
        """Return current joint positions, velocities, and torques for one or both arms."""
        try:
            return self.robot.getJ(right=right, left=left)
        except Exception as err:
            return ActionFailed(errors=[f"getJ failed: {err}"])

    @action(
        name="moveL",
        description="[STUB] Move end-effector to a Cartesian pose. Not yet implemented - requires IK.",
    )
    def moveL(
        self,
        right_pose: Annotated[
            Optional[list[float]],
            "[x, y, z, roll, pitch, yaw] target pose for the right arm in metres/radians.",
        ] = None,
        left_pose: Annotated[
            Optional[list[float]],
            "[x, y, z, roll, pitch, yaw] target pose for the left arm in metres/radians.",
        ] = None,
        speed: Annotated[Optional[float], "Motion speed [0.0-1.0]."] = None,
    ) -> ActionFailed:
        """Not yet implemented - requires an IK solver (e.g. pinocchio + OpenArm URDF)."""
        return ActionFailed(errors=["moveL is not yet implemented. An IK solver is required."])

    @action(
        name="getL",
        description="[STUB] Get end-effector Cartesian pose. Not yet implemented - requires FK.",
    )
    def getL(self) -> ActionFailed:
        """Not yet implemented - requires a forward kinematics model."""
        return ActionFailed(errors=["getL is not yet implemented. A forward kinematics model is required."])

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def pause(self) -> None:
        """Pause the node."""
        self.logger.log("Pausing node...")
        self.node_status.paused = True
        self.logger.log("Node paused.")
        return True

    def resume(self) -> None:
        """Resume the node."""
        self.logger.log("Resuming node...")
        self.node_status.paused = False
        self.logger.log("Node resumed.")
        return True

    def shutdown(self) -> None:
        """Shutdown the node."""
        self.shutdown_handler()
        return True

    def reset(self) -> None:
        """Reset the node."""
        self.logger.log("Resetting node...")
        result = super().reset()
        self.logger.log("Node reset.")
        return result

    def safety_stop(self) -> None:
        """Emergency stop - disable all motors immediately."""
        self.logger.log("Stopping node...")
        self.node_status.stopped = True
        try:
            if self.robot is not None:
                self.robot.shutdown()
                del self.robot
                self.robot = None
        except Exception as err:
            self.logger.log_error(f"Error during safety stop: {err}")
        self.logger.log("Node stopped.")
        return True

    def cancel(self) -> None:
        """Cancel the current action."""
        self.logger.log("Canceling node...")
        self.node_status.cancelled = True
        self.logger.log("Node cancelled.")
        return True


if __name__ == "__main__":
    openarm_node = OpenArmNode()
    openarm_node.start_node()