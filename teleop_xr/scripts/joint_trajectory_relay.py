#!/usr/bin/env python3
"""
Relay /joint_trajectory -> left/right arm trajectory controllers.

TeleopXR publishes: /joint_trajectory with 16 joints (7L arm + 7R arm + 2 grippers)

This relay:
1. Splits trajectories: 7 left arm  → /left_joint_trajectory_controller/joint_trajectory
2. Splits trajectories: 7 right arm → /right_joint_trajectory_controller/joint_trajectory
3. Strips finger joints from both trajectories (gripper is controlled directly
   by demo_with_ros2.py via /left_gripper_controller/gripper_cmd action server).

NOTE: Gripper forwarding was intentionally removed. demo_with_ros2.py owns gripper
control via direct GripperCommand action calls on trigger press/release. Having the
relay ALSO send gripper commands from the joint trajectory caused a conflict:
the relay sent finger_joint=0.044 (from the IK viz state) at max_effort=10 N right
after demo_with_ros2.py sent position=0.12 at max_effort=5 N, causing the gripper
to stop at half-open on the first trigger press.

Run with:
  source /opt/ros/jazzy/setup.bash
  source ~/OPEN_ARM/packages/install/setup.bash
  python3 scripts/joint_trajectory_relay.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryRelay(Node):
    def __init__(self):
        super().__init__("joint_trajectory_relay")

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # ARM TRAJECTORY PUBLISHERS
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.left_traj_pub = self.create_publisher(
            JointTrajectory,
            "/left_joint_trajectory_controller/joint_trajectory",
            10,
        )
        self.right_traj_pub = self.create_publisher(
            JointTrajectory,
            "/right_joint_trajectory_controller/joint_trajectory",
            10,
        )

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # TRAJECTORY SUBSCRIBER
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        self.create_subscription(
            JointTrajectory,
            "/joint_trajectory",
            self._on_trajectory,
            10,
        )

        self._recv_count = 0

        self.get_logger().info(
            "✅ Relay READY:\n"
            "   ARM TRAJECTORIES: /joint_trajectory → left + right controllers (7 joints each)\n"
            "   GRIPPERS: stripped — controlled directly by demo_with_ros2.py"
        )

    def _on_trajectory(self, msg: JointTrajectory):
        """Process incoming trajectory message."""
        self._recv_count += 1

        left_arm_indices = []
        right_arm_indices = []
        left_gripper_idx = None
        right_gripper_idx = None

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # EXTRACT JOINT INDICES
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        for i, name in enumerate(msg.joint_names):
            # Gripper joints
            if "finger_joint" in name:
                if "left" in name:
                    left_gripper_idx = i
                elif "right" in name:
                    right_gripper_idx = i
                continue  # Skip gripper from trajectory

            # Arm joints
            if name.startswith("openarm_left_"):
                left_arm_indices.append(i)
            elif name.startswith("openarm_right_"):
                right_arm_indices.append(i)

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # LOG FIRST MESSAGE
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        if self._recv_count == 1:
            self.get_logger().info(
                f"[FIRST MESSAGE] Received {len(msg.joint_names)} total joints"
            )
            self.get_logger().info(f"  Left arm: {len(left_arm_indices)} joints")
            self.get_logger().info(f"  Right arm: {len(right_arm_indices)} joints")
            self.get_logger().info(f"  Left gripper index: {left_gripper_idx}")
            self.get_logger().info(f"  Right gripper index: {right_gripper_idx}")
            self.get_logger().info(f"  Joint names: {msg.joint_names}")

        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        # FORWARD ARM TRAJECTORIES
        # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
        if msg.points:
            for indices, pub, side in [
                (left_arm_indices, self.left_traj_pub, "left"),
                (right_arm_indices, self.right_traj_pub, "right"),
            ]:
                if not indices:
                    continue

                out = JointTrajectory()
                out.header = msg.header
                out.joint_names = [msg.joint_names[i] for i in indices]

                for pt in msg.points:
                    new_pt = JointTrajectoryPoint()
                    new_pt.time_from_start = pt.time_from_start
                    if pt.positions:
                        new_pt.positions = [pt.positions[i] for i in indices]
                    if pt.velocities:
                        new_pt.velocities = [pt.velocities[i] for i in indices]
                    if pt.accelerations:
                        new_pt.accelerations = [pt.accelerations[i] for i in indices]
                    out.points.append(new_pt)

                pub.publish(out)


def main():
    rclpy.init()
    node = JointTrajectoryRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
