#!/usr/bin/env python3
"""
Relay /joint_trajectory -> left and right hardware controller topics.
FIXED: Excludes gripper joints from trajectory (only 7 arm joints per controller).
Gripper commands are handled by separate /left_gripper_controller and /right_gripper_controller.

TeleopXR publishes a single /joint_trajectory with all joints (7 left + 7 right + 2 grippers = 16).
ros2_control expects:
  /left_joint_trajectory_controller/joint_trajectory (7 arm joints ONLY)
  /right_joint_trajectory_controller/joint_trajectory (7 arm joints ONLY)

Run with:
  source /opt/ros/jazzy/setup.bash
  source /home/air-lab-ncsu/OPEN_ARM/packages/install/setup.bash
  python3 scripts/joint_trajectory_relay.py
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class JointTrajectoryRelay(Node):
    def __init__(self):
        super().__init__("joint_trajectory_relay")

        self.left_pub = self.create_publisher(
            JointTrajectory,
            "/left_joint_trajectory_controller/joint_trajectory",
            10,
        )
        self.right_pub = self.create_publisher(
            JointTrajectory,
            "/right_joint_trajectory_controller/joint_trajectory",
            10,
        )

        self.create_subscription(
            JointTrajectory,
            "/joint_trajectory",
            self._cb,
            10,
        )

        self._recv_count = 0
        self.get_logger().info("Relay ready: /joint_trajectory -> left + right controllers (7 joints each, grippers excluded)")

    def _cb(self, msg: JointTrajectory):
        self._recv_count += 1

        left_indices = []
        right_indices = []

        # Extract indices for arm joints ONLY (skip gripper joints)
        for i, name in enumerate(msg.joint_names):
            # Skip gripper joints - they go to separate gripper controllers
            if "finger_joint" in name:
                continue
            if name.startswith("openarm_left_"):
                left_indices.append(i)
            elif name.startswith("openarm_right_"):
                right_indices.append(i)

        # Log incoming message on first receive and every 100 after
        if self._recv_count == 1 or self._recv_count % 100 == 0:
            self.get_logger().info(
                f"[#{self._recv_count}] Received {len(msg.joint_names)} joints (16 total) | "
                f"left_arm={len(left_indices)} right_arm={len(right_indices)}"
            )
            self.get_logger().info(f"  All joints: {msg.joint_names}")
            if msg.points:
                pos = [round(float(p), 4) for p in msg.points[0].positions]
                self.get_logger().info(f"  Positions:  {pos}")

        # Publish to trajectory controllers (arm joints only)
        for indices, pub, side in [
            (left_indices, self.left_pub, "left"),
            (right_indices, self.right_pub, "right"),
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

            if self._recv_count == 1 or self._recv_count % 100 == 0:
                self.get_logger().info(
                    f"  -> {side}_traj: {out.joint_names} | "
                    f"pos={[round(float(p), 4) for p in out.points[0].positions]}"
                )


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
