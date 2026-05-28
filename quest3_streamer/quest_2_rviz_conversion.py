import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped

class QuestToTF(Node):
    def __init__(self):
        super().__init__('quest_to_tf')
        self.br = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info("QuestToTF node started!")

        self.create_subscription(
            PoseStamped,
            '/quest/right_hand/pose',
            self.cb_right,
            10
        )
        self.create_subscription(
            PoseStamped,
            '/quest/left_hand/pose',
            self.cb_left,
            10
        )

    def convert(self, msg, child_frame):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "ros_world"
        t.child_frame_id = child_frame

        t.transform.translation.x = msg.pose.position.z
        t.transform.translation.y = -msg.pose.position.x
        t.transform.translation.z = msg.pose.position.y
        t.transform.rotation = msg.pose.orientation
        return t

    def cb_right(self, msg):
        self.get_logger().info(f"Received RIGHT hand pose → broadcasting TF")
        self.br.sendTransform(self.convert(msg, "right_hand"))

    def cb_left(self, msg):
        self.get_logger().info(f"Received LEFT hand pose → broadcasting TF")
        self.br.sendTransform(self.convert(msg, "left_hand"))


def main():
    rclpy.init()
    node = QuestToTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
