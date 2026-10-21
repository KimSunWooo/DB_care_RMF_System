"""Copy global /tf and /tf_static onto /pinky1/tf for a namespaced Nav2 stack.

Gazebo and robot_state_publisher publish the un-namespaced TF topics.
Nav2 bringup remaps /tf to the robot namespace. This relay does not invent frames.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.parameter import Parameter
from tf2_msgs.msg import TFMessage


class TfRelay(Node):
    def __init__(self) -> None:
        super().__init__(
            "pinky1_tf_relay",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        static_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._tf_pub = self.create_publisher(TFMessage, "/pinky1/tf", 100)
        self._static_pub = self.create_publisher(TFMessage, "/pinky1/tf_static", static_qos)
        self.create_subscription(TFMessage, "/tf", self._tf_pub.publish, 100)
        self.create_subscription(TFMessage, "/tf_static", self._static_pub.publish, static_qos)


def main() -> None:
    rclpy.init()
    node = TfRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
