"""Process A. Publish mock odom, joint states, and TF on domain 90 only."""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hardware_safety.mock_path import wheel_samples  # noqa: E402
from hardware_safety.session import OdomSession, framed_outputs, load_config, mock_domain_block  # noqa: E402


def _domain() -> int | None:
    raw = os.environ.get("ROS_DOMAIN_ID")
    if raw in (None, ""):
        return None
    return int(raw)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--exit-after", type=float, default=3.0)
    args = parser.parse_args()
    config = load_config(ROOT)
    domain = _domain()
    reason = mock_domain_block(domain, config)
    if reason:
        print(json.dumps({"ready": False, "node_created": False, "reason": reason, "port_opened": False}))
        return 2

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from tf2_ros import TransformBroadcaster

    frames = framed_outputs(config, "pinky1")
    rclpy.init()
    node = Node("pinky1_odom_publisher")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    odom_pub = node.create_publisher(Odometry, frames["odom_topic"], qos)
    joint_pub = node.create_publisher(JointState, frames["joint_topic"], qos)
    broadcaster = TransformBroadcaster(node)
    session = OdomSession("ODOM_MOCK", 90, config)
    session.arm({"encoder_joints"})
    print(json.dumps({"ready": True, "node_created": True, "domain": domain, "port_opened": False}), flush=True)

    def publish() -> None:
        stamp = node.get_clock().now().to_msg()
        half = session.theta / 2.0
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = frames["parent"]
        odom.child_frame_id = frames["child"]
        odom.pose.pose.position.x = session.x
        odom.pose.pose.position.y = session.y
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x = session.measured_vx
        odom.twist.twist.angular.z = session.measured_wz
        joint = JointState()
        joint.header.stamp = stamp
        joint.name = list(frames["joints"])
        joint.position = list(session.joint_state()["position"])
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = frames["parent"]
        transform.child_frame_id = frames["child"]
        transform.transform.translation.x = session.x
        transform.transform.translation.y = session.y
        transform.transform.rotation.z = math.sin(half)
        transform.transform.rotation.w = math.cos(half)
        odom_pub.publish(odom)
        joint_pub.publish(joint)
        broadcaster.sendTransform(transform)

    started = time.monotonic()
    try:
        for clock, left, right in wheel_samples():
            if not session.push_wheels(left, right, clock):
                print(json.dumps({"ready": True, "passed": False, "reason": session.last_reject}))
                return 3
            publish()
            spin_until = time.monotonic() + 0.05
            while time.monotonic() < spin_until:
                rclpy.spin_once(node, timeout_sec=0.01)
        while time.monotonic() - started < args.exit_after:
            publish()
            rclpy.spin_once(node, timeout_sec=0.05)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
