"""Publish mock odom on ROS domain 90 and check the messages in-process.

Exits before rclpy.init when the domain is not 90. Does not open a motor port
and does not subscribe to the Pinky domain.
"""
from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hardware_safety.session import OdomSession, framed_outputs, load_config, mock_domain_block  # noqa: E402


def _domain() -> int | None:
    raw = os.environ.get("ROS_DOMAIN_ID")
    if raw in (None, ""):
        return None
    return int(raw)


def _yaw(message) -> float:
    q = message.pose.pose.orientation
    return math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)


def main() -> int:
    config = load_config(ROOT)
    domain = _domain()
    reason = mock_domain_block(domain, config)
    report = {
        "node_created": False,
        "published": False,
        "port_opened": False,
        "domain": domain,
    }
    if reason:
        report["reason"] = reason
        print(json.dumps(report, ensure_ascii=False))
        return 2

    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from tf2_msgs.msg import TFMessage

    frames = framed_outputs(config, "pinky1")
    rclpy.init()
    report["node_created"] = True
    node = Node("pinky1_odom_mock")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    odom_pub = node.create_publisher(Odometry, frames["odom_topic"], qos)
    joint_pub = node.create_publisher(JointState, frames["joint_topic"], qos)
    tf_pub = node.create_publisher(TFMessage, frames["tf_topic"], qos)
    seen: dict = {}
    node.create_subscription(Odometry, frames["odom_topic"], lambda msg: seen.__setitem__("odom", msg), qos)
    node.create_subscription(JointState, frames["joint_topic"], lambda msg: seen.__setitem__("joint", msg), qos)
    node.create_subscription(TFMessage, frames["tf_topic"], lambda msg: seen.__setitem__("tf", msg), qos)
    session = OdomSession("ODOM_MOCK", 90, config)
    session.arm({"encoder_joints"})

    def publish(now: float) -> None:
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
        tf_message = TFMessage(transforms=[transform])
        odom_pub.publish(odom)
        joint_pub.publish(joint)
        tf_pub.publish(tf_message)
        session.last_reject = ""

    try:
        session.push_wheels(0, 0, 0.0)
        session.push_command(0.1, 0.0, 0.0)
        session.push_wheels(80, 80, 0.05)
        publish(0.05)
        for _ in range(40):
            rclpy.spin_once(node, timeout_sec=0.05)
            if "odom" in seen and "joint" in seen and "tf" in seen:
                break
        forward = seen.get("odom")
        turned_ok = False
        if forward is not None and forward.pose.pose.position.x > 0.0:
            session.push_wheels(160, 0, 0.10)
            publish(0.10)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.05)
                current = seen.get("odom")
                if current is not None and abs(_yaw(current)) > 1e-6:
                    turned_ok = True
                    break
        before_stop = None if seen.get("odom") is None else (
            seen["odom"].header.stamp.sec,
            seen["odom"].header.stamp.nanosec,
        )
        session.poll(1.0)
        publish(1.0)
        stopped = None
        for _ in range(20):
            rclpy.spin_once(node, timeout_sec=0.05)
            current = seen.get("odom")
            if current is None:
                continue
            stamp = (current.header.stamp.sec, current.header.stamp.nanosec)
            if stamp != before_stop and abs(current.twist.twist.linear.x) < 1e-9:
                stopped = current
                break
        pose_before = session.x
        session.request_recovery()
        session.push_wheels(0, 0, 1.2)
        rejected = session.push_wheels(2048, 0, 1.2 + session.config.nominal_period_sec) is False
        publish(1.3)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
        final = seen.get("odom")
        tf_message = seen.get("tf")
        joint = seen.get("joint")
        same_stamp = False
        tf_frames = False
        if final is not None and tf_message is not None and tf_message.transforms:
            transform = tf_message.transforms[0]
            same_stamp = (
                final.header.stamp.sec == transform.header.stamp.sec
                and final.header.stamp.nanosec == transform.header.stamp.nanosec
            )
            tf_frames = (
                transform.header.frame_id == frames["parent"]
                and transform.child_frame_id == frames["child"]
            )
        report.update({
            "published": forward is not None,
            "forward_x": None if forward is None else forward.pose.pose.position.x,
            "turned": turned_ok,
            "timeout_vx": None if stopped is None else stopped.twist.twist.linear.x,
            "timeout_state": session.state,
            "spike_rejected": rejected,
            "pose_held": final is not None and abs(final.pose.pose.position.x - pose_before) < 1e-9,
            "stamp_match": same_stamp,
            "tf_frames": tf_frames,
            "joint_names": [] if joint is None else list(joint.name),
            "publisher_count": 3,
            "port_opened": session.motor_port_open,
            "physical_stop_confirmed": session.physical_stop_confirmed,
        })
        report["passed"] = bool(
            report["published"]
            and report["forward_x"] > 0.0
            and turned_ok
            and report["timeout_vx"] == 0.0
            and rejected
            and report["pose_held"]
            and same_stamp
            and tf_frames
            and report["joint_names"] == frames["joints"]
            and session.motor_port_open is False
        )
        print(json.dumps(report, ensure_ascii=False))
        return 0 if report["passed"] else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
