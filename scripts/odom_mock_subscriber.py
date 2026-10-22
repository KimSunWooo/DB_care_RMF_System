"""Process B. Subscribe on domain 90 and check odom, joints, and TF."""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hardware_safety.session import framed_outputs, load_config, mock_domain_block  # noqa: E402


def _domain() -> int | None:
    raw = os.environ.get("ROS_DOMAIN_ID")
    if raw in (None, ""):
        return None
    return int(raw)


def _yaw(message) -> float:
    quat = message.pose.pose.orientation
    return math.atan2(2.0 * quat.w * quat.z, 1.0 - 2.0 * quat.z * quat.z)


def main() -> int:
    config = load_config(ROOT)
    domain = _domain()
    reason = mock_domain_block(domain, config)
    report = {"node_created": False, "port_opened": False, "domain": domain}
    if reason:
        report["reason"] = reason
        print(json.dumps(report, ensure_ascii=False))
        return 2

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from rclpy.time import Time
    from sensor_msgs.msg import JointState
    from tf2_ros import Buffer, TransformException, TransformListener

    frames = framed_outputs(config, "pinky1")
    rclpy.init()
    report["node_created"] = True
    node = Node("pinky1_odom_subscriber")
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
    state = {
        "odom": None,
        "joint": None,
        "stamps": [],
        "max_x": 0.0,
        "max_abs_yaw": 0.0,
        "saw_forward_twist": False,
        "saw_turn_twist": False,
        "saw_zero_twist": False,
        "straight_before_turn": False,
        "pose_match": False,
        "tf_ok": False,
        "last_rx": None,
        "lost": False,
        "reconnected": False,
    }
    buffer = Buffer()
    TransformListener(buffer, node)

    def on_odom(message: Odometry) -> None:
        now = time.monotonic()
        if state["lost"] and state["last_rx"] is not None and now - state["last_rx"] > 0.4:
            state["reconnected"] = True
        yaw = _yaw(message)
        state["max_x"] = max(state["max_x"], message.pose.pose.position.x)
        state["max_abs_yaw"] = max(state["max_abs_yaw"], abs(yaw))
        if message.pose.pose.position.x > 0.01 and abs(yaw) < 0.02:
            state["straight_before_turn"] = True
        if abs(message.twist.twist.linear.x) > 0.01 and abs(message.twist.twist.angular.z) < 0.05:
            state["saw_forward_twist"] = True
        if abs(message.twist.twist.angular.z) > 0.05:
            state["saw_turn_twist"] = True
        if abs(message.twist.twist.linear.x) < 1e-6 and state["max_x"] > 0.01:
            state["saw_zero_twist"] = True
        if message.header.stamp.sec or message.header.stamp.nanosec:
            state["stamps"].append(message.header.stamp.sec + message.header.stamp.nanosec * 1e-9)
        state["odom"] = message
        state["last_rx"] = now
        if not state.get("types_captured"):
            state["types"] = dict(node.get_topic_names_and_types())
            infos = node.get_publishers_info_by_topic(frames["odom_topic"])
            if infos:
                state["reliability"] = str(infos[0].qos_profile.reliability)
            state["types_captured"] = True
        try:
            transform = buffer.lookup_transform(frames["parent"], frames["child"], Time())
        except TransformException:
            return
        state["tf_ok"] = True
        error = abs(transform.transform.translation.x - message.pose.pose.position.x)
        error += abs(transform.transform.translation.y - message.pose.pose.position.y)
        if error < 1e-3:
            state["pose_match"] = True

    def on_joint(message: JointState) -> None:
        state["joint"] = message

    node.create_subscription(Odometry, frames["odom_topic"], on_odom, qos)
    node.create_subscription(JointState, frames["joint_topic"], on_joint, qos)
    deadline = time.monotonic() + 12.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if state["last_rx"] is not None and time.monotonic() - state["last_rx"] > 0.45 and state["max_x"] > 0.01:
                state["lost"] = True
            if state["reconnected"] and state["pose_match"] and state["saw_turn_twist"] and state["straight_before_turn"]:
                break
        names = []
        types = state.get("types") or {}
        reliability = state.get("reliability") or ""
        joint = state["joint"]
        if joint is not None:
            names = list(joint.name)
        periods = []
        stamps = state["stamps"]
        for earlier, later in zip(stamps, stamps[1:]):
            delta = later - earlier
            if 0 < delta <= 0.2:
                periods.append(delta)
        mean_period = sum(periods) / len(periods) if periods else None
        report.update({
            "odom_type": types.get(frames["odom_topic"], []),
            "joint_type": types.get(frames["joint_topic"], []),
            "tf_type": types.get("/tf", []),
            "reliability": reliability,
            "mean_period_sec": mean_period,
            "max_x": state["max_x"],
            "max_abs_yaw": state["max_abs_yaw"],
            "straight_before_turn": state["straight_before_turn"],
            "saw_forward_twist": state["saw_forward_twist"],
            "saw_turn_twist": state["saw_turn_twist"],
            "saw_zero_twist": state["saw_zero_twist"],
            "joint_names": names,
            "tf_ok": state["tf_ok"],
            "pose_match": state["pose_match"],
            "publisher_lost": state["lost"],
            "reconnected": state["reconnected"],
            "port_opened": False,
        })
        reliable = reliability in {"1", "ReliabilityPolicy.RELIABLE"} or "RELIABLE" in reliability
        report["passed"] = bool(
            report["odom_type"] == ["nav_msgs/msg/Odometry"]
            and report["joint_type"] == ["sensor_msgs/msg/JointState"]
            and "tf2_msgs/msg/TFMessage" in report["tf_type"]
            and reliable
            and mean_period is not None
            and 0.02 <= mean_period <= 0.15
            and state["straight_before_turn"]
            and state["saw_forward_twist"]
            and state["saw_turn_twist"]
            and state["max_abs_yaw"] > 0.05
            and names == frames["joints"]
            and state["tf_ok"]
            and state["pose_match"]
            and state["lost"]
            and state["reconnected"]
        )
        print(json.dumps(report, ensure_ascii=False))
        return 0 if report["passed"] else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
