"""Publish encoder odometry on domain 53. This process does not write the bus."""
from __future__ import annotations

import json
import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hardware_safety.real_driver import RealPinkyDriver
from hardware_safety.session import load_config
from hardware_safety.xl330_bus import Xl330ReadBus


def _yaw_quat(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") != "53":
        print(json.dumps({"started": False, "reason": "odom publisher refuses every domain except 53"}))
        return 2
    bus = Xl330ReadBus()
    driver = RealPinkyDriver(bus, load_config(ROOT))
    reason = driver.identify()
    if reason:
        print(json.dumps({"started": False, "reason": reason, "refused": bus.refused}))
        return 3
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from tf2_ros import TransformBroadcaster

    rclpy.init()
    node = Node("real_pinky1_encoder_odom")
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    broadcaster = TransformBroadcaster(node)
    started = time.monotonic()
    driver.rebase_clock(started)
    poses = []
    try:
        while time.monotonic() - started < 4.0:
            now = time.monotonic()
            samples = bus.read_samples()
            if not driver.push_feedback(samples, now):
                break
            stamp = node.get_clock().now().to_msg()
            pose = driver.tf()
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = "odom"
            message.child_frame_id = "base_footprint"
            message.pose.pose.position.x = pose["x"]
            message.pose.pose.position.y = pose["y"]
            qx, qy, qz, qw = _yaw_quat(pose["yaw"])
            message.pose.pose.orientation.x = qx
            message.pose.pose.orientation.y = qy
            message.pose.pose.orientation.z = qz
            message.pose.pose.orientation.w = qw
            message.twist.twist.linear.x = driver.odom.measured_vx
            message.twist.twist.angular.z = driver.odom.measured_wz
            odom_pub.publish(message)
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = "odom"
            transform.child_frame_id = "base_footprint"
            transform.transform.translation.x = pose["x"]
            transform.transform.translation.y = pose["y"]
            transform.transform.rotation.x = qx
            transform.transform.rotation.y = qy
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            broadcaster.sendTransform(transform)
            poses.append({"t": round(now - started, 3), "x": pose["x"], "y": pose["y"], "yaw": pose["yaw"]})
            time.sleep(0.05)
    finally:
        bus.close()
        node.destroy_node()
        rclpy.shutdown()
    moved = math.hypot(poses[-1]["x"] - poses[0]["x"], poses[-1]["y"] - poses[0]["y"]) if len(poses) > 1 else 0.0
    print(json.dumps({
        "started": True,
        "writes": 0,
        "refused": bus.refused,
        "torque": [sample.torque for sample in driver.samples],
        "operating_mode": [sample.operating_mode for sample in driver.samples],
        "voltage_0_1v": bus.voltage,
        "samples": len(poses),
        "moved_m": moved,
        "final": poses[-1] if poses else None,
        "state": driver.state,
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
