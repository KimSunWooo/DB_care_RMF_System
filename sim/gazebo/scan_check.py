"""Check that /pinky1/scan carries real LaserScan ranges. Does not publish cmd_vel."""
from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

TOPIC = "/pinky1/scan"
FRAME = "pinky1/rplidar_link"


class ScanCheck(Node):
    def __init__(self) -> None:
        super().__init__(
            "pinky1_scan_check",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.messages: list[LaserScan] = []
        self.create_subscription(LaserScan, TOPIC, self.messages.append, qos)

    def summary(self) -> dict:
        latest = self.messages[-1]
        ranges = list(latest.ranges)
        finite = [value for value in ranges if math.isfinite(value)]
        stamps = [
            float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
            for msg in self.messages
        ]
        period = None
        if len(stamps) >= 2 and stamps[-1] > stamps[0]:
            period = (stamps[-1] - stamps[0]) / (len(stamps) - 1)
        return {
            "topic": TOPIC,
            "type": "sensor_msgs/msg/LaserScan",
            "messages": len(self.messages),
            "frame_id": latest.header.frame_id,
            "range_count": len(ranges),
            "finite_count": len(finite),
            "nan_count": sum(math.isnan(value) for value in ranges),
            "inf_count": sum(math.isinf(value) for value in ranges),
            "finite_min_m": min(finite) if finite else None,
            "finite_max_m": max(finite) if finite else None,
            "angle_min": latest.angle_min,
            "angle_max": latest.angle_max,
            "mean_period_sec": period,
        }


def main() -> int:
    rclpy.init()
    node = ScanCheck()
    deadline = time.monotonic() + 20.0
    try:
        while time.monotonic() < deadline and len(node.messages) < 5:
            rclpy.spin_once(node, timeout_sec=0.2)
        if len(node.messages) < 2:
            print(json.dumps({"passed": False, "reason": "fewer than 2 scan messages", "messages": len(node.messages)}))
            return 2
        report = node.summary()
        finite = report["finite_count"]
        period = report["mean_period_sec"]
        report["passed"] = bool(
            report["frame_id"] == FRAME
            and finite >= 100
            and report["finite_min_m"] is not None
            and 0.05 <= report["finite_min_m"] <= 6.0
            and report["finite_max_m"] > 1.0
            and period is not None
            and period < 0.5
        )
        if not report["passed"]:
            report["reason"] = "scan contents did not match the lidar contract"
        print(json.dumps(report, ensure_ascii=False))
        return 0 if report["passed"] else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
