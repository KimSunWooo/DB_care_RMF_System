"""Send one short NavigateToPose goal to the virtual pinky1.

Refuses the goal unless the action server, a real scan, and map->base TF exist.
Does not call the fleet adapter or any elevator interface.
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from sim.gazebo.room_geometry import GOAL_XY, point_occupied

ACTION = "/pinky1/navigate_to_pose"
BASE = "pinky1/base_footprint"
MAP = "map"
ODOM = "/pinky1/odom"
SCAN = "/pinky1/scan"
TOLERANCE_M = 0.25


class GoalCheck(Node):
    def __init__(self) -> None:
        super().__init__(
            "pinky1_nav_goal",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            cli_args=[
                "--ros-args",
                "--remap", "/tf:=/pinky1/tf",
                "--remap", "/tf_static:=/pinky1/tf_static",
            ],
        )
        self.scan: LaserScan | None = None
        self.odom: Odometry | None = None
        self.feedback: list[float] = []
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(LaserScan, SCAN, self._scan, qos)
        self.create_subscription(Odometry, ODOM, self._odom, 20)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(self, NavigateToPose, ACTION)

    def _scan(self, msg: LaserScan) -> None:
        self.scan = msg

    def _odom(self, msg: Odometry) -> None:
        self.odom = msg

    def scan_ok(self) -> bool:
        if self.scan is None or self.scan.header.frame_id != "pinky1/rplidar_link":
            return False
        finite = [value for value in self.scan.ranges if math.isfinite(value)]
        return len(finite) >= 100 and min(finite) <= 6.0

    def lookup(self) -> tuple[float, float] | None:
        try:
            transform = self.tf_buffer.lookup_transform(MAP, BASE, rclpy.time.Time())
        except TransformException:
            return None
        t = transform.transform.translation
        return t.x, t.y


def spin_until(node: GoalCheck, predicate, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if predicate():
            return True
    return False


def main() -> int:
    goal_xy = GOAL_XY
    if len(sys.argv) == 3:
        goal_xy = (float(sys.argv[1]), float(sys.argv[2]))
    if point_occupied(*goal_xy):
        print(json.dumps({"passed": False, "reason": "goal cell is occupied", "goal_xy": list(goal_xy)}))
        return 2
    rclpy.init()
    node = GoalCheck()
    report: dict = {
        "action": ACTION,
        "goal_xy": list(goal_xy),
        "goal_sent": False,
        "passed": False,
    }
    try:
        ready = spin_until(
            node,
            lambda: node.client.server_is_ready() and node.scan_ok() and node.lookup() is not None and node.odom is not None,
            90.0,
        )
        start = node.lookup()
        report["action_ready"] = node.client.server_is_ready()
        report["scan_ok"] = node.scan_ok()
        report["start_xy"] = list(start) if start else None
        if not ready or start is None:
            report["reason"] = "action, scan, or map TF was not ready"
            print(json.dumps(report, ensure_ascii=False))
            return 2
        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = MAP
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = goal_xy[0]
        pose.pose.position.y = goal_xy[1]
        pose.pose.orientation.w = 1.0
        goal.pose = pose
        send = node.client.send_goal_async(goal, feedback_callback=lambda fb: node.feedback.append(float(fb.feedback.distance_remaining)))
        rclpy.spin_until_future_complete(node, send, timeout_sec=30.0)
        handle = send.result()
        report["goal_sent"] = True
        report["accepted"] = bool(handle is not None and handle.accepted)
        if not report["accepted"]:
            report["reason"] = "NavigateToPose goal was rejected"
            print(json.dumps(report, ensure_ascii=False))
            return 3
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future, timeout_sec=180.0)
        result = result_future.result()
        status = None if result is None else int(result.status)
        report["status"] = status
        report["feedback_samples"] = len(node.feedback)
        report["feedback_last_distance"] = node.feedback[-1] if node.feedback else None
        settle_deadline = time.monotonic() + 2.5
        while time.monotonic() < settle_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        end = node.lookup()
        report["end_xy"] = list(end) if end else None
        if end is None or node.odom is None:
            report["reason"] = "final pose missing"
            print(json.dumps(report, ensure_ascii=False))
            return 4
        error = math.hypot(end[0] - goal_xy[0], end[1] - goal_xy[1])
        report["position_error_m"] = error
        report["stop_vx"] = node.odom.twist.twist.linear.x
        report["stop_wz"] = node.odom.twist.twist.angular.z
        report["moved_m"] = math.hypot(end[0] - start[0], end[1] - start[1])
        report["passed"] = bool(status == 4 and error <= TOLERANCE_M and abs(report["stop_vx"]) < 0.05 and report["moved_m"] > 0.5)
        if not report["passed"]:
            report["reason"] = "goal result, arrival error, or stop velocity failed"
        print(json.dumps(report, ensure_ascii=False))
        return 0 if report["passed"] else 5
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
