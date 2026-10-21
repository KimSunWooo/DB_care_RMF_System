"""Limited forward and stop check for the virtual pinky1 on the sim domain.

Publishes geometry_msgs/Twist on /pinky1/cmd_vel only. Does not call Nav2,
the fleet adapter, or any elevator interface.
"""
from __future__ import annotations

import json
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

CMD_TOPIC = "/pinky1/cmd_vel"
ODOM_TOPIC = "/pinky1/odom"
CLOCK_TOPIC = "/clock"
TF_TOPIC = "/tf"
LINEAR_X = 0.12
FORWARD_SEC = 2.0
STOP_SEC = 2.0
MIN_FORWARD_M = 0.08


class MotionCheck(Node):
    def __init__(self) -> None:
        super().__init__(
            "pinky1_gz_motion_check",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.clock: Clock | None = None
        self.odom: Odometry | None = None
        self.tf_count = 0
        self.create_subscription(Clock, CLOCK_TOPIC, self._on_clock, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 20)
        self.create_subscription(TFMessage, TF_TOPIC, self._on_tf, 50)
        self.cmd = self.create_publisher(Twist, CMD_TOPIC, 10)

    def _on_clock(self, msg: Clock) -> None:
        self.clock = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom = msg

    def _on_tf(self, msg: TFMessage) -> None:
        if msg.transforms:
            self.tf_count += 1

    def sim_sec(self) -> float | None:
        if self.clock is None:
            return None
        return float(self.clock.clock.sec) + float(self.clock.clock.nanosec) * 1e-9


def wait_until(node: MotionCheck, predicate, timeout_sec: float) -> bool:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if predicate():
            return True
    return False


def publish_for(node: MotionCheck, twist: Twist, seconds: float) -> None:
    start = time.monotonic()
    while time.monotonic() - start < seconds:
        node.cmd.publish(twist)
        rclpy.spin_once(node, timeout_sec=0.05)


def pose(msg: Odometry) -> tuple[float, float, float]:
    p = msg.pose.pose.position
    v = msg.twist.twist.linear
    return p.x, p.y, v.x


def main() -> int:
    rclpy.init()
    node = MotionCheck()
    report: dict = {
        "cmd_vel_topic": CMD_TOPIC,
        "cmd_vel_type": "geometry_msgs/msg/Twist",
        "odom_topic": ODOM_TOPIC,
        "clock_seen": False,
        "odom_seen": False,
        "tf_seen": False,
        "forward_sent": False,
        "stop_sent": False,
        "passed": False,
    }
    try:
        report["clock_seen"] = wait_until(node, lambda: node.clock is not None, 30.0)
        report["odom_seen"] = wait_until(node, lambda: node.odom is not None, 40.0)
        report["tf_seen"] = wait_until(node, lambda: node.tf_count > 0, 15.0)
        if not (report["clock_seen"] and report["odom_seen"]):
            report["reason"] = "clock or odom did not arrive"
            print(json.dumps(report, ensure_ascii=False))
            return 2
        start = pose(node.odom)
        report["start_x"] = start[0]
        report["start_y"] = start[1]
        report["start_sim_sec"] = node.sim_sec()
        forward = Twist()
        forward.linear.x = LINEAR_X
        publish_for(node, forward, FORWARD_SEC)
        report["forward_sent"] = True
        wait_until(node, lambda: False, 0.4)
        moved = pose(node.odom)
        report["moved_x"] = moved[0]
        report["moved_y"] = moved[1]
        report["moved_vx"] = moved[2]
        dx = moved[0] - start[0]
        dy = moved[1] - start[1]
        report["dx"] = dx
        report["dy"] = dy
        report["distance_m"] = math.hypot(dx, dy)
        stop = Twist()
        publish_for(node, stop, STOP_SEC)
        report["stop_sent"] = True
        wait_until(node, lambda: False, 0.5)
        stopped = pose(node.odom)
        hold_deadline = time.monotonic() + 1.0
        while time.monotonic() < hold_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        hold = pose(node.odom) if node.odom is not None else stopped
        report["stop_x"] = hold[0]
        report["stop_y"] = hold[1]
        report["stop_vx"] = hold[2]
        report["stop_drift_m"] = math.hypot(hold[0] - stopped[0], hold[1] - stopped[1])
        report["end_sim_sec"] = node.sim_sec()
        report["tf_messages"] = node.tf_count
        moved_forward = dx > MIN_FORWARD_M and abs(dy) < 0.2
        stopped_ok = abs(hold[2]) < 0.05 and report["stop_drift_m"] < 0.05
        report["passed"] = bool(moved_forward and stopped_ok and report["tf_seen"])
        if not report["passed"]:
            report["reason"] = "forward or stop criterion failed"
        print(json.dumps(report, ensure_ascii=False))
        return 0 if report["passed"] else 3
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
