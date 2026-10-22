"""Domain 42 Nav2 gateway for the Gazebo pinky1 session.

Listens on 127.0.0.1:17442. Does not publish /cmd_vel. Refuses every domain
except 42 and refuses to start if a Pinky serial device is present.
"""
from __future__ import annotations

import json
import math
import os
import queue
import socket
import sys
import threading
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fleet_adapter.live_nav import GoalTracker
from fleet_adapter.sim_graph import GraphError, load_sim_session
from sim.gazebo.room_geometry import point_occupied

HOST = "127.0.0.1"
PORT = 17442
ACTION = "/pinky1/navigate_to_pose"
MAP = "map"
BASE = "pinky1/base_footprint"


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class Gateway:
    def __init__(self) -> None:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import LaserScan
        from tf2_ros import Buffer, TransformException, TransformListener

        self.rclpy = rclpy
        self.TransformException = TransformException
        self.PoseStamped = PoseStamped
        self.NavigateToPose = NavigateToPose
        rclpy.init()
        self.node = Node(
            "gazebo_pinky1_nav_gateway",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
            cli_args=["--ros-args", "--remap", "/tf:=/pinky1/tf", "--remap", "/tf_static:=/pinky1/tf_static"],
        )
        self.scan = None
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.node.create_subscription(LaserScan, "/pinky1/scan", self._scan, qos)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self.node)
        self.client = ActionClient(self.node, NavigateToPose, ACTION)
        self.tracker = GoalTracker()
        self.goal_handle = None
        self.session = load_sim_session(ROOT)
        self.started = time.monotonic()

    def _scan(self, message) -> None:
        self.scan = message

    def spin_once(self) -> None:
        self.rclpy.spin_once(self.node, timeout_sec=0.05)

    def pose(self) -> tuple[float, float, float] | None:
        try:
            transform = self.buffer.lookup_transform(MAP, BASE, self.rclpy.time.Time())
        except self.TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            translation.x,
            translation.y,
            yaw_from_quat(rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def scan_ok(self) -> bool:
        message = self.scan
        if message is None or message.header.frame_id != "pinky1/rplidar_link":
            return False
        finite = [value for value in message.ranges if math.isfinite(value)]
        return len(finite) >= 100

    def clock_ok(self) -> bool:
        return self.node.get_clock().now().nanoseconds > 0

    def health(self) -> dict:
        pose = self.pose()
        return {
            "ok": True,
            "domain": 42,
            "action_ready": bool(self.client.server_is_ready()),
            "scan_ok": self.scan_ok(),
            "clock_ok": self.clock_ok(),
            "pose": list(pose) if pose else None,
            "pose_fresh": pose is not None,
            "hardware_port": False,
            "publishes_cmd_vel": False,
            "goal": self.tracker.snapshot(),
        }

    def _match_waypoint(self, name: str, x: float, y: float):
        if name:
            pose = self.session.nav_pose(name)
        else:
            pose = None
            for candidate in self.session.waypoints.values():
                if math.hypot(candidate.x - x, candidate.y - y) <= 0.05:
                    pose = self.session.nav_pose(candidate.name)
                    break
            if pose is None:
                raise GraphError("destination is not GZ_SPAWN or GZ_EAST")
        if math.hypot(pose.x - x, pose.y - y) > 0.05:
            raise GraphError("destination does not match the gazebo graph")
        if point_occupied(pose.x, pose.y):
            raise GraphError("destination cell is occupied")
        return pose

    def go(self, payload: dict) -> dict:
            if not self.client.server_is_ready() or not self.scan_ok() or not self.clock_ok():
                return {"ok": False, "reason": "Nav2 action, scan, or clock is not ready", "goal": self.tracker.snapshot()}
            if self.pose() is None:
                return {"ok": False, "reason": "map to base_footprint is missing", "goal": self.tracker.snapshot()}
            try:
                waypoint = self._match_waypoint(
                    str(payload.get("waypoint") or ""),
                    float(payload["x"]),
                    float(payload["y"]),
                )
            except (GraphError, KeyError, TypeError, ValueError) as exc:
                return {"ok": False, "reason": str(exc), "goal": self.tracker.snapshot()}
            generation = self.tracker.begin()
            if generation is None:
                return {"ok": False, "reason": "a Nav2 goal is already active", "goal": self.tracker.snapshot()}
            goal = self.NavigateToPose.Goal()
            pose = self.PoseStamped()
            pose.header.frame_id = MAP
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            qx, qy, qz, qw = quat_from_yaw(float(payload.get("yaw") or 0.0))
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            goal.pose = pose
            send = self.client.send_goal_async(goal)
            deadline = time.monotonic() + 20.0
            while not send.done() and time.monotonic() < deadline:
                self.spin_once()
            handle = send.result() if send.done() else None
            accepted = bool(handle is not None and handle.accepted)
            if not accepted:
                self.tracker.reject(generation)
                return {"ok": False, "reason": "NavigateToPose rejected the goal", "goal": self.tracker.snapshot()}
            goal_id = str(handle.goal_id.uuid)
            self.tracker.accept(generation, goal_id)
            self.goal_handle = handle
            result = handle.get_result_async()

            def _done(future, gen=generation) -> None:
                try:
                    wrapped = future.result()
                    status = None if wrapped is None else int(wrapped.status)
                except Exception:
                    status = None
                self.tracker.observe_status(gen, status)

            result.add_done_callback(_done)
            return {
                "ok": True,
                "phase": "accepted",
                "waypoint": waypoint.name,
                "x": waypoint.x,
                "y": waypoint.y,
                "goal": self.tracker.snapshot(),
            }

    def cancel(self) -> dict:
            generation = self.tracker.generation
            if not self.tracker.request_cancel(generation) or self.goal_handle is None:
                return {"ok": False, "reason": "no accepted goal to cancel", "goal": self.tracker.snapshot()}
            future = self.goal_handle.cancel_goal_async()
            deadline = time.monotonic() + 10.0
            while not future.done() and time.monotonic() < deadline:
                self.spin_once()
            return {
                "ok": True,
                "cancel_response": future.done(),
                "result_confirmed": False,
                "goal": self.tracker.snapshot(),
            }

    def handle(self, payload: dict) -> dict:
        op = payload.get("op")
        if op in {"health", "poll"}:
            body = self.health()
            body["op"] = op
            return body
        if op == "go":
            return self.go(payload)
        if op == "cancel":
            return self.cancel()
        return {"ok": False, "reason": "unknown op"}


def serve(commands: queue.Queue) -> None:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((HOST, PORT))
    server.listen(8)
    server.settimeout(0.2)
    print(json.dumps({"ready": True, "host": HOST, "port": PORT, "domain": 42}), flush=True)
    while True:
        try:
            conn, _addr = server.accept()
        except socket.timeout:
            continue
        threading.Thread(target=_client, args=(commands, conn), daemon=True).start()


def _client(commands: queue.Queue, conn: socket.socket) -> None:
    box = {"event": threading.Event(), "reply": None}
    try:
        data = b""
        conn.settimeout(30)
        while b"\n" not in data:
            chunk = conn.recv(65536)
            if not chunk:
                break
            data += chunk
        if not data:
            return
        payload = json.loads(data.split(b"\n", 1)[0])
        if not isinstance(payload, dict):
            box["reply"] = {"ok": False, "reason": "payload must be an object"}
        else:
            commands.put((payload, box))
            box["event"].wait(40)
        reply = box["reply"] or {"ok": False, "reason": "gateway timed out"}
        conn.sendall((json.dumps(reply) + "\n").encode("utf-8"))
    except Exception as exc:
        try:
            conn.sendall((json.dumps({"ok": False, "reason": str(exc)}) + "\n").encode("utf-8"))
        except OSError:
            pass
    finally:
        conn.close()


def main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") != "42":
        print(json.dumps({"ready": False, "reason": "gateway refuses every domain except 42"}))
        return 2
    if Path("/dev/ttyAMA0").exists() or Path("/dev/ttyAMA4").exists():
        print(json.dumps({"ready": False, "reason": "refusing to start while a Pinky serial device exists"}))
        return 3
    commands: queue.Queue = queue.Queue()
    gateway = Gateway()
    threading.Thread(target=serve, args=(commands,), daemon=True).start()
    while True:
        gateway.spin_once()
        try:
            payload, box = commands.get_nowait()
        except queue.Empty:
            continue
        try:
            box["reply"] = gateway.handle(payload)
        except Exception as exc:
            box["reply"] = {"ok": False, "reason": str(exc)}
        box["event"].set()


if __name__ == "__main__":
    raise SystemExit(main())
