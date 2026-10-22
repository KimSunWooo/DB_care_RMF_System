"""Local gateway between an isolated RMF domain and the Gazebo Nav2 action.

The gateway never publishes cmd_vel and never accepts a hardware backend.
"""
from __future__ import annotations

import json
import socket
from dataclasses import dataclass, field

from fleet_adapter.sim_graph import GraphError, SimSession, dispatch_block


class GatewayRefused(Exception):
    pass


@dataclass
class GoalTicket:
    task_id: str
    waypoint: str
    x: float
    y: float
    yaw: float
    phase: str


@dataclass
class SimGateway:
    up: bool = False
    stale: bool = False
    pose: tuple[float, float, float] | None = None
    pose_at: float | None = None
    goal: GoalTicket | None = None
    hardware_commands: int = 0
    robots: list[str] = field(default_factory=list)

    def connect(self, reason: str | None) -> None:
        if reason:
            raise GatewayRefused(reason)
        self.up = True
        self.stale = False

    def register(self, robot_id: str) -> None:
        if not self.up:
            raise GatewayRefused("gateway is down")
        if robot_id not in self.robots:
            self.robots.append(robot_id)

    def publish_pose(self, x: float, y: float, yaw: float, now: float) -> None:
        if not self.up:
            self.stale = True
            return
        self.pose = (x, y, yaw)
        self.pose_at = now
        self.stale = False

    def pose_fresh(self, now: float, timeout: float = 2.0) -> bool:
        if not self.up or self.stale or self.pose_at is None:
            return False
        return now - self.pose_at <= timeout

    def send_navigation(self, session: SimSession, waypoint: str, task_id: str) -> GoalTicket:
        if self.hardware_commands:
            raise GatewayRefused("a hardware command was already refused")
        if not self.up or self.stale:
            raise GatewayRefused("gateway is down")
        try:
            pose = session.nav_pose(waypoint)
        except GraphError as exc:
            raise GatewayRefused(str(exc)) from exc
        self.goal = GoalTicket(task_id, pose.name, pose.x, pose.y, pose.yaw, "accepted")
        return self.goal

    def refuse_hardware(self) -> None:
        self.hardware_commands += 1
        raise GatewayRefused("refusing cmd_vel and hardware NavigateToPose")

    def cancel(self) -> None:
        if self.goal and self.goal.phase == "accepted":
            self.goal.phase = "canceled"

    def succeed(self) -> None:
        if self.goal and self.goal.phase == "accepted":
            self.goal.phase = "succeeded"

    def drop(self) -> None:
        self.up = False
        self.stale = True
        if self.goal and self.goal.phase == "accepted":
            self.goal.phase = "uncertain"


@dataclass
class TaskRecord:
    phase: str = "idle"
    rmf_status: str = "none"
    reason: str = ""
    goal_sent: bool = False

    def dispatch(self, session: SimSession, gateway: SimGateway, waypoint: str, task_id: str,
                 now: float, **runtime) -> None:
        reason = dispatch_block(session, **runtime)
        if reason:
            self.phase = "rejected"
            self.rmf_status = "rejected"
            self.reason = reason
            return
        if not gateway.pose_fresh(now):
            self.phase = "rejected"
            self.rmf_status = "rejected"
            self.reason = "robot pose is stale"
            return
        try:
            gateway.send_navigation(session, waypoint, task_id)
        except GatewayRefused as exc:
            self.phase = "rejected"
            self.rmf_status = "rejected"
            self.reason = str(exc)
            return
        self.goal_sent = True
        self.phase = "navigating"
        self.rmf_status = "underway"

    def observe(self, gateway: SimGateway) -> None:
        if gateway.goal is None:
            return
        phase = gateway.goal.phase
        if phase == "succeeded":
            self.phase = "completed"
            self.rmf_status = "completed"
        elif phase == "canceled":
            self.phase = "canceled"
            self.rmf_status = "canceled"
        elif phase == "uncertain":
            self.phase = "uncertain"
            self.rmf_status = "failed"
            self.reason = "gateway lost the goal result"


def readonly_argv_allowed(argv: list[str]) -> bool:
    if not argv or argv[0] != "ros2":
        return False
    banned = {"pub", "send_goal", "call", "launch", "param"}
    if any(token in banned for token in argv):
        return False
    allowed = {
        ("ros2", "node", "list"),
        ("ros2", "topic", "list"),
        ("ros2", "topic", "info"),
        ("ros2", "topic", "echo"),
        ("ros2", "topic", "hz"),
        ("ros2", "action", "list"),
        ("ros2", "action", "info"),
        ("ros2", "run",),
    }
    head = tuple(argv[:3])
    if head in allowed or head[:2] == ("ros2", "topic") and argv[2] in {"list", "info", "echo", "hz"}:
        if "echo" in argv and "--once" not in argv:
            return False
        return "cmd_vel" not in " ".join(argv)
    if head == ("ros2", "action", "list") or head == ("ros2", "action", "info"):
        return True
    if head == ("ros2", "node", "list"):
        return True
    return False


def serve_line(handler, host: str = "127.0.0.1") -> tuple[socket.socket, int]:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind((host, 0))
    server.listen(1)
    server.settimeout(5)
    return server, server.getsockname()[1]


def read_json_line(conn: socket.socket) -> dict:
    data = b""
    while b"\n" not in data:
        chunk = conn.recv(4096)
        if not chunk:
            break
        data += chunk
    if not data:
        raise GatewayRefused("gateway peer closed")
    payload = json.loads(data.split(b"\n", 1)[0])
    if not isinstance(payload, dict) or "op" not in payload:
        raise GatewayRefused("gateway message has no op")
    return payload


def write_json_line(conn: socket.socket, payload: dict) -> None:
    conn.sendall((json.dumps(payload) + "\n").encode("utf-8"))
