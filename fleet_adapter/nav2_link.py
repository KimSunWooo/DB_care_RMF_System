"""Per-robot Nav2 goal and pose bookkeeping. This module does not publish ROS messages."""
from __future__ import annotations

from dataclasses import dataclass


class Nav2Unavailable(Exception):
    pass


class Nav2Busy(Exception):
    pass


@dataclass
class GoalRecord:
    phase: str
    x: float
    y: float
    yaw: float
    cancel_requested: bool = False


class Nav2Link:
    def __init__(self, robot_id: str, namespace: str, action_name: str = "navigate_to_pose",
                 pose_timeout_sec: float = 2.0, goal_timeout_sec: float = 120.0):
        self.robot_id = robot_id
        self.namespace = namespace.strip("/")
        self.action_name = action_name
        self.pose_timeout_sec = pose_timeout_sec
        self.goal_timeout_sec = goal_timeout_sec
        self.action_available = False
        self.pose: tuple[float, float, float] | None = None
        self.pose_at: float | None = None
        self.goal: GoalRecord | None = None
        self.disconnects = 0

    @property
    def action_path(self) -> str:
        return f"/{self.namespace}/{self.action_name}"

    def set_action_available(self, available: bool) -> None:
        self.action_available = available

    def observe_pose(self, x: float, y: float, yaw: float, monotonic_now: float) -> None:
        self.pose = (x, y, yaw)
        self.pose_at = monotonic_now

    def pose_fresh(self, monotonic_now: float) -> bool:
        if self.pose is None or self.pose_at is None:
            return False
        return monotonic_now - self.pose_at <= self.pose_timeout_sec

    def mark_disconnected(self) -> None:
        self.action_available = False
        self.disconnects += 1
        if self.goal and self.goal.phase in {"pending", "accepted"}:
            self.goal.phase = "failed"

    def request_goal(self, x: float, y: float, yaw: float) -> GoalRecord:
        if not self.action_available:
            raise Nav2Unavailable(f"{self.action_path} is not available")
        if self.goal and self.goal.phase in {"pending", "accepted"}:
            raise Nav2Busy(self.robot_id)
        self.goal = GoalRecord("pending", x, y, yaw)
        return self.goal

    def mark_accepted(self) -> None:
        if self.goal and self.goal.phase == "pending":
            self.goal.phase = "accepted"

    def mark_rejected(self) -> None:
        if self.goal and self.goal.phase == "pending":
            self.goal.phase = "rejected"

    def observe_feedback(self, x: float, y: float, yaw: float, monotonic_now: float) -> None:
        if self.goal and self.goal.phase == "accepted":
            self.observe_pose(x, y, yaw, monotonic_now)

    def mark_result(self, success: bool) -> None:
        if self.goal and self.goal.phase == "accepted":
            self.goal.phase = "succeeded" if success else "failed"

    def request_cancel(self) -> None:
        if self.goal and self.goal.phase in {"pending", "accepted"}:
            self.goal.cancel_requested = True

    def mark_canceled(self) -> None:
        if self.goal and self.goal.cancel_requested:
            self.goal.phase = "canceled"

    def mark_goal_timeout(self, monotonic_now: float, started_at: float) -> None:
        if self.goal and self.goal.phase == "accepted" and monotonic_now - started_at >= self.goal_timeout_sec:
            self.goal.phase = "timed_out"
