"""Bookkeeping for one Gazebo Nav2 goal. This module does not publish ROS messages."""
from __future__ import annotations

SUCCEEDED = 4
CANCELED = 5
ABORTED = 6


class GoalTracker:
    def __init__(self) -> None:
        self.generation = 0
        self.phase = "idle"
        self.goal_id = ""
        self.cancel_requested = False

    def begin(self) -> int | None:
        if self.phase in {"pending", "accepted"}:
            return None
        self.generation += 1
        self.phase = "pending"
        self.goal_id = ""
        self.cancel_requested = False
        return self.generation

    def accept(self, generation: int, goal_id: str) -> bool:
        if generation != self.generation or self.phase != "pending":
            return False
        self.phase = "accepted"
        self.goal_id = goal_id
        return True

    def reject(self, generation: int) -> bool:
        if generation != self.generation or self.phase != "pending":
            return False
        self.phase = "rejected"
        return True

    def request_cancel(self, generation: int) -> bool:
        if generation != self.generation or self.phase != "accepted":
            return False
        self.cancel_requested = True
        return True

    def observe_status(self, generation: int, status: int | None) -> str:
        if generation != self.generation or self.phase not in {"pending", "accepted"}:
            return "stale"
        if status == SUCCEEDED:
            self.phase = "succeeded"
        elif status == CANCELED:
            self.phase = "canceled"
        elif status == ABORTED:
            self.phase = "aborted"
        else:
            self.phase = "uncertain"
        return self.phase

    def snapshot(self) -> dict:
        return {
            "generation": self.generation,
            "phase": self.phase,
            "goal_id": self.goal_id,
            "cancel_requested": self.cancel_requested,
            "result_confirmed": self.phase in {"succeeded", "canceled", "aborted", "rejected", "uncertain"},
        }
