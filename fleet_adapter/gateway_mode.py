"""Per-robot gateway mode. Motion publishers follow this table, not a new process."""
from __future__ import annotations

import threading

from fleet_adapter.physical_motion import FORWARD_TEST, PHYSICAL_ROBOT_IDS, ROTATE_TEST, STOP_ANGULAR_RPS, STOP_SPEED_MPS

DRY_RUN = "PHYSICAL_DRY_RUN"
TRANSITIONING = "TRANSITIONING"
PHYSICAL_TEST = "PHYSICAL_TEST"
STOPPING = "STOPPING"
FAULT = "FAULT"
_TESTS = {FORWARD_TEST, ROTATE_TEST}


class ModeTable:
    """One mode per Pinky. Arming one robot does not change the others."""

    def __init__(self, initial: str = DRY_RUN) -> None:
        self._lock = threading.Lock()
        self._mode = {robot_id: DRY_RUN for robot_id in PHYSICAL_ROBOT_IDS}
        if initial == PHYSICAL_TEST:
            self._mode["real_pinky1"] = PHYSICAL_TEST

    def get(self, robot_id: str) -> str:
        with self._lock:
            return self._mode.get(robot_id, DRY_RUN)

    def snapshot(self) -> dict[str, str]:
        with self._lock:
            return dict(self._mode)

    def arm(self, robot_id: str, robot: dict, phase: str, session_busy: bool, kind: str) -> dict:
        with self._lock:
            current = self._mode.get(robot_id, DRY_RUN)
            if robot_id not in self._mode:
                return {"ok": False, "mode": current, "reason": "unknown Pinky"}
            if current == PHYSICAL_TEST:
                return {"ok": True, "mode": current, "already": True}
            if current in {TRANSITIONING, STOPPING, FAULT}:
                return {"ok": False, "mode": current, "reason": f"cannot arm from {current}"}
            reasons = _preflight(robot, phase, session_busy, kind)
            if reasons:
                self._mode[robot_id] = DRY_RUN
                return {"ok": False, "mode": DRY_RUN, "reasons": reasons, "reason": reasons[0]}
            self._mode[robot_id] = TRANSITIONING
            self._mode[robot_id] = PHYSICAL_TEST
            return {"ok": True, "mode": PHYSICAL_TEST, "already": False}

    def disarm(self, robot_id: str, speed: float, angular: float, phase: str) -> dict:
        """Leave PHYSICAL_TEST only after the wheel feedback is stopped."""
        with self._lock:
            current = self._mode.get(robot_id, DRY_RUN)
            if robot_id not in self._mode:
                return {"ok": False, "mode": current, "reason": "unknown Pinky"}
            if current == DRY_RUN:
                return {"ok": True, "mode": DRY_RUN}
            self._mode[robot_id] = STOPPING
            moving = phase in {"moving", "stopping"} or abs(speed) > STOP_SPEED_MPS or abs(angular) > STOP_ANGULAR_RPS
            if moving:
                self._mode[robot_id] = FAULT
                return {"ok": False, "mode": FAULT, "reason": "stop was not confirmed"}
            self._mode[robot_id] = DRY_RUN
            return {"ok": True, "mode": DRY_RUN}


def _preflight(robot: dict, phase: str, session_busy: bool, kind: str) -> list[str]:
    reasons = []
    if kind not in _TESTS:
        reasons.append("unknown physical test")
    if not robot.get("available"):
        reasons.append("Pinky is not visible on ROS domain 24")
    if robot.get("motor_ready") is not True:
        reasons.append("motor is not ready")
    if not robot.get("odom_fresh") or not robot.get("pose"):
        reasons.append("odometry is not fresh")
    if abs(float(robot.get("speed") or 0.0)) > STOP_SPEED_MPS or abs(float(robot.get("angular") or 0.0)) > STOP_ANGULAR_RPS:
        reasons.append("robot is still moving")
    if phase in {"moving", "stopping"}:
        reasons.append("an active goal is already running")
    if session_busy:
        reasons.append("a test session is already open")
    return reasons
