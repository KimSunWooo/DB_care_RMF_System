"""Map RMF fleet snapshots onto Care-Move robot records.

A missing or stale sample clears the live pose. Configured UI waypoints are
never treated as a measured robot position.
"""
from __future__ import annotations

from datetime import datetime, timezone
from typing import Any


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def issue_text(issues: Any) -> str | None:
    if not isinstance(issues, list) or not issues:
        return None
    parts: list[str] = []
    for issue in issues:
        if isinstance(issue, dict):
            category = issue.get("category") or "issue"
            detail = issue.get("detail")
            parts.append(f"{category}: {detail}" if detail else str(category))
        else:
            parts.append(str(issue))
    return "; ".join(parts)[:500]


def battery_percent(value: Any, fallback: Any) -> Any:
    if isinstance(value, (int, float)):
        return round(float(value) * 100, 1) if 0 <= float(value) <= 1 else value
    return fallback


def apply_fleet_state(robots: dict[str, dict], fleet_state: dict, received_at: str) -> None:
    remote = fleet_state.get("robots") if isinstance(fleet_state, dict) else None
    if not isinstance(remote, dict):
        remote = {}
    for robot_id, robot in robots.items():
        sample = remote.get(robot_id)
        if not isinstance(sample, dict):
            robot["position_fresh"] = False
            robot["pose"] = None
            robot["waypoint"] = None
            if robot.get("last_state_at"):
                robot["connection"] = "offline"
                robot["status"] = "offline"
            else:
                robot["connection"] = "unverified"
                robot["status"] = "unverified"
                robot["floor"] = None
            continue
        location = sample.get("location") if isinstance(sample.get("location"), dict) else None
        robot.update({
            "status": sample.get("status") or "unverified",
            "connection": "online",
            "task_id": sample.get("task_id") or None,
            "last_state_at": received_at,
            "position_fresh": location is not None,
            "position_source": "rmf" if location else "rmf_no_pose",
            "pose": location,
            "floor": location.get("map") if location else None,
            "waypoint": None,
            "battery": battery_percent(sample.get("battery"), robot.get("battery")),
            "error": issue_text(sample.get("issues")),
        })


def mark_stale(robots: dict[str, dict]) -> None:
    """Drop live poses after a failed poll. Robots never seen stay unverified."""
    for robot in robots.values():
        robot["position_fresh"] = False
        robot["pose"] = None
        robot["waypoint"] = None
        if not robot.get("last_state_at"):
            robot["connection"] = "unverified"
            robot["status"] = "unverified"
            robot["floor"] = None
            continue
        robot["connection"] = "stale"
        robot["status"] = "stale"
        robot["floor"] = None
