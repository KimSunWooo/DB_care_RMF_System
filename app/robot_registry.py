"""Availability comes from the live gateway, not a cached RMF fleet."""
from __future__ import annotations

from fleet_adapter.physical_motion import DRIVE_DOMAIN_ID, PHYSICAL_ROBOT_IDS


PROFILES = {
    "real_pinky1": {
        "fleet": "physical_pinky",
        "ssh_host": None,
        "ros_domain_id": 24,
        "namespace": "",
        "odom_topic": "/odom",
        "cmd_vel_topic": "/cmd_vel",
    },
    "real_pinky2": {
        "fleet": "physical_pinky",
        "ssh_host": None,
        "ros_domain_id": 24,
        "namespace": "pinky2",
        "odom_topic": "/pinky2/odom",
        "cmd_vel_topic": "/pinky2/cmd_vel",
    },
    "real_pinky3": {
        "fleet": "physical_pinky",
        "ssh_host": None,
        "ros_domain_id": 24,
        "namespace": "pinky3",
        "odom_topic": "/pinky3/odom",
        "cmd_vel_topic": "/pinky3/cmd_vel",
    },
}


def can_dispatch(row: dict) -> bool:
    """OFFLINE and stale robots are not ARM targets. A missing robot is not online."""
    return row.get("status") == "IDLE" and bool(row.get("available")) and bool(row.get("motor_ready"))


def classify(robot: dict, mode: str, goal_phase: str | None) -> str:
    if mode == "FAULT":
        return "FAULT"
    if not robot.get("available"):
        return "OFFLINE"
    if not robot.get("odom_fresh"):
        return "STALE"
    if goal_phase in {"moving", "stopping"} or mode in {"TRANSITIONING", "STOPPING"}:
        return "BUSY"
    if robot.get("motor_ready") is not True:
        return "UNAVAILABLE"
    return "IDLE"


def registry(robots: list[dict], modes: dict[str, str], goal_phase: str, selected_id: str) -> list[dict]:
    listed = {item.get("id"): item for item in robots if isinstance(item, dict)}
    rows = []
    for robot_id in PHYSICAL_ROBOT_IDS:
        robot = listed.get(robot_id) or {"id": robot_id, "available": False}
        mode = modes.get(robot_id) or "PHYSICAL_DRY_RUN"
        phase = goal_phase if robot_id == selected_id else "idle"
        status = classify(robot, mode, phase)
        rows.append({
            "robot_id": robot_id,
            "fleet": "physical_pinky",
            "namespace": robot.get("namespace") or "",
            "odom_topic": robot.get("odom_topic"),
            "cmd_vel_topic": robot.get("cmd_vel_topic"),
            "domain_id": int(DRIVE_DOMAIN_ID),
            "rmf_domain": 77,
            "map_id": None,
            "map_aligned": False,
            "status": status,
            "available": bool(robot.get("available")),
            "motor_ready": robot.get("motor_ready") is True,
            "odom_fresh": bool(robot.get("odom_fresh")),
            "gateway_mode": mode,
            "active_goal_phase": phase,
        })
    return rows
