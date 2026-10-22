"""Command gate for a future real Pinky fleet. It does not start RMF or Nav2."""
from __future__ import annotations

REAL_ROBOT_ID = "real_pinky1"
REAL_FLEET_NAME = "real_pinky1"
REAL_NAV_DOMAIN = 53
GAZEBO_ROBOT_ID = "pinky1"
GAZEBO_FLEET_NAME = "gazebo_pinky1"
FORBIDDEN_RMF_DOMAINS = {0, 42, 53, 77, 90}


def real_task_block(
    *,
    robot_id: str,
    fleet_name: str,
    rmf_domain: int,
    nav_domain: int,
    commands_enabled: bool,
    operator_stop_approved: bool,
    safety_ready: bool,
    nav2_ready: bool,
    graph_aligned: bool,
    goal_is_short_trial: bool,
) -> str | None:
    """Return a refusal reason. None means this one short trial may be sent."""
    if robot_id == GAZEBO_ROBOT_ID or fleet_name == GAZEBO_FLEET_NAME:
        return "refusing the gazebo pinky1 identity"
    if robot_id != REAL_ROBOT_ID or fleet_name != REAL_FLEET_NAME:
        return "real fleet only accepts real_pinky1"
    if rmf_domain in FORBIDDEN_RMF_DOMAINS:
        return f"refusing RMF domain {rmf_domain}"
    if nav_domain != REAL_NAV_DOMAIN:
        return "real Nav2 must stay on domain 53"
    if not commands_enabled:
        return "real command forwarding is disabled"
    if not operator_stop_approved:
        return "operator has not confirmed an independent stop"
    if not safety_ready or not nav2_ready or not graph_aligned:
        return "safety, Nav2, or the navigation graph is not ready"
    if not goal_is_short_trial:
        return "only one approved short trial goal is allowed"
    return None
