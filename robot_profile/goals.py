"""Turn a fleet waypoint into a Nav2 action path. Coordinates stay outside this step."""
from __future__ import annotations

from fleet_adapter.nav2_link import Nav2Link
from robot_profile.profiles import RobotProfile


class GoalRejected(Exception):
    pass


def fleet_goal_command(profile: RobotProfile, link: Nav2Link, waypoint: str,
                       graph_alignment: str) -> dict:
    if graph_alignment != "surveyed":
        raise GoalRejected("refusing to invent map coordinates for an unsurveyed graph")
    if not waypoint:
        raise GoalRejected("waypoint is empty")
    if link.action_path != profile.navigate_to_pose:
        raise GoalRejected("fleet link action does not match the robot profile")
    return {
        "robot_id": profile.robot_id,
        "action": link.action_path,
        "frame_id": "map",
        "waypoint": waypoint,
        "backend": profile.backend.name,
    }
