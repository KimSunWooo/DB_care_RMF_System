"""Load the care_move fleet file without starting ROS or publishing state."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


class FleetConfigError(Exception):
    pass


@dataclass(frozen=True)
class RobotSpec:
    robot_id: str
    enabled: bool
    ros_namespace: str
    navigate_action: str
    pose_topic: str
    initial_waypoint: str
    charger: str | None


@dataclass(frozen=True)
class FleetSpec:
    fleet_name: str
    graph_alignment: str
    allow_motion: bool
    simulation_confirmed: bool
    server_uri: str
    map_name: str
    pose_timeout_sec: float
    goal_timeout_sec: float
    nav_graph: Path
    robots: tuple[RobotSpec, ...]

    @property
    def active_robots(self) -> tuple[RobotSpec, ...]:
        return tuple(robot for robot in self.robots if robot.enabled)


def load_fleet_spec(path: Path, root: Path | None = None) -> FleetSpec:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise FleetConfigError("fleet config must be a mapping")
    rmf = raw.get("rmf_fleet") or {}
    care = raw.get("care_move") or {}
    if not isinstance(rmf, dict) or not isinstance(care, dict):
        raise FleetConfigError("rmf_fleet and care_move must be mappings")
    fleet_name = str(rmf.get("name") or "").strip()
    if not fleet_name:
        raise FleetConfigError("rmf_fleet.name is required")
    alignment = str(care.get("graph_alignment") or "").strip()
    if alignment not in {"placeholder", "surveyed"}:
        raise FleetConfigError("care_move.graph_alignment must be placeholder or surveyed")
    allow_motion = bool(care.get("allow_motion"))
    if allow_motion and alignment != "surveyed":
        raise FleetConfigError("allow_motion requires graph_alignment surveyed")
    server_uri = str(care.get("server_uri") or "").strip()
    if not server_uri.startswith("ws://") and not server_uri.startswith("wss://"):
        raise FleetConfigError("care_move.server_uri must be a websocket URI")
    nav_graph = Path(str(care.get("nav_graph") or ""))
    if not nav_graph.is_absolute():
        nav_graph = (root or path.parent.parent.parent) / nav_graph
    if not nav_graph.is_file():
        raise FleetConfigError(f"navigation graph not found: {nav_graph}")
    rmf_robots = rmf.get("robots") or {}
    care_robots = care.get("robots") or {}
    if not isinstance(rmf_robots, dict) or not isinstance(care_robots, dict):
        raise FleetConfigError("robot tables must be mappings")
    robots: list[RobotSpec] = []
    for robot_id, item in care_robots.items():
        if not isinstance(item, dict):
            raise FleetConfigError(f"{robot_id} must be a mapping")
        enabled = bool(item.get("enabled"))
        charger = None
        if enabled:
            rmf_item = rmf_robots.get(robot_id)
            if not isinstance(rmf_item, dict) or not rmf_item.get("charger"):
                raise FleetConfigError(f"enabled robot {robot_id} needs rmf_fleet.robots charger")
            charger = str(rmf_item["charger"])
        elif robot_id in rmf_robots:
            raise FleetConfigError(f"disabled robot {robot_id} must not be listed in rmf_fleet.robots")
        robots.append(RobotSpec(
            robot_id=str(robot_id),
            enabled=enabled,
            ros_namespace=str(item.get("ros_namespace") or robot_id).strip("/"),
            navigate_action=str(item.get("navigate_action") or "navigate_to_pose"),
            pose_topic=str(item.get("pose_topic") or "amcl_pose"),
            initial_waypoint=str(item.get("initial_waypoint") or ""),
            charger=charger,
        ))
    enabled_ids = {robot.robot_id for robot in robots if robot.enabled}
    unknown = sorted(set(rmf_robots) - enabled_ids)
    if unknown:
        raise FleetConfigError("rmf_fleet.robots has robots that are not enabled: " + ", ".join(unknown))
    if not any(robot.enabled for robot in robots):
        raise FleetConfigError("at least one robot must be enabled")
    return FleetSpec(
        fleet_name=fleet_name,
        graph_alignment=alignment,
        allow_motion=allow_motion,
        simulation_confirmed=bool(care.get("simulation_confirmed")),
        server_uri=server_uri,
        map_name=str(care.get("map_name") or "L1"),
        pose_timeout_sec=float(care.get("pose_timeout_sec") or 2.0),
        goal_timeout_sec=float(care.get("goal_timeout_sec") or 120.0),
        nav_graph=nav_graph,
        robots=tuple(robots),
    )


def read_graph_names(path: Path) -> list[str]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    names: list[str] = []
    levels = raw.get("levels") if isinstance(raw, dict) else None
    if not isinstance(levels, dict):
        raise FleetConfigError("navigation graph has no levels")
    for level in levels.values():
        if not isinstance(level, dict):
            continue
        for vertex in level.get("vertices") or []:
            if isinstance(vertex, list) and vertex and isinstance(vertex[-1], dict) and vertex[-1].get("name"):
                names.append(str(vertex[-1]["name"]))
    if len(names) < 2:
        raise FleetConfigError("navigation graph needs at least two named vertices")
    return names
