"""Gazebo test-room graph. The operational placeholder graph is not loaded here."""
from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import yaml

from sim.gazebo.room_geometry import point_occupied

FLEET_FILE = Path("config/rmf/gazebo_test_room/fleet.yaml")
GRAPH_FILE = Path("config/rmf/gazebo_test_room/nav.yaml")
SIM_RMF_DOMAIN = 77
GAZEBO_NAV_DOMAIN = 42
OPERATIONAL_RMF_DOMAIN = 0


class GraphError(Exception):
    pass


@dataclass(frozen=True)
class Waypoint:
    name: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class SimSession:
    fleet_name: str
    robot_id: str
    backend: str
    rmf_domain_id: int
    nav_domain_id: int
    graph_id: str
    clearance_m: float
    transform: tuple[float, float, float, float]
    waypoints: dict[str, Waypoint]
    allow_motion: bool

    def nav_pose(self, name: str) -> Waypoint:
        waypoint = self.waypoints.get(name)
        if waypoint is None:
            raise GraphError(f"waypoint is not on the gazebo test graph: {name}")
        tx, ty, yaw, scale = self.transform
        if (tx, ty, yaw, scale) != (0.0, 0.0, 0.0, 1.0):
            raise GraphError("the gazebo transform is not the verified identity")
        if not clearance_ok(waypoint.x, waypoint.y, self.clearance_m):
            raise GraphError(f"waypoint {name} is inside the test-room obstacle")
        return waypoint


def clearance_ok(x: float, y: float, radius: float) -> bool:
    if point_occupied(x, y):
        return False
    for step in range(8):
        angle = step * math.tau / 8
        if point_occupied(x + radius * math.cos(angle), y + radius * math.sin(angle)):
            return False
    return True


def _waypoints(path: Path) -> dict[str, Waypoint]:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    levels = raw.get("levels") if isinstance(raw, dict) else None
    level = (levels or {}).get("gazebo_test_room")
    if not isinstance(level, dict):
        raise GraphError("gazebo test graph is missing level gazebo_test_room")
    found: dict[str, Waypoint] = {}
    for vertex in level.get("vertices") or []:
        if not isinstance(vertex, list) or len(vertex) < 3 or not isinstance(vertex[-1], dict):
            raise GraphError("graph vertex is not [x, y, properties]")
        name = str(vertex[-1].get("name") or "")
        if not name:
            raise GraphError("graph vertex has no name")
        found[name] = Waypoint(name, float(vertex[0]), float(vertex[1]), 0.0)
    if set(found) != {"GZ_SPAWN", "GZ_EAST"}:
        raise GraphError("gazebo test graph must contain only GZ_SPAWN and GZ_EAST")
    return found


def load_sim_session(root: Path) -> SimSession:
    fleet_path = root / FLEET_FILE
    graph_path = root / GRAPH_FILE
    raw = yaml.safe_load(fleet_path.read_text(encoding="utf-8"))
    session = raw.get("sim_session") or {}
    care = raw.get("care_move") or {}
    fleet_name = str((raw.get("rmf_fleet") or {}).get("name") or "")
    if fleet_name == "care_move" or fleet_name != "gazebo_pinky1":
        raise GraphError("sim session must use fleet gazebo_pinky1")
    if bool(care.get("allow_motion")):
        raise GraphError("the gazebo fleet file must not turn on operational allow_motion")
    transform = session.get("transform") or {}
    loaded = SimSession(
        fleet_name=fleet_name,
        robot_id=str(session.get("robot_id") or ""),
        backend=str(session.get("backend") or ""),
        rmf_domain_id=int(session.get("rmf_domain_id")),
        nav_domain_id=int(session.get("nav_domain_id")),
        graph_id=str(session.get("graph_id") or ""),
        clearance_m=float(session.get("clearance_m") or 0.0),
        transform=(
            float(transform.get("tx")),
            float(transform.get("ty")),
            float(transform.get("yaw")),
            float(transform.get("scale")),
        ),
        waypoints=_waypoints(graph_path),
        allow_motion=bool(care.get("allow_motion")),
    )
    if loaded.rmf_domain_id != SIM_RMF_DOMAIN or loaded.nav_domain_id != GAZEBO_NAV_DOMAIN:
        raise GraphError("sim session domains drifted from 77 and 42")
    if loaded.graph_id != "gazebo_test_room" or loaded.backend != "gazebo":
        raise GraphError("sim session is not bound to the gazebo test room")
    for waypoint in loaded.waypoints.values():
        loaded.nav_pose(waypoint.name)
    return loaded


def dispatch_block(session: SimSession, *, rmf_domain: int, nav_domain: int, backend: str,
                   robot_id: str, fleet_name: str, action_ready: bool, pose_fresh: bool,
                   clock_ok: bool, hardware_devices: bool) -> str | None:
    if backend != "gazebo" or session.backend != "gazebo":
        return "backend is not the gazebo session"
    if robot_id != "pinky1" or session.robot_id != "pinky1":
        return "gazebo session only includes pinky1"
    if fleet_name != "gazebo_pinky1":
        return "refusing a fleet other than gazebo_pinky1"
    if rmf_domain != SIM_RMF_DOMAIN or rmf_domain == OPERATIONAL_RMF_DOMAIN:
        return "simulation RMF is not on its isolated domain"
    if nav_domain != GAZEBO_NAV_DOMAIN:
        return "Nav2 is not on the gazebo domain"
    if hardware_devices:
        return "hardware serial devices are visible on this host"
    if not action_ready or not pose_fresh or not clock_ok:
        return "Nav2 action, pose, or clock is not ready"
    return None
