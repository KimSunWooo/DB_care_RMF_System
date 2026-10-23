"""Classify maps that already exist. None of them is an aligned hospital map."""
from __future__ import annotations

from pathlib import Path

import yaml

CONCEPT = "ui_concept"
RMF_GRAPH = "rmf_navigation_graph"
NAV2_GRID = "nav2_occupancy_grid"
BUILDING = "rmf_building_map"
TRIAL = "odom_trial_graph"


def load_registry(root: Path) -> dict:
    catalog_path = root / "config" / "maps" / "catalog.yaml"
    raw = yaml.safe_load(catalog_path.read_text(encoding="utf-8")) if catalog_path.is_file() else {}
    maps = []
    for map_id, item in (raw.get("maps") or {}).items():
        yaml_path = item.get("yaml")
        path = Path(yaml_path) if yaml_path else None
        if path and not path.is_absolute():
            path = root / path
        present = bool(path and path.is_file())
        image = None
        resolution = None
        origin = None
        if present and path is not None:
            meta = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
            image = meta.get("image")
            resolution = meta.get("resolution")
            origin = meta.get("origin")
        kind = NAV2_GRID if present else "missing"
        maps.append({
            "map_id": map_id,
            "kind": kind,
            "floor_id": item.get("floor_id"),
            "version": item.get("version"),
            "verification": item.get("verification"),
            "yaml": str(path) if path else None,
            "image": image,
            "resolution": resolution,
            "origin": origin,
            "file_present": present,
            "physical_aligned": False,
            "active_master": False,
        })
    maps.extend([
        {"map_id": "care_move_ui", "kind": CONCEPT, "file_present": True,
         "yaml": "config/waypoints.json", "physical_aligned": False,
         "note": "normalized dashboard drawing, not a surveyed map"},
        {"map_id": "gazebo_test_room_graph", "kind": RMF_GRAPH, "file_present": True,
         "yaml": "config/rmf/gazebo_test_room/nav.yaml", "physical_aligned": False},
        {"map_id": "gazebo_test_room_building", "kind": BUILDING, "file_present": True,
         "yaml": "config/rmf/gazebo_test_room/gazebo_test_room.building.yaml", "physical_aligned": False},
        {"map_id": "physical_odom_trial", "kind": TRIAL, "file_present": False,
         "physical_aligned": False,
         "note": "generated per 5cm test; not a facility map"},
    ])
    return {
        "physical_navigation_ready": False,
        "reason": "no surveyed map is aligned with a physical Pinky localization frame",
        "maps": maps,
    }


def same_concept_floor(waypoints: dict, origin: str, destination: str | None) -> str | None:
    if origin not in waypoints:
        return f"unknown waypoint: {origin}"
    if destination and destination not in waypoints:
        return f"unknown waypoint: {destination}"
    if destination and waypoints[origin].get("floor") != waypoints[destination].get("floor"):
        return "origin and destination are on different floors"
    return None
