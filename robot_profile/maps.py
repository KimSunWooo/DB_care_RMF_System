"""Occupancy-map catalog. Missing maps raise; they are not replaced with blanks."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

USABLE = {"gazebo_nav2_goal", "surveyed"}


class MapError(Exception):
    pass


@dataclass(frozen=True)
class MapRecord:
    map_id: str
    floor_id: str
    version: int | None
    method: str | None
    verification: str
    active_master: bool
    yaml_path: Path | None
    image_path: Path | None
    resolution: float | None
    origin: tuple[float, float, float] | None
    occupied_thresh: float | None
    free_thresh: float | None
    negate: int | None

    @property
    def usable_for_navigation(self) -> bool:
        return self.verification in USABLE and self.yaml_path is not None and self.image_path is not None


def catalog_path(root: Path) -> Path:
    return root / "config" / "maps" / "catalog.yaml"


def _parse_map_yaml(path: Path) -> dict:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise MapError(f"map yaml is not a mapping: {path}")
    for key in ("image", "resolution", "origin", "occupied_thresh", "free_thresh", "negate"):
        if key not in raw:
            raise MapError(f"{path} is missing {key}")
    origin = raw["origin"]
    if not isinstance(origin, list) or len(origin) != 3:
        raise MapError(f"{path} origin must be [x, y, yaw]")
    return raw


def load_map(root: Path, map_id: str) -> MapRecord:
    catalog = yaml.safe_load(catalog_path(root).read_text(encoding="utf-8"))
    item = (catalog.get("maps") or {}).get(map_id)
    if not isinstance(item, dict):
        raise MapError(f"unknown map id: {map_id}")
    verification = str(item.get("verification") or "")
    yaml_value = item.get("yaml")
    if verification == "missing" or not yaml_value:
        return MapRecord(
            map_id=map_id,
            floor_id=str(item.get("floor_id") or ""),
            version=None,
            method=None,
            verification="missing",
            active_master=False,
            yaml_path=None,
            image_path=None,
            resolution=None,
            origin=None,
            occupied_thresh=None,
            free_thresh=None,
            negate=None,
        )
    yaml_path = Path(str(yaml_value))
    if not yaml_path.is_absolute():
        yaml_path = root / yaml_path
    if not yaml_path.is_file():
        raise MapError(f"map yaml is not a file: {yaml_path}")
    parsed = _parse_map_yaml(yaml_path)
    image_path = yaml_path.parent / str(parsed["image"])
    if not image_path.is_file():
        raise MapError(f"map image is not a file: {image_path}")
    origin = tuple(float(value) for value in parsed["origin"])
    return MapRecord(
        map_id=map_id,
        floor_id=str(item.get("floor_id") or ""),
        version=None if item.get("version") is None else int(item["version"]),
        method=None if item.get("method") is None else str(item["method"]),
        verification=verification,
        active_master=bool(item.get("active_master")),
        yaml_path=yaml_path,
        image_path=image_path,
        resolution=float(parsed["resolution"]),
        origin=(origin[0], origin[1], origin[2]),
        occupied_thresh=float(parsed["occupied_thresh"]),
        free_thresh=float(parsed["free_thresh"]),
        negate=int(parsed["negate"]),
    )


def require_navigation_map(root: Path, map_id: str | None) -> MapRecord:
    if not map_id:
        raise MapError("no navigation map is selected")
    record = load_map(root, map_id)
    if record.verification == "missing":
        raise MapError(f"{map_id} has no occupancy map; do not invent one")
    if not record.usable_for_navigation:
        raise MapError(f"{map_id} is not verified for navigation")
    return record
