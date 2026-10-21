"""Navigation and mapping stay separate. Master maps are not overwritten."""
from __future__ import annotations

from pathlib import Path

from robot_profile.maps import MapRecord

MODES = {"navigation", "mapping", "map_update"}


class ModeError(Exception):
    pass


def staging_dir(root: Path) -> Path:
    return root / "maps" / "staging"


def assert_mode(mode: str) -> str:
    if mode not in MODES:
        raise ModeError(f"unknown operation mode: {mode}")
    return mode


def mapping_output_path(root: Path, name: str, masters: list[Path]) -> Path:
    """Return a staging path. Refuse names that would replace a catalog map."""
    cleaned = Path(name).name
    if cleaned != name or not cleaned or cleaned in {".", ".."}:
        raise ModeError("mapping output must be a file name, not a path")
    target = staging_dir(root) / cleaned
    resolved = target.resolve()
    for master in masters:
        if master.resolve() == resolved or master.resolve().parent == resolved:
            raise ModeError("mapping output collides with a catalog map")
        if master.name == cleaned:
            raise ModeError("mapping output reuses a master map file name")
    return target


def master_replace_allowed(mode: str) -> bool:
    assert_mode(mode)
    return False


def update_block_reason(current: MapRecord, candidate: MapRecord) -> str | None:
    if current.verification == "missing" or candidate.verification == "missing":
        return "a missing map cannot be compared or promoted"
    if current.resolution != candidate.resolution or current.origin != candidate.origin:
        return "resolution or origin differs; the candidate stays in staging"
    if candidate.verification not in {"surveyed", "gazebo_nav2_goal"}:
        return "candidate is not verified"
    return "promotion stays manual; this function does not copy the master map"
