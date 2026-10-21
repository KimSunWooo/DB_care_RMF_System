"""Decide whether a Nav2 goal is allowed. Default fleet config never allows one."""
from __future__ import annotations


def navigation_block_reason(*, graph_alignment: str, allow_motion: bool,
                            simulation_confirmed: bool, action_available: bool,
                            pose_fresh: bool) -> str | None:
    if graph_alignment != "surveyed":
        return "navigation graph is not surveyed against a Nav2 map"
    if not allow_motion:
        return "fleet config disables motion"
    if not simulation_confirmed:
        return "simulation is not confirmed"
    if not action_available:
        return "Nav2 action server is not available"
    if not pose_fresh:
        return "robot pose is missing or stale"
    return None
