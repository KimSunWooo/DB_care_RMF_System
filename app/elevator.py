"""Elevator gateway contract.

This module only describes the future Lift Adapter → Gateway payload.
It has no socket, serial, HTTP client, or ROS publisher.
"""
from __future__ import annotations

from enum import Enum
from typing import Literal


class ElevatorPhase(str, Enum):
    IDLE = "IDLE"
    REQUESTED = "REQUESTED"
    MOVING = "MOVING"
    ARRIVED = "ARRIVED"
    DOOR_OPEN = "DOOR_OPEN"
    BOARDING = "BOARDING"
    ERROR = "ERROR"


class BoardingCheck(str, Enum):
    """Separate from phase. A QR scan is not one of these checks."""
    BOARDING_PERMITTED = "boarding_permitted"
    DOOR_CLOSED = "door_closed_confirmed"
    DESTINATION_ARRIVED = "destination_floor_confirmed"
    ALIGHTED = "alighting_confirmed"


def build_trip_preview(*, robot_id: str, source_floor: Literal["1F", "7F"],
                       target_floor: Literal["1F", "7F"], session_id: str) -> dict:
    """Return the Gateway JSON an adapter would send later. Does not transmit it."""
    if source_floor == target_floor:
        raise ValueError("source_floor and target_floor must differ")
    return {
        "command_id": None,
        "robot_id": robot_id,
        "source_floor": source_floor,
        "target_floor": target_floor,
        "requested_by": "rmf-lift-adapter",
        "session_id": session_id,
        "transmit": False,
        "checks_required": [item.value for item in BoardingCheck],
        "qr_is_completion": False,
    }


def refuse_hardware_command() -> None:
    raise RuntimeError("Elevator hardware commands are disabled in this starter")
