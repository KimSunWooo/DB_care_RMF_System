"""External transport requests. They enter the queue and do not move a robot."""
from __future__ import annotations

import json
import uuid

from app.map_registry import same_concept_floor
from app.task_store import TaskStore


def plan_phases(action: str, trip_type: str, origin: str, destination: str | None) -> list[dict]:
    phases = [{"step": "current_pose_not_assumed", "place": None, "assumes_robot_at_origin": False}]
    if action == "CALL":
        phases.append({"step": "move_to_origin", "place": origin, "automatic": False})
        phases.append({"step": "wait_for_confirmation", "place": origin, "automatic": False})
        if destination:
            phases.append({"step": "move_to_destination", "place": destination, "automatic": False})
            if trip_type == "ROUND_TRIP":
                phases.append({"step": "return_to_origin", "place": origin, "automatic": False})
        return phases
    phases.append({"step": "move_to_origin_if_needed", "place": origin, "assumes_robot_at_origin": False, "automatic": False})
    phases.append({"step": "move_to_destination", "place": destination, "automatic": False})
    if trip_type == "ROUND_TRIP":
        phases.append({"step": "return_to_origin", "place": origin, "automatic": False})
    return phases


def accept_transport(store: TaskStore, body: dict, waypoints: dict, robots: list[dict]) -> dict:
    action = body.get("action")
    origin = body.get("origin")
    destination = body.get("destination")
    trip_type = body.get("trip_type") or "ONE_WAY"
    if action not in {"MOVE", "CALL"}:
        raise ValueError("action must be MOVE or CALL")
    if trip_type not in {"ONE_WAY", "ROUND_TRIP"}:
        raise ValueError("trip_type must be ONE_WAY or ROUND_TRIP")
    if not origin:
        raise ValueError("origin is required")
    if action == "MOVE" and not destination:
        raise ValueError("destination is required for MOVE")
    floor_error = same_concept_floor(waypoints, origin, destination)
    if floor_error:
        raise ValueError(floor_error)
    robot_id = body.get("robot_id")
    if robot_id:
        known = {item.get("robot_id"): item for item in robots}
        robot = known.get(robot_id)
        if robot is None or robot.get("status") == "OFFLINE" or not robot.get("available"):
            raise ValueError("robot is offline")
    else:
        idle = [item for item in robots if item.get("status") == "IDLE" and item.get("available")]
        robot_id = idle[0]["robot_id"] if idle else None
    request_id = str(body.get("request_id") or uuid.uuid4().hex)
    phases = plan_phases(action, trip_type, origin, destination)
    record = store.put({
        "request_id": request_id,
        "robot_id": robot_id,
        "fleet_id": "physical_pinky",
        "origin": origin,
        "destination": destination,
        "action": action,
        "trip_type": trip_type,
        "status": "QUEUED",
        "dispatch_allowed": False,
        "kind": "transport",
        "phases": json.dumps(phases),
        "error": "queued only; physical map alignment is not verified, so this request is not submitted to RMF",
        "rmf_booking_id": None,
    })
    return {
        "request_id": request_id,
        "accepted": True,
        "status": "QUEUED",
        "robot_id": robot_id,
        "rmf_booking_id": None,
        "phases": phases,
        "execution": "not_submitted",
        "detail": record.get("error"),
    }
