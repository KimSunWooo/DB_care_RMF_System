"""Which unfinished RMF bookings can take a Pinky. No network and no motion."""
from __future__ import annotations

from app.task_reconcile import task_updated_ms

NONTERMINAL_RMF_STATUSES = frozenset({"queued", "standby", "underway", "blocked"})
PHYSICAL_FLEET_NAME = "physical_pinky"


def conflicting_bookings(states: object, robot_id: str, fleet: str = PHYSICAL_FLEET_NAME) -> list[dict]:
    """Bookings that are not terminal and can still be assigned to this robot.

    A booking assigned to another robot in the same fleet is not a conflict.
    A booking with no robot name on this fleet, or with no fleet at all, can
    still be dispatched here, so it is a conflict.
    """
    if not isinstance(states, list):
        raise ValueError("task states are not a list")
    found = []
    for state in states:
        if not isinstance(state, dict):
            continue
        status = str(state.get("status") or "").strip().lower()
        if status not in NONTERMINAL_RMF_STATUSES:
            continue
        assigned = state.get("assigned_to") if isinstance(state.get("assigned_to"), dict) else {}
        group = str(assigned.get("group") or "")
        name = str(assigned.get("name") or "")
        if not _affects_robot(group, name, robot_id, fleet):
            continue
        booking = state.get("booking") if isinstance(state.get("booking"), dict) else {}
        found.append({
            "id": str(booking.get("id") or ""),
            "status": status,
            "robot_id": name or None,
            "fleet": group or None,
            "category": state.get("category"),
            "updated_ms": task_updated_ms(state),
        })
    return found


def _affects_robot(group: str, name: str, robot_id: str, fleet: str) -> bool:
    if group and group != fleet:
        return False
    if name and name != robot_id:
        return False
    return True
