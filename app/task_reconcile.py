"""Compare an RMF booking with the fleet process that would have to own it.

No network, no gateway command, and no change to a stored booking.
rmf_fleet_adapter 2.7.2 keeps TaskManager queues in the adapter process.
After a restart those queues are empty, and cancel_task() publishes nothing
when the id is missing. This module only names that condition.
"""
from __future__ import annotations

OBSERVATION_WINDOW_MS = 15_000
NONTERMINAL = frozenset({"queued", "standby", "underway", "blocked"})
BLOCKING = frozenset({
    "PENDING_CONFIRMATION",
    "OWNER_MISSING",
    "STATE_MISMATCH",
    "RMF_UNREACHABLE",
    "GATEWAY_UNREACHABLE",
    "RECOVERY_REQUIRED",
})


def task_updated_ms(state: dict) -> int | None:
    values: list[int] = []
    for key in ("unix_millis_finish_time", "unix_millis_start_time"):
        value = state.get(key)
        if isinstance(value, (int, float)) and value > 0:
            values.append(int(value))
    phases = state.get("phases") if isinstance(state.get("phases"), dict) else {}
    for phase in phases.values():
        if not isinstance(phase, dict):
            continue
        for key in ("unix_millis_finish_time", "unix_millis_start_time"):
            value = phase.get(key)
            if isinstance(value, (int, float)) and value > 0:
                values.append(int(value))
    return max(values) if values else None


class ReconciliationLog:
    """Repeated observations of one booking. The first sample is not a loss."""

    def __init__(self) -> None:
        self._samples: dict[tuple[str, str], dict] = {}

    def observe(self, robot_id: str, booking_id: str, at_ms: int, updated_ms: int | None) -> dict:
        key = (robot_id, booking_id)
        record = self._samples.get(key)
        stamp = int(at_ms)
        updated = None if updated_ms is None else int(updated_ms)
        if record is None:
            record = {"started_ms": stamp, "samples": [(stamp, updated)]}
            self._samples[key] = record
            return record
        previous = record["samples"][-1][0]
        if stamp < previous:
            record = {"started_ms": stamp, "samples": [(stamp, updated)]}
            self._samples[key] = record
            return record
        if stamp == previous:
            return record
        record["samples"].append((stamp, updated))
        del record["samples"][:-8]
        return record


def judge(
    *,
    robot_id: str,
    rmf_ok: bool,
    gateway_ok: bool,
    fleet_known: bool,
    bookings: list[dict],
    fleet_task_id: str,
    gateway_goal_active: bool,
    at_ms: int,
    log: ReconciliationLog,
    window_ms: int = OBSERVATION_WINDOW_MS,
) -> dict:
    if not gateway_ok:
        return _summary("GATEWAY_UNREACHABLE", [], fleet_task_id, gateway_goal_active,
                         "Gateway 상태를 확인하지 못했습니다.")
    if not rmf_ok:
        return _summary("RMF_UNREACHABLE", [], fleet_task_id, gateway_goal_active,
                         "RMF booking 상태를 확인하지 못했습니다.")
    if not bookings:
        return _summary("CONSISTENT", [], fleet_task_id, gateway_goal_active, None)
    if not fleet_known:
        rows = [_row(item, robot_id, "PENDING_CONFIRMATION", False) for item in bookings]
        ident = ", ".join(row["booking_id"] for row in rows)
        return _summary("PENDING_CONFIRMATION", rows, fleet_task_id, gateway_goal_active,
                         f"Fleet의 현재 task id를 확인하지 못했습니다: {ident}")
    rows = []
    worst = "CONSISTENT"
    for booking in bookings:
        booking_id = str(booking.get("id") or "")
        status = str(booking.get("status") or "").lower()
        owned = bool(booking_id) and fleet_task_id == booking_id
        if status not in NONTERMINAL:
            rows.append(_row(booking, robot_id, "CONSISTENT", owned))
            continue
        if owned:
            rows.append(_row(booking, robot_id, "CONSISTENT", True, gateway_goal_active))
            continue
        if gateway_goal_active:
            rows.append(_row(booking, robot_id, "STATE_MISMATCH", False, True))
            worst = _worse(worst, "STATE_MISMATCH")
            continue
        record = log.observe(robot_id, booking_id, at_ms, booking.get("updated_ms"))
        kind = _unowned(record, at_ms, window_ms)
        rows.append(_row(booking, robot_id, kind, False, False))
        worst = _worse(worst, kind)
    reason = None
    if worst != "CONSISTENT":
        ident = ", ".join(row["booking_id"] for row in rows if row["reconciliation_status"] == worst)
        reason = _reason(worst, ident)
    elif any(not row["dispatch_allowed"] for row in rows):
        ident = ", ".join(row["booking_id"] for row in rows if not row["dispatch_allowed"])
        reason = f"이 로봇의 RMF task가 아직 끝나지 않았습니다: {ident}"
        worst_state = "BLOCKED"
        return _summary(worst, rows, fleet_task_id, gateway_goal_active, reason, state=worst_state)
    return _summary(worst, rows, fleet_task_id, gateway_goal_active, reason)


def _unowned(record: dict, at_ms: int, window_ms: int) -> str:
    """Ownership is lost only after the observation window, not because a timestamp is old."""
    samples = record["samples"]
    if len(samples) < 2:
        return "PENDING_CONFIRMATION"
    updated = [item[1] for item in samples if item[1] is not None]
    stable = len(set(updated)) <= 1
    span = int(at_ms) - int(record["started_ms"])
    if stable and span >= window_ms:
        return "OWNER_MISSING"
    return "PENDING_CONFIRMATION"


def _worse(current: str, new: str) -> str:
    rank = {
        "CONSISTENT": 0,
        "PENDING_CONFIRMATION": 1,
        "STATE_MISMATCH": 2,
        "OWNER_MISSING": 3,
        "RECOVERY_REQUIRED": 4,
        "RMF_UNREACHABLE": 5,
        "GATEWAY_UNREACHABLE": 6,
    }
    return new if rank.get(new, 0) > rank.get(current, 0) else current


def _reason(kind: str, ident: str) -> str:
    if kind == "OWNER_MISSING":
        return f"Fleet Adapter가 RMF task를 소유하지 않습니다: {ident}"
    if kind == "PENDING_CONFIRMATION":
        return f"RMF task 소유권을 아직 확정하지 못했습니다: {ident}"
    if kind == "STATE_MISMATCH":
        return f"Gateway goal과 RMF task 소유가 일치하지 않습니다: {ident}"
    return f"RMF task를 확인할 수 없습니다: {ident}"


def _row(booking: dict, robot_id: str, kind: str, owned: bool, gateway_goal_active: bool = False) -> dict:
    status = str(booking.get("status") or booking.get("rmf_status") or "")
    return {
        "booking_id": str(booking.get("id") or booking.get("booking_id") or ""),
        "robot_id": booking.get("robot_id") or robot_id,
        "fleet": booking.get("fleet"),
        "rmf_status": status,
        "updated_ms": booking.get("updated_ms"),
        "adapter_task_owned": owned,
        "gateway_goal_active": gateway_goal_active,
        "reconciliation_status": kind,
        "dispatch_allowed": kind == "CONSISTENT" and status.lower() not in NONTERMINAL,
        "recovery_required": kind in BLOCKING,
    }


def _summary(kind: str, rows: list[dict], fleet_task_id: str, gateway_goal_active: bool,
             reason: str | None, state: str | None = None) -> dict:
    recovery_required = kind in BLOCKING
    if state is None:
        state = "CLEAR" if kind == "CONSISTENT" else ("RECOVERY_REQUIRED" if recovery_required else "BLOCKED")
    owned = any(row.get("adapter_task_owned") for row in rows)
    return {
        "reconciliation_status": kind,
        "recovery_required": recovery_required,
        "dispatch_allowed": state == "CLEAR",
        "state": state,
        "reason": reason,
        "fleet_task_id": fleet_task_id or "",
        "gateway_goal_active": gateway_goal_active,
        "adapter_task_owned": owned,
        "bookings": rows,
        "reports": rows,
    }
