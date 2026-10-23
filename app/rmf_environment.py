"""Separate the legacy RMF environment from the isolated one.

A new environment id does not grant the physical gateway lease.
"""
from __future__ import annotations

LEGACY = {
    "environment_id": "rmf-a-domain77",
    "api": "http://127.0.0.1:8000",
    "ros_domain": 77,
    "database": "/tmp/care-move-e2e/rmf_api_server.db",
    "gateway_port": 17453,
    "physical": True,
    "role": "legacy",
}
ISOLATED = {
    "environment_id": "rmf-b-domain91",
    "api": "http://127.0.0.1:8001",
    "ros_domain": 91,
    "database": "/tmp/care-move-isolated/rmf.sqlite",
    "gateway_port": 17491,
    "physical": False,
    "role": "isolated",
}
PHYSICAL_GATEWAY_PORT = 17453
LEASE_HOLDER = LEGACY["environment_id"]

CUTOVER_REQUIREMENTS = (
    "old_environment_fenced",
    "no_active_physical_goal",
    "robot_stopped",
    "gateway_dry_run",
    "old_task_cannot_replay",
    "new_database_independent",
    "new_task_lifecycle_verified",
    "rollback_plan_ready",
    "operator_approved",
)


def same_task(left_environment: str, left_booking: str, right_environment: str, right_booking: str) -> bool:
    return left_environment == right_environment and left_booking == right_booking


def accept_event(active_environment: str, event_environment: str) -> bool:
    return active_environment == event_environment


def goal_event_current(session_id: str, event_session_id: str, session_generation: int,
                       event_generation: int) -> bool:
    """A late completed event from an older session does not finish the current one."""
    if not session_id or session_id != event_session_id:
        return False
    return int(session_generation) == int(event_generation)


def fence_physical(environment_id: str, gateway_port: int) -> str | None:
    """Return a refusal reason. None means this check does not block."""
    if gateway_port != PHYSICAL_GATEWAY_PORT:
        return None
    if environment_id != LEASE_HOLDER:
        return "이 환경은 실물 Gateway 제어권이 없습니다."
    if environment_id == ISOLATED["environment_id"]:
        return "격리 환경은 실물 Gateway에 명령을 보내지 않습니다."
    return None


def cutover_report(checks: dict) -> dict:
    missing = [name for name in CUTOVER_REQUIREMENTS if checks.get(name) is not True]
    ready = not missing
    return {
        "status": "SAFE_CUTOVER_READY" if ready else "RECOVERY_REQUIRED",
        "ready": ready,
        "applied": False,
        "missing": missing,
        "meaning": "ready는 조건이 문서상 충족됐다는 뜻이다. 이 함수는 제어권을 옮기지 않는다.",
    }


def apply_cutover(checks: dict) -> dict:
    report = cutover_report(checks)
    if not report["ready"]:
        raise PermissionError("cutover refused: " + ", ".join(report["missing"]))
    raise PermissionError("physical cutover is not executed by this process")
