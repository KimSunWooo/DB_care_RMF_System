"""Orphaned-task reconciliation. No gateway command and no operating database."""
from app.isolated_env import ISOLATED_STACK, isolation_problems
from app.task_reconcile import ReconciliationLog, judge


def _booking(booking_id="7c5e2da1-e89a-44e3-9d68-591a027c4922", robot_id="real_pinky1",
             updated_ms=1_000, status="standby"):
    return {"id": booking_id, "status": status, "robot_id": robot_id, "fleet": "physical_pinky",
            "updated_ms": updated_ms}


def _judge(log, at_ms, bookings, fleet_task_id="", gateway_goal_active=False, rmf_ok=True,
           gateway_ok=True, fleet_known=True, robot_id="real_pinky1"):
    return judge(
        robot_id=robot_id, rmf_ok=rmf_ok, gateway_ok=gateway_ok, fleet_known=fleet_known,
        bookings=bookings, fleet_task_id=fleet_task_id, gateway_goal_active=gateway_goal_active,
        at_ms=at_ms, log=log,
    )


def test_one_sample_does_not_declare_owner_missing():
    log = ReconciliationLog()
    first = _judge(log, 20_000, [_booking()])
    assert first["reconciliation_status"] == "PENDING_CONFIRMATION"
    assert first["recovery_required"] is True
    assert first["dispatch_allowed"] is False
    assert first["reports"][0]["adapter_task_owned"] is False
    assert first["reports"][0]["rmf_status"] == "standby"


def test_old_rmf_timestamp_alone_does_not_confirm_owner_missing():
    log = ReconciliationLog()
    _judge(log, 20_000, [_booking()])
    second = _judge(log, 21_000, [_booking()])
    assert second["reconciliation_status"] == "PENDING_CONFIRMATION"
    assert second["recovery_required"] is True
    assert second["dispatch_allowed"] is False
    assert second["reports"][0]["rmf_status"] == "standby"


def test_short_sample_buffer_still_keeps_the_first_observation():
    log = ReconciliationLog()
    booking = _booking(updated_ms=10**15)
    result = None
    for index in range(16):
        result = _judge(log, 1_000 + index * 1_000, [booking])
    assert result["reconciliation_status"] == "OWNER_MISSING"


def test_a_young_unowned_task_stays_pending():
    log = ReconciliationLog()
    booking = _booking(updated_ms=20_000)
    _judge(log, 21_000, [booking])
    second = _judge(log, 22_000, [booking])
    assert second["reconciliation_status"] == "PENDING_CONFIRMATION"


def test_adapter_restart_does_not_restore_or_resubmit():
    log = ReconciliationLog()
    owned = _judge(log, 1_000, [_booking(updated_ms=1_000)], fleet_task_id=_booking()["id"])
    assert owned["reconciliation_status"] == "CONSISTENT"
    assert owned["dispatch_allowed"] is False
    restarted = ReconciliationLog()
    _judge(restarted, 40_000, [_booking()])
    lost = _judge(restarted, 55_000, [_booking()])
    assert lost["reconciliation_status"] == "OWNER_MISSING"
    assert lost["reports"][0]["rmf_status"] == "standby"


def test_rmf_and_gateway_failures_are_not_owner_missing():
    log = ReconciliationLog()
    rmf = _judge(log, 10_000, [_booking()], rmf_ok=False)
    gateway = _judge(log, 10_000, [_booking()], gateway_ok=False)
    assert rmf["reconciliation_status"] == "RMF_UNREACHABLE"
    assert gateway["reconciliation_status"] == "GATEWAY_UNREACHABLE"
    assert rmf["recovery_required"] and gateway["recovery_required"]


def test_owned_goal_stays_consistent_and_an_unowned_goal_is_a_mismatch():
    log = ReconciliationLog()
    booking = _booking()
    owned = _judge(log, 5_000, [booking], fleet_task_id=booking["id"])
    moving = _judge(log, 5_000, [booking], fleet_task_id=booking["id"], gateway_goal_active=True)
    mismatch = _judge(log, 5_000, [booking], gateway_goal_active=True)
    assert owned["reconciliation_status"] == "CONSISTENT"
    assert owned["state"] == "BLOCKED"
    assert moving["reconciliation_status"] == "CONSISTENT"
    assert moving["dispatch_allowed"] is False
    assert mismatch["reconciliation_status"] == "STATE_MISMATCH"


def test_another_robot_is_not_marked_from_this_booking():
    log = ReconciliationLog()
    clear = _judge(log, 50_000, [], robot_id="real_pinky2")
    assert clear["reconciliation_status"] == "CONSISTENT"
    assert clear["state"] == "CLEAR"
    assert clear["recovery_required"] is False


def test_isolated_stack_does_not_use_the_operating_endpoints():
    assert isolation_problems(ISOLATED_STACK) == []
    leaked = dict(ISOLATED_STACK, ros_domain=77, api_port=8000, gateway_port=17453,
                   database="/run/rmf_api_server.db", cmd_vel="/cmd_vel")
    problems = isolation_problems(leaked)
    assert any("77" in item for item in problems)
    assert any("8000" in item for item in problems)
    assert any("17453" in item for item in problems)
    assert any("cmd_vel" in item for item in problems)


def test_admin_reconcile_reads_without_arming(monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    calls = []

    async def report(robot_id, gateway=None):
        assert gateway is not None
        if robot_id != "real_pinky1":
            return {"state": "CLEAR", "reconciliation_status": "CONSISTENT", "bookings": [],
                    "reports": [], "recovery_required": False, "dispatch_allowed": True,
                    "adapter_task_owned": False, "fleet_task_id": "", "reason": None}
        return {
            "state": "RECOVERY_REQUIRED",
            "reconciliation_status": "OWNER_MISSING",
            "recovery_required": True,
            "dispatch_allowed": False,
            "adapter_task_owned": False,
            "fleet_task_id": "",
            "reason": "Fleet Adapter가 RMF task를 소유하지 않습니다: 7c5e2da1",
            "bookings": [{"id": "7c5e2da1-e89a-44e3-9d68-591a027c4922", "rmf_status": "standby",
                          "adapter_task_owned": False, "reconciliation_status": "OWNER_MISSING"}],
            "reports": [],
        }

    monkeypatch.delenv("CARE_MOVE_ENV", raising=False)
    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    async def refresh():
        return {"robots": [], "motion": False, "duplicate": False}

    monkeypatch.setattr(main, "booking_recovery", report)
    monkeypatch.setattr(main, "refresh_recovery", refresh)
    monkeypatch.setattr(main, "read_physical_gateway", lambda: {"reachable": True, "goal_phase": "idle"})
    monkeypatch.setattr(main, "gateway_command", lambda payload, timeout=2.0: calls.append(payload))
    client = TestClient(main.app)
    denied = client.post("/api/admin/recovery/reconcile")
    allowed = client.post("/api/admin/recovery/reconcile", headers={"Authorization": "Bearer operator-token"})
    one = client.get("/api/admin/recovery/tasks/7c5e2da1-e89a-44e3-9d68-591a027c4922",
                     headers={"Authorization": "Bearer operator-token"})
    assert denied.status_code == 401
    assert allowed.status_code == 200
    assert allowed.json()["motion"] is False
    assert one.json()["reconciliation_status"] == "OWNER_MISSING"
    assert one.json()["rmf_status"] == "standby"
    assert calls == []


def test_read_path_does_not_add_an_observation(monkeypatch):
    import asyncio
    from app import main

    def explode(**_kwargs):
        raise AssertionError("observed")

    monkeypatch.setattr(main, "judge", explode)
    first = asyncio.run(main.booking_recovery("real_pinky1"))
    second = asyncio.run(main.booking_recovery("real_pinky1"))
    assert first["dispatch_allowed"] is False
    assert second == first


def test_worker_confirms_only_after_the_window_and_reads_do_not_change_it(tmp_path):
    from app.reconcile_worker import ReconcileService
    service = ReconcileService(tmp_path / "reconcile.sqlite", min_gap_ms=500)
    booking = _booking()

    def epoch(mono, fleet_task_id=""):
        assert service.begin_epoch(mono)
        report = _judge(service.log, mono, [booking], fleet_task_id=fleet_task_id)
        service.commit_epoch({"real_pinky1": report}, mono, utc="2026-10-23T07:00:00Z")
        return service.view("real_pinky1", mono)

    assert epoch(1_000)["reconciliation_status"] == "PENDING_CONFIRMATION"
    assert service.begin_epoch(1_100) is False
    assert epoch(16_000)["reconciliation_status"] == "OWNER_MISSING"
    assert service.view("real_pinky1", 16_000)["reports"][0]["rmf_status"] == "standby"
    fresh = ReconcileService(tmp_path / "reconcile.sqlite", min_gap_ms=0)
    assert fresh.view("real_pinky1", 16_000)["observed"] is False
    assert fresh.view("real_pinky1", 16_000)["dispatch_allowed"] is False
    assert fresh.historical()[0]["historical_classification"] == "ORPHANED"
    assert fresh.historical()[0]["rmf_status"] == "standby"


def test_stale_snapshot_and_clock_jump_block_without_confirming_loss(tmp_path):
    from app.reconcile_worker import ReconcileService
    service = ReconcileService(tmp_path / "reconcile.sqlite", min_gap_ms=0, stale_ms=1_000)
    assert service.begin_epoch(0)
    clear = _judge(service.log, 0, [], robot_id="real_pinky2")
    service.commit_epoch({"real_pinky2": clear}, 0)
    stale = service.view("real_pinky2", 5_000)
    assert stale["dispatch_allowed"] is False
    assert stale["snapshot_stale"] is True
    assert service.begin_epoch(100)
    pending = _judge(service.log, 100, [_booking()])
    service.commit_epoch({"real_pinky1": pending}, 100)
    assert service.view("real_pinky1", 100)["reconciliation_status"] == "PENDING_CONFIRMATION"
    assert service.begin_epoch(50)
    rewound = _judge(service.log, 50, [_booking()])
    service.commit_epoch({"real_pinky1": rewound}, 50)
    assert service.view("real_pinky1", 50)["reconciliation_status"] == "PENDING_CONFIRMATION"


def test_environments_do_not_share_tasks_or_the_physical_lease():
    from app.rmf_environment import accept_event, apply_cutover, cutover_report, fence_physical, goal_event_current
    assert fence_physical("rmf-b-domain91", 17453)
    assert fence_physical("rmf-a-domain77", 17453) is None
    assert accept_event("rmf-b-domain91", "rmf-a-domain77") is False
    assert goal_event_current("session-2", "session-1", 4, 3) is False
    report = cutover_report({"operator_approved": False})
    assert report["applied"] is False and report["ready"] is False
    try:
        apply_cutover({name: True for name in (
            "old_environment_fenced", "no_active_physical_goal", "robot_stopped",
            "gateway_dry_run", "old_task_cannot_replay", "new_database_independent",
            "new_task_lifecycle_verified", "rollback_plan_ready", "operator_approved")})
    except PermissionError as exc:
        assert "not executed" in str(exc)
    else:
        raise AssertionError("cutover was applied")


def test_duplicate_request_id_is_one_row(tmp_path):
    import threading
    from app.task_store import TaskStore
    store = TaskStore(tmp_path / "queue.sqlite")
    errors = []

    def put():
        try:
            store.put({"request_id": "same", "robot_id": "mock_pinky1", "status": "QUEUED",
                       "dispatch_allowed": False})
        except ValueError:
            errors.append("duplicate")

    threads = [threading.Thread(target=put) for _ in range(2)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert store.get("same")["robot_id"] == "mock_pinky1"
    assert errors == ["duplicate"]
