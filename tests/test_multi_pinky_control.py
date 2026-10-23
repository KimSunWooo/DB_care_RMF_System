import uuid

from fleet_adapter.gateway_mode import DRY_RUN, FAULT, PHYSICAL_TEST, ModeTable
from fleet_adapter.physical_motion import FORWARD_TEST


def _robot(available=True):
    return {"available": available, "motor_ready": True, "odom_fresh": True, "pose": [0, 0, 0], "speed": 0, "angular": 0,
            "cmd_vel_topic": "/odom" if False else "/cmd_vel"}


def test_arm_is_per_robot_and_refuses_a_moving_goal():
    table = ModeTable()
    assert table.get("real_pinky2") == DRY_RUN
    armed = table.arm("real_pinky1", _robot(), "idle", False, FORWARD_TEST)
    assert armed["ok"] is True and table.get("real_pinky1") == PHYSICAL_TEST
    assert table.get("real_pinky2") == DRY_RUN
    again = table.arm("real_pinky1", _robot(), "idle", False, FORWARD_TEST)
    assert again["already"] is True
    refused = table.arm("real_pinky2", _robot(), "moving", False, FORWARD_TEST)
    assert refused["ok"] is False and table.get("real_pinky2") == DRY_RUN
    hidden = table.arm("real_pinky3", _robot(False), "idle", False, FORWARD_TEST)
    assert hidden["ok"] is False and table.get("real_pinky3") == DRY_RUN


def test_disarm_requires_a_confirmed_stop():
    table = ModeTable()
    table.arm("real_pinky1", _robot(), "idle", False, FORWARD_TEST)
    fault = table.disarm("real_pinky1", 0.2, 0.0, "moving")
    assert fault["mode"] == FAULT
    assert table.arm("real_pinky1", _robot(), "idle", False, FORWARD_TEST)["ok"] is False
    stopped = ModeTable()
    stopped.arm("real_pinky1", _robot(), "idle", False, FORWARD_TEST)
    done = stopped.disarm("real_pinky1", 0.0, 0.0, "succeeded")
    assert done["mode"] == DRY_RUN
    assert stopped.get("real_pinky2") == DRY_RUN


def test_queue_is_fifo_and_restart_does_not_resubmit(tmp_path):
    from app.task_store import TaskStore, promote_fifo
    store = TaskStore(tmp_path / "queue.sqlite")
    store.put({"request_id": "a", "robot_id": "real_pinky1", "status": "QUEUED", "dispatch_allowed": True})
    store.put({"request_id": "b", "robot_id": "real_pinky1", "status": "QUEUED", "dispatch_allowed": True})
    calls = []

    def dispatcher(row):
        calls.append(row["request_id"])
        return {"submitted": True, "rmf_booking_id": "book-" + row["request_id"]}

    promote_fifo(store, "real_pinky1", dispatcher)
    promote_fifo(store, "real_pinky1", dispatcher)
    assert calls == ["a"]
    store.mark("a", "COMPLETED")
    promote_fifo(store, "real_pinky1", dispatcher)
    assert calls == ["a", "b"]
    reopened = TaskStore(tmp_path / "queue.sqlite")
    assert reopened.get("b")["status"] == "INTERRUPTED"
    assert promote_fifo(reopened, "real_pinky1", dispatcher) is None
    canceled = reopened.put({"request_id": "c", "robot_id": "real_pinky1", "status": "QUEUED", "dispatch_allowed": False})
    assert canceled["status"] == "QUEUED"
    assert reopened.cancel_queued("c")["status"] == "CANCELED"


def test_transport_stays_queued_and_rejects_bad_requests(tmp_path, monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    from app.task_store import TaskStore
    from app.transport_requests import accept_transport
    store = TaskStore(tmp_path / "transport.sqlite")
    robots = [{"robot_id": "real_pinky1", "status": "IDLE", "available": True},
              {"robot_id": "real_pinky2", "status": "OFFLINE", "available": False}]
    waypoints = {"N_ROOM102": {"floor": "1F"}, "N_PT": {"floor": "1F"}, "N_CAFE": {"floor": "7F"}}
    queued = accept_transport(store, {"request_id": "move-1", "origin": "N_ROOM102", "destination": "N_PT",
                                      "action": "MOVE", "trip_type": "ROUND_TRIP", "robot_id": "real_pinky1"},
                              waypoints, robots)
    assert queued["status"] == "QUEUED" and queued["rmf_booking_id"] is None
    assert queued["execution"] == "not_submitted"
    phases = queued["phases"]
    assert phases[0]["assumes_robot_at_origin"] is False
    assert any(item["step"] == "return_to_origin" for item in phases)
    try:
        accept_transport(store, {"request_id": "move-1", "origin": "N_ROOM102", "destination": "N_PT", "action": "MOVE"},
                         waypoints, robots)
    except ValueError as exc:
        assert "duplicate" in str(exc)
    else:
        raise AssertionError("duplicate request was accepted")
    try:
        accept_transport(store, {"origin": "N_ROOM102", "destination": "N_CAFE", "action": "MOVE"}, waypoints, robots)
    except ValueError as exc:
        assert "floors" in str(exc)
    else:
        raise AssertionError("cross-floor request was accepted")
    try:
        accept_transport(store, {"origin": "N_ROOM102", "action": "CALL", "robot_id": "real_pinky2"}, waypoints, robots)
    except ValueError as exc:
        assert "offline" in str(exc)
    else:
        raise AssertionError("offline robot was accepted")
    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    monkeypatch.setattr(main, "task_store", store)
    monkeypatch.setattr(main, "_transport_robots", lambda: robots)
    production = main.ROOT / "data" / "task_queue.sqlite"
    before = production.read_bytes() if production.exists() else b""
    client = TestClient(main.app)
    denied = client.post("/api/v1/transport/requests", json={"origin": "N_ROOM102", "destination": "N_PT", "action": "MOVE"})
    assert denied.status_code == 401
    request_id = "api-move-" + uuid.uuid4().hex
    allowed = client.post(
        "/api/v1/transport/requests",
        headers={"Authorization": "Bearer operator-token"},
        json={"request_id": request_id, "origin": "N_ROOM102", "destination": "N_PT", "action": "MOVE"},
    )
    assert allowed.status_code == 202
    body = allowed.json()
    assert body["rmf_booking_id"] is None and body["status"] == "QUEUED"
    assert store.get(request_id)["status"] == "QUEUED"
    assert (production.read_bytes() if production.exists() else b"") == before


def _ready_status():
    return {
        "reachable": True, "mode": "PHYSICAL_DRY_RUN", "domain": 24,
        "selected_robot_id": "real_pinky1", "robot_id": "real_pinky1",
        "selected_available": True, "motor_ready": True, "odom_fresh": True,
        "pose": [0.0, 0.0, 0.0], "odom_velocity": 0.0, "odom_angular": 0.0,
        "goal_phase": "idle", "session_open": False, "session_submitted": False,
        "gateway_version": "test", "session_protocol_version": 2,
        "supported_test_kinds": ["forward_5cm", "rotate_180"],
    }


async def _clear_bookings(_robot_id, _gateway=None):
    return {"state": "CLEAR", "bookings": [], "reason": None}


def test_leftover_success_does_not_release_a_new_session():
    from app.main import dry_run_ready
    leftover = {"session_id": "new-session", "session_submitted": False, "goal_phase": "succeeded",
                "goal_generation": 1, "session_goal_generation": 1,
                "odom_velocity": 0.0, "odom_angular": 0.0}
    assert dry_run_ready(leftover, "new-session") is False
    finished = {**leftover, "session_submitted": True}
    assert dry_run_ready(finished, "new-session") is True
    assert dry_run_ready({**finished, "goal_phase": "moving"}, "new-session") is False
    assert dry_run_ready(finished, "other-session") is False
    assert dry_run_ready({**finished, "session_goal_generation": None}, "new-session") is False
    assert dry_run_ready({**finished, "goal_generation": 4, "session_goal_generation": None}, "new-session") is False
    from app.main import stopped_terminal
    assert stopped_terminal(leftover, "new-session") is False
    assert stopped_terminal({**finished, "goal_phase": "moving"}, "new-session") is False
    assert stopped_terminal(finished, "new-session") is True


def test_unrecoverable_refusal_does_not_include_an_active_goal():
    from fleet_adapter.physical_motion import unrecoverable_goal
    assert unrecoverable_goal("robot is not in PHYSICAL_TEST") is True
    assert unrecoverable_goal("a physical goal is already active") is False
    assert unrecoverable_goal("duplicate physical goal", duplicate=True) is False


def test_local_ui_skips_the_token_only_from_loopback(monkeypatch):
    from fastapi.testclient import TestClient
    from starlette.requests import Request
    from app import main
    monkeypatch.delenv("CARE_MOVE_ENV", raising=False)
    monkeypatch.delenv("CARE_MOVE_LOCAL_TEST_UI_ENABLED", raising=False)
    calls = []
    monkeypatch.setattr(main, "gateway_command", lambda payload, timeout=2.0: calls.append(payload) or {"ok": True})
    client = TestClient(main.app)
    assert client.post("/api/physical/test", json={"kind": "forward_5cm", "robot_id": "real_pinky1"}).status_code == 401
    monkeypatch.setenv("CARE_MOVE_ENV", "local")
    monkeypatch.setenv("CARE_MOVE_LOCAL_TEST_UI_ENABLED", "true")
    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    foreign = client.post(
        "/api/physical/test",
        headers={"Origin": "http://evil.example", "Host": "127.0.0.1:8011"},
        json={"kind": "forward_5cm", "robot_id": "real_pinky1"},
    )
    assert foreign.status_code == 403 and calls == []
    transport = client.post("/api/v1/transport/requests", json={"origin": "N_ROOM102", "destination": "N_PT", "action": "MOVE"})
    assert transport.status_code == 401
    scope = {
        "type": "http", "method": "POST", "scheme": "http", "path": "/api/physical/test",
        "query_string": b"", "server": ("127.0.0.1", 8011), "client": ("127.0.0.1", 40000),
        "headers": [(b"host", b"127.0.0.1:8011"), (b"origin", b"http://127.0.0.1:8011")],
    }
    main.authorize_drive(Request(scope), "forward_5cm", "real_pinky1")
    proxied = Request({**scope, "headers": scope["headers"] + [(b"x-forwarded-for", b"8.8.8.8")]})
    try:
        main.authorize_drive(proxied, "forward_5cm", "real_pinky1")
    except Exception as exc:
        assert getattr(exc, "status_code", None) == 403
    else:
        raise AssertionError("proxied drive was accepted")


def test_drive_endpoints_reject_missing_and_wrong_tokens(monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    calls = []
    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    monkeypatch.setattr(main, "gateway_command", lambda payload, timeout=2.0: calls.append(payload) or {"ok": True})
    monkeypatch.setattr(main, "read_physical_gateway", _ready_status)
    client = TestClient(main.app)
    missing = client.post("/api/physical/test", json={"kind": "forward_5cm", "robot_id": "real_pinky1"})
    wrong = client.post(
        "/api/physical/test-5cm",
        headers={"Authorization": "Bearer wrong-token"},
    )
    assert missing.status_code == 401 and wrong.status_code == 401
    assert calls == []
    session = client.post("/api/physical/session", headers={"Authorization": "Bearer operator-token"})
    assert session.status_code == 204
    assert calls == []


def test_failed_arm_does_not_start_the_fleet_adapter(monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    calls = []

    def command(payload, timeout=2.0):
        calls.append(payload["op"])
        if payload["op"] == "prepare":
            return {"ok": False, "reason": "odometry is not fresh"}
        return {"ok": True, "mode": "PHYSICAL_DRY_RUN"}

    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    monkeypatch.setattr(main, "MODE", "rmf")
    monkeypatch.setattr(main, "gateway_command", command)
    monkeypatch.setattr(main, "read_physical_gateway", _ready_status)
    monkeypatch.setattr(main, "active_physical_task", lambda: None)
    monkeypatch.setattr(main, "booking_recovery", _clear_bookings)

    def refuse_adapter():
        raise AssertionError("adapter must stay running")

    monkeypatch.setattr(main, "stop_physical_adapter", refuse_adapter)
    monkeypatch.setattr(main, "start_physical_adapter", refuse_adapter)
    response = TestClient(main.app).post(
        "/api/physical/test",
        headers={"Authorization": "Bearer operator-token"},
        json={"kind": "forward_5cm", "robot_id": "real_pinky1"},
    )
    assert response.status_code == 409
    assert calls == ["select", "arm", "prepare", "disarm"]
    main.MODE = "mock"


def test_physical_cancel_stops_the_gateway_before_rmf(monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    calls = []

    def command(payload, timeout=2.0):
        calls.append(payload)
        return {"ok": True, "goal": {"phase": "canceled"}}

    async def rmf_cancel(task_id):
        calls.append({"op": "rmf", "task_id": task_id})
        return {"success": True}

    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    monkeypatch.setattr(main, "gateway_command", command)
    monkeypatch.setattr(main.rmf_client, "cancel_task", rmf_cancel)
    main.TASKS["local-physical"] = {
        "id": "local-physical", "rmf_task_id": "booking-1", "robot_id": "real_pinky1",
        "source": "RMF", "status": "rmf_accepted_submission", "completed": False,
    }
    client = TestClient(main.app)
    denied = client.post("/api/tasks/local-physical/cancel")
    assert denied.status_code == 401 and calls == []
    allowed = client.post(
        "/api/tasks/local-physical/cancel",
        headers={"Authorization": "Bearer operator-token"},
    )
    assert allowed.status_code == 200
    assert calls[0]["op"] == "cancel" and calls[0]["robot_id"] == "real_pinky1"
    assert calls[1] == {"op": "rmf", "task_id": "booking-1"}
    assert allowed.json()["completed"] is False
    main.TASKS.pop("local-physical", None)


def test_map_registry_does_not_treat_the_trial_graph_as_a_facility_map():
    from app.map_registry import load_registry
    from pathlib import Path
    report = load_registry(Path(__file__).resolve().parents[1])
    assert report["physical_navigation_ready"] is False
    trial = next(item for item in report["maps"] if item["map_id"] == "physical_odom_trial")
    assert trial["physical_aligned"] is False
    concept = next(item for item in report["maps"] if item["kind"] == "ui_concept")
    assert concept["physical_aligned"] is False


def _booking(task_id, status, group, name, category="Patrol"):
    return {"status": status, "category": category, "assigned_to": {"group": group, "name": name},
            "booking": {"id": task_id}}


def test_unfinished_booking_blocks_only_the_assigned_robot():
    from app.task_guard import conflicting_bookings
    states = [
        _booking("7c5e2da1-e89a-44e3-9d68-591a027c4922", "standby", "physical_pinky", "real_pinky1"),
        _booking("gazebo-live", "underway", "gazebo_pinky1", "pinky1"),
        _booking("done", "completed", "physical_pinky", "real_pinky1"),
    ]
    pinky1 = conflicting_bookings(states, "real_pinky1")
    pinky2 = conflicting_bookings(states, "real_pinky2")
    assert [item["id"] for item in pinky1] == ["7c5e2da1-e89a-44e3-9d68-591a027c4922"]
    assert pinky2 == []
    unassigned = [_booking("open-fleet", "queued", "physical_pinky", "")]
    assert conflicting_bookings(unassigned, "real_pinky2")[0]["id"] == "open-fleet"


def test_standby_booking_rejects_a_new_physical_task_before_arm(monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    calls = []

    def command(payload, timeout=2.0):
        calls.append(payload["op"])
        return {"ok": True}

    async def stuck(_robot_id, _gateway=None):
        return {"state": "RECOVERY_REQUIRED", "bookings": [{"id": "7c5e2da1-e89a-44e3-9d68-591a027c4922", "status": "standby"}],
                "reason": "이전 RMF booking이 종료 상태가 아닙니다: 7c5e2da1-e89a-44e3-9d68-591a027c4922"}

    monkeypatch.delenv("CARE_MOVE_ENV", raising=False)
    monkeypatch.delenv("CARE_MOVE_LOCAL_TEST_UI_ENABLED", raising=False)
    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    monkeypatch.setattr(main, "MODE", "rmf")
    monkeypatch.setattr(main, "gateway_command", command)
    monkeypatch.setattr(main, "read_physical_gateway", _ready_status)
    monkeypatch.setattr(main, "booking_recovery", stuck)
    def refuse_adapter():
        raise AssertionError("adapter started")
    monkeypatch.setattr(main, "start_physical_adapter", refuse_adapter)
    response = TestClient(main.app).post(
        "/api/physical/test",
        headers={"Authorization": "Bearer operator-token"},
        json={"kind": "forward_5cm", "robot_id": "real_pinky1"},
    )
    assert response.status_code == 409
    assert "7c5e2da1" in response.json()["detail"]
    assert calls == ["select"]
    main.MODE = "mock"


def test_unknown_booking_lookup_does_not_arm(monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    calls = []
    monkeypatch.delenv("CARE_MOVE_ENV", raising=False)
    monkeypatch.delenv("CARE_MOVE_LOCAL_TEST_UI_ENABLED", raising=False)
    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    monkeypatch.setattr(main, "MODE", "rmf")
    monkeypatch.setattr(main, "gateway_command", lambda payload, timeout=2.0: calls.append(payload["op"]) or {"ok": True})
    monkeypatch.setattr(main, "read_physical_gateway", _ready_status)

    async def unknown(_robot_id, _gateway=None):
        return {"state": "UNKNOWN", "bookings": [], "reason": "RMF booking 상태를 확인하지 못했습니다."}

    monkeypatch.setattr(main, "booking_recovery", unknown)
    response = TestClient(main.app).post(
        "/api/physical/test",
        headers={"Authorization": "Bearer operator-token"},
        json={"kind": "rotate_180", "robot_id": "real_pinky1"},
    )
    assert response.status_code == 409 and calls == ["select"]
    main.MODE = "mock"


def test_old_gateway_protocol_rejects_arm(monkeypatch):
    from fastapi.testclient import TestClient
    from fleet_adapter.physical_motion import gateway_protocol_block
    from app import main
    calls = []
    stale = _ready_status()
    stale.pop("session_protocol_version")
    stale.pop("supported_test_kinds")
    assert gateway_protocol_block(stale)
    monkeypatch.delenv("CARE_MOVE_ENV", raising=False)
    monkeypatch.delenv("CARE_MOVE_LOCAL_TEST_UI_ENABLED", raising=False)
    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    monkeypatch.setattr(main, "MODE", "rmf")
    monkeypatch.setattr(main, "gateway_command", lambda payload, timeout=2.0: calls.append(payload["op"]) or {"ok": True})
    monkeypatch.setattr(main, "read_physical_gateway", lambda: stale)
    monkeypatch.setattr(main, "booking_recovery", _clear_bookings)
    response = TestClient(main.app).post(
        "/api/physical/test",
        headers={"Authorization": "Bearer operator-token"},
        json={"kind": "forward_5cm", "robot_id": "real_pinky1"},
    )
    assert response.status_code == 409 and "프로토콜" in response.json()["detail"]
    assert calls == ["select"]
    main.MODE = "mock"


def test_clear_preflight_reaches_arm_without_moving(monkeypatch):
    from fastapi.testclient import TestClient
    from app import main
    calls = []

    def command(payload, timeout=2.0):
        calls.append(payload["op"])
        if payload["op"] == "prepare":
            return {"ok": False, "reason": "odometry is not fresh"}
        return {"ok": True, "mode": "PHYSICAL_DRY_RUN"}

    monkeypatch.delenv("CARE_MOVE_ENV", raising=False)
    monkeypatch.delenv("CARE_MOVE_LOCAL_TEST_UI_ENABLED", raising=False)
    monkeypatch.setenv("CARE_MOVE_OPERATOR_TOKEN", "operator-token")
    monkeypatch.setattr(main, "MODE", "rmf")
    monkeypatch.setattr(main, "gateway_command", command)
    monkeypatch.setattr(main, "read_physical_gateway", _ready_status)
    monkeypatch.setattr(main, "booking_recovery", _clear_bookings)
    def refuse_adapter():
        raise AssertionError("adapter started")
    monkeypatch.setattr(main, "start_physical_adapter", refuse_adapter)
    response = TestClient(main.app).post(
        "/api/physical/test",
        headers={"Authorization": "Bearer operator-token"},
        json={"kind": "rotate_180", "robot_id": "real_pinky1"},
    )
    assert response.status_code == 409
    assert calls == ["select", "arm", "prepare", "disarm"]
    main.MODE = "mock"


def test_schedule_view_does_not_invent_trajectories():
    from app.schedule_view import schedule_view
    view = schedule_view()
    assert view["trajectories"] == []
    assert view["conflicts"] == []
    if not view["websocket_connected"]:
        assert view["reason"]


def test_offline_pinkies_are_not_dispatchable_and_have_no_pinned_address():
    from app.robot_registry import PROFILES, can_dispatch, registry
    rows = registry(
        [{
            "id": "real_pinky1",
            "available": True,
            "odom_fresh": True,
            "motor_ready": True,
            "namespace": "",
            "odom_topic": "/odom",
            "cmd_vel_topic": "/cmd_vel",
        }],
        {},
        "idle",
        "real_pinky1",
    )
    by_id = {row["robot_id"]: row for row in rows}
    assert can_dispatch(by_id["real_pinky1"]) is True
    assert by_id["real_pinky2"]["status"] == "OFFLINE"
    assert by_id["real_pinky3"]["status"] == "OFFLINE"
    assert can_dispatch(by_id["real_pinky2"]) is False
    assert can_dispatch(by_id["real_pinky3"]) is False
    assert all(profile["ssh_host"] is None for profile in PROFILES.values())
    assert PROFILES["real_pinky1"]["odom_topic"] == "/odom"
    assert PROFILES["real_pinky2"]["cmd_vel_topic"] == "/pinky2/cmd_vel"


def test_isolated_mock_fleet_has_three_robots_and_no_motor_topic():
    import importlib.util
    from pathlib import Path
    import yaml
    root = Path(__file__).resolve().parents[1]
    text = (root / "config" / "rmf" / "isolated_mock" / "fleet.yaml").read_text(encoding="utf-8")
    body = yaml.safe_load(text)
    assert set(body["rmf_fleet"]["robots"]) == {"mock_pinky1", "mock_pinky2", "mock_pinky3"}
    assert body["care_move"]["allow_motion"] is False
    assert "17453" not in text and "/cmd_vel" not in text
    spec = importlib.util.spec_from_file_location("isolated_mock_fleet_under_test", root / "scripts" / "isolated_mock_fleet.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    assert module.ROBOTS == ("mock_pinky1", "mock_pinky2", "mock_pinky3")


def test_robot_leases_do_not_authorize_each_other(tmp_path):
    from fleet_adapter.control_lease import LeaseAuthority
    authority = LeaseAuthority("multi-robot-secret", tmp_path / "generation")
    first = authority.issue("rmf-b-domain91", "mock_pinky1", "slot-1", ttl_s=30, now=10)
    second = authority.issue("rmf-b-domain91", "mock_pinky3", "slot-3", ttl_s=30, now=10)
    assert authority.allows(first["lease"], "mock_pinky3", "slot-3", 11, "rmf-b-domain91")[0] is False
    assert authority.allows(second["lease"], "mock_pinky3", "slot-3", 11, "rmf-b-domain91")[0] is True
    assert authority.allows(first["lease"], "mock_pinky1", "slot-1", 11, "rmf-b-domain91")[0] is True
