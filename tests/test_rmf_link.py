import copy

import httpx
import pytest
from fastapi.testclient import TestClient

from app import main
from app.elevator import build_trip_preview, refuse_hardware_command
from app.rmf_client import RmfClient
from app.rmf_state import apply_fleet_state, mark_stale

client = TestClient(main.app)
OPENAPI = {"paths": {
    "/fleets": {}, "/fleets/{name}/state": {}, "/tasks/dispatch_task": {},
    "/tasks/robot_task": {}, "/tasks/cancel_task": {}, "/tasks/{task_id}/state": {},
    "/tasks": {}, "/socket.io": {}, "/lifts": {}, "/building_map": {},
}}
GRAPH = {"levels": [{"name": "L1", "nav_graphs": [{"vertices": [
    {"name": "N_DOCK1"}, {"name": "N_GYM"}, {"name": "N_EV7"}]}]}]}


@pytest.fixture(autouse=True)
def isolate():
    saved_mode = main.MODE
    saved_url = main.RMF_API_URL
    saved_client = main.rmf_client
    saved_status = copy.deepcopy(main.RMF_STATUS)
    saved_robots = copy.deepcopy(main.ROBOTS)
    saved_tasks = copy.deepcopy(main.TASKS)
    main.MODE = "mock"
    main._next_probe_monotonic = 0.0
    yield
    main.MODE = saved_mode
    main.RMF_API_URL = saved_url
    main.rmf_client = saved_client
    main.RMF_STATUS.clear()
    main.RMF_STATUS.update(saved_status)
    main.ROBOTS.clear()
    main.ROBOTS.update(saved_robots)
    main.TASKS.clear()
    main.TASKS.update(saved_tasks)


def ready_status(**extra):
    payload = {"status": "ready", "fleet_registered": True, "authenticated": True, "reachable": True,
               "robots_on_fleet": ["pinky1", "pinky2", "pinky3"],
               "graph_waypoints": ["N_DOCK1", "N_GYM", "N_EV7"], "detail": "ready"}
    payload.update(extra)
    return payload


def scripted_client(handler) -> RmfClient:
    return RmfClient("http://rmf.test", token="test-token", transport=httpx.MockTransport(handler))


def test_probe_ready_fleet_and_graph():
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/openapi.json":
            return httpx.Response(200, json=OPENAPI)
        if request.url.path == "/fleets":
            return httpx.Response(200, json=[{"name": "care_move", "robots": {
                "pinky1": {"name": "pinky1", "status": "idle"}}}])
        if request.url.path == "/building_map":
            return httpx.Response(200, json=GRAPH)
        raise AssertionError(request.url.path)

    observed = client_run(scripted_client(handler).probe("care_move", ["pinky1", "pinky2"]))
    assert observed["status"] == "ready"
    assert observed["fleet_registered"] is True
    assert observed["missing_robots"] == ["pinky2"]
    assert "N_GYM" in observed["graph_waypoints"]
    assert observed["endpoints"]["cancel_task"] is True
    assert observed["endpoints"]["socket_io"] is True


def test_probe_auth_error_is_not_success():
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/openapi.json":
            return httpx.Response(200, json=OPENAPI)
        return httpx.Response(401, json={"detail": "Not authenticated"})

    observed = client_run(scripted_client(handler).probe("care_move", ["pinky1"]))
    assert observed["reachable"] is True
    assert observed["authenticated"] is False
    assert observed["status"] == "auth_failed"
    assert observed["operation_ready"] is False
    assert observed["data_fresh"] is False
    assert observed["fleet_registered"] is False


def test_probe_timeout_and_unreachable():
    def timeout(_request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectTimeout("timed out")

    def down(_request: httpx.Request) -> httpx.Response:
        raise httpx.ConnectError("connection refused")

    timed = client_run(scripted_client(timeout).probe("care_move", []))
    refused = client_run(scripted_client(down).probe("care_move", []))
    assert timed["status"] == "timeout"
    assert refused["status"] == "unreachable"


def test_probe_blank_url_does_not_guess_a_port():
    observed = client_run(RmfClient("").probe("care_move", ["pinky1"]))
    assert observed["status"] == "not_configured"
    assert observed["api_url"] is None


def test_server_error_is_distinct_from_auth():
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/openapi.json":
            return httpx.Response(200, json=OPENAPI)
        return httpx.Response(503, json={"detail": "unavailable"})

    observed = client_run(scripted_client(handler).probe("care_move", []))
    assert observed["status"] == "server"
    assert observed["status"] != "auth_failed"


def test_rmf_task_blocked_when_fleet_not_ready(monkeypatch):
    main.MODE = "rmf"
    main.RMF_API_URL = "http://rmf.test"
    main.RMF_STATUS.update(status="auth_required", detail="Not authenticated",
                           robots_on_fleet=[], graph_waypoints=None)
    called = {"post": False}

    async def fake_post(path, payload):
        called["post"] = True
        return {}

    monkeypatch.setattr(main, "rmf_post", fake_post)
    response = client.post("/api/tasks", json={"robot_id": "pinky1", "places": ["N_GYM"]})
    assert response.status_code == 409
    assert called["post"] is False
    assert "보내지 않았습니다" in response.json()["detail"]


def test_rmf_task_rejects_unknown_graph_waypoint(monkeypatch):
    main.MODE = "rmf"
    main.RMF_API_URL = "http://rmf.test"
    main.RMF_STATUS.update(**ready_status(graph_waypoints=["N_DOCK1"]))
    called = {"post": False}

    async def fake_post(path, payload):
        called["post"] = True
        return {}

    monkeypatch.setattr(main, "rmf_post", fake_post)
    response = client.post("/api/tasks", json={"robot_id": "pinky1", "places": ["N_GYM"]})
    assert response.status_code == 409
    assert called["post"] is False
    assert "N_GYM" in response.json()["detail"]


def test_rmf_rejection_is_not_recorded_as_success(monkeypatch):
    main.MODE = "rmf"
    main.RMF_API_URL = "http://rmf.test"
    main.RMF_STATUS.update(**ready_status())

    async def fake_post(path, payload):
        raise main.http_from_rmf(main.RmfClientError("rejected", "no fleet adapter bid", 400))

    monkeypatch.setattr(main, "rmf_post", fake_post)
    before = len(main.TASKS)
    response = client.post("/api/tasks", json={"robot_id": "pinky2", "places": ["N_GYM"]})
    assert response.status_code == 409
    assert response.json()["detail"]["kind"] == "rejected"
    assert len(main.TASKS) == before


def test_rmf_timeout_on_submit(monkeypatch):
    main.MODE = "rmf"
    main.RMF_API_URL = "http://rmf.test"
    main.RMF_STATUS.update(**ready_status())

    async def fake_post(path, payload):
        raise main.http_from_rmf(main.RmfClientError("timeout", "timed out"))

    monkeypatch.setattr(main, "rmf_post", fake_post)
    response = client.post("/api/tasks", json={"places": ["N_GYM"]})
    assert response.status_code == 504
    assert response.json()["detail"]["kind"] == "timeout"


def test_robot_state_polling_reconnect_and_added_robot():
    main.ROBOTS["pinky4"] = main.make_robot({
        "id": "pinky4", "label": "Pinky #4", "initial_waypoint": "N_DOCK1", "battery": 80})
    sample = {"name": "care_move", "robots": {
        "pinky1": {"status": "idle", "task_id": "", "battery": 0.5,
                   "location": {"map": "L1", "x": 1.0, "y": 2.0, "yaw": 0.0}, "issues": []},
        "pinky2": {"status": "working", "task_id": "task-2", "battery": 0.2,
                   "location": {"map": "L7", "x": 3.0, "y": 4.0, "yaw": 1.0},
                   "issues": [{"category": "navigation", "detail": "planner failed"}]},
        "pinky3": {"status": "error", "task_id": "task-3", "battery": 0.1, "location": None,
                   "issues": [{"category": "adapter", "detail": "nav2 down"}]},
        "pinky4": {"status": "idle", "task_id": "", "battery": 0.9,
                   "location": {"map": "L1", "x": 8.0, "y": 9.0, "yaw": 0.2}, "issues": []},
    }}
    apply_fleet_state(main.ROBOTS, sample, "2026-10-20T00:00:00+00:00")
    assert main.ROBOTS["pinky1"]["connection"] == "online"
    assert main.ROBOTS["pinky1"]["position_fresh"] is True
    assert main.ROBOTS["pinky1"]["battery"] == 50.0
    assert main.ROBOTS["pinky2"]["task_id"] == "task-2"
    assert "planner failed" in main.ROBOTS["pinky2"]["error"]
    assert main.ROBOTS["pinky3"]["position_fresh"] is False
    assert main.ROBOTS["pinky4"]["pose"]["x"] == 8.0
    mark_stale(main.ROBOTS)
    assert main.ROBOTS["pinky1"]["connection"] == "stale"
    assert main.ROBOTS["pinky1"]["pose"] is None
    assert main.ROBOTS["pinky1"]["position_fresh"] is False
    apply_fleet_state(main.ROBOTS, sample, "2026-10-20T00:00:02+00:00")
    assert main.ROBOTS["pinky1"]["connection"] == "online"
    assert main.ROBOTS["pinky1"]["pose"]["map"] == "L1"


def test_unseen_robot_stays_unverified_when_poll_fails():
    mark_stale(main.ROBOTS)
    assert main.ROBOTS["pinky1"]["connection"] == "unverified"
    assert main.ROBOTS["pinky1"]["pose"] is None
    assert main.ROBOTS["pinky1"]["position_fresh"] is False


def test_status_route_reports_auth_without_changing_mock_robots():
    before = copy.deepcopy(main.ROBOTS["pinky1"])

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/openapi.json":
            return httpx.Response(200, json=OPENAPI)
        return httpx.Response(401, json={"detail": "Not authenticated"})

    main.MODE = "mock"
    main.rmf_client = scripted_client(handler)
    response = client.get("/api/rmf/status")
    assert response.status_code == 200
    body = response.json()
    assert body["status"] == "auth_failed"
    assert body["operation_ready"] is False
    assert body["commands_enabled"] is False
    assert main.ROBOTS["pinky1"]["waypoint"] == before["waypoint"]
    assert main.ROBOTS["pinky1"]["status"] == before["status"]


def test_bearer_prefix_is_not_duplicated():
    seen = {}

    def handler(request: httpx.Request) -> httpx.Response:
        seen["authorization"] = request.headers.get("authorization")
        if request.url.path == "/openapi.json":
            return httpx.Response(200, json=OPENAPI)
        return httpx.Response(401, json={"detail": "Not authenticated"})

    client_run(RmfClient("http://rmf.test", token="Bearer Bearer abc", transport=httpx.MockTransport(handler)).probe("care_move", []))
    assert seen["authorization"] == "Bearer abc"


def test_missing_token_does_not_call_fleets():
    called = []

    def handler(request: httpx.Request) -> httpx.Response:
        called.append(request.url.path)
        assert request.url.path == "/openapi.json"
        return httpx.Response(200, json=OPENAPI)

    observed = client_run(RmfClient("http://rmf.test", token="", transport=httpx.MockTransport(handler)).probe("care_move", ["pinky1"]))
    assert observed["status"] == "token_missing"
    assert observed["api_connected"] is True
    assert observed["authenticated"] is False
    assert "/fleets" not in called


def test_empty_fleet_is_not_an_auth_error_and_map_404_is_missing_data():
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/openapi.json":
            return httpx.Response(200, json=OPENAPI)
        if request.url.path == "/fleets":
            return httpx.Response(200, json=[])
        if request.url.path == "/building_map":
            return httpx.Response(404, json={"detail": "Not Found"})
        raise AssertionError(request.url.path)

    observed = client_run(scripted_client(handler).probe("care_move", ["pinky1"]))
    assert observed["authenticated"] is True
    assert observed["status"] == "fleet_empty"
    assert observed["fleet_data"] == "empty"
    assert observed["robot_data"] == "none"
    assert observed["map_data"] == "missing"
    assert observed["map_http_status"] == 404
    assert observed["operation_ready"] is False
    assert observed["data_fresh"] is True


def test_fleet_without_robots_waits_for_robot_data():
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/openapi.json":
            return httpx.Response(200, json=OPENAPI)
        if request.url.path == "/fleets":
            return httpx.Response(200, json=[{"name": "care_move", "robots": {}}])
        if request.url.path == "/building_map":
            return httpx.Response(200, json=GRAPH)
        raise AssertionError(request.url.path)

    observed = client_run(scripted_client(handler).probe("care_move", ["pinky1"]))
    assert observed["status"] == "robots_waiting"
    assert observed["fleet_data"] == "present"
    assert observed["robot_data"] == "none"
    assert observed["operation_ready"] is False


def test_forbidden_is_distinct_from_unauthorized():
    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/openapi.json":
            return httpx.Response(200, json=OPENAPI)
        return httpx.Response(403, json={"detail": "forbidden"})

    observed = client_run(scripted_client(handler).probe("care_move", []))
    assert observed["status"] == "forbidden"
    assert observed["auth_state"] == "forbidden"


def test_auth_failure_is_not_retried_immediately():
    calls = {"fleets": 0}

    def handler(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/openapi.json":
            return httpx.Response(200, json=OPENAPI)
        if request.url.path == "/fleets":
            calls["fleets"] += 1
            return httpx.Response(401, json={"detail": "Not authenticated"})
        raise AssertionError(request.url.path)

    main._next_probe_monotonic = 0.0
    main.rmf_client = scripted_client(handler)
    first = client.get("/api/rmf/status")
    second = client.get("/api/rmf/status")
    assert first.json()["status"] == "auth_failed"
    assert second.json()["cached"] is True
    assert calls["fleets"] == 1
    assert "eyJ" not in first.text


def test_dashboard_state_contains_connection_fields():
    response = client.get("/api/state")
    body = response.json()
    assert response.status_code == 200
    assert len(body["robots"]) == 3
    assert "rmf" in body
    assert body["rmf"]["status"]
    page = client.get("/")
    assert page.status_code == 200
    assert 'id="rmfBanner"' in page.text
    assert "실제 LiftRequest는 전송하지 않습니다" in page.text


def test_mock_cancel_and_rmf_cancel_request(monkeypatch):
    main.MODE = "mock"
    for robot in main.ROBOTS.values():
        robot["status"] = "idle"
    created = client.post("/api/tasks", json={"robot_id": "pinky3", "places": ["N_GYM"]})
    task_id = created.json()["id"]
    cancelled = client.post(f"/api/tasks/{task_id}/cancel")
    assert cancelled.status_code == 200
    assert cancelled.json()["status"] == "cancelled"
    main.MODE = "rmf"
    main.TASKS["local-1"] = {"id": "local-1", "source": "RMF", "rmf_task_id": "rmf-9",
                             "status": "rmf_accepted_submission", "completed": False, "places": ["N_GYM"],
                             "robot_id": "pinky1"}
    called = {}

    async def fake_cancel(rmf_task_id):
        called["id"] = rmf_task_id
        return {"success": True}

    monkeypatch.setattr(main.rmf_client, "cancel_task", fake_cancel)
    response = client.post("/api/tasks/local-1/cancel")
    assert response.status_code == 200
    assert called["id"] == "rmf-9"
    assert response.json()["status"] == "cancel_requested"
    assert response.json()["completed"] is False


def test_mock_task_and_elevator_still_work():
    main.MODE = "mock"
    for robot in main.ROBOTS.values():
        robot["status"] = "idle"
    created = client.post("/api/tasks", json={"robot_id": "pinky1", "places": ["N_ROOM102"]})
    assert created.status_code == 202
    assert created.json()["source"] == "MOCK"
    travel = client.post("/api/elevator/mock-travel", json={"robot_id": "pinky2", "target_floor": "7F"})
    assert travel.status_code == 202
    assert travel.json()["hardware_commands"] is False


def test_elevator_contract_does_not_transmit():
    preview = build_trip_preview(robot_id="pinky1", source_floor="1F", target_floor="7F",
                                 session_id="preview")
    assert preview["transmit"] is False
    assert preview["qr_is_completion"] is False
    with pytest.raises(RuntimeError):
        refuse_hardware_command()
    response = client.post("/api/elevator/hardware")
    assert response.status_code == 403
    described = client.get("/api/elevator/interface")
    assert described.json()["commands_enabled"] is False
    assert "DOOR_OPEN" in described.json()["phases"]


def client_run(coro):
    import asyncio
    return asyncio.run(coro)
