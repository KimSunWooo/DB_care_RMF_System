import httpx
import pytest
from fastapi.testclient import TestClient

from app import main
from app.rmf_client import RmfClient, RmfClientError, booking_ids_for_label, state_for_label

client = TestClient(main.app)


def _state(booking_id: str, label: str, status: str = "queued") -> dict:
    return {"booking": {"id": booking_id, "labels": [label], "requester": "care-move:abc"},
            "status": status}


def test_label_match_is_unique_or_empty():
    label = "care-move-request=abc"
    assert booking_ids_for_label([_state("task-1", label)], label) == ["task-1"]
    assert state_for_label([_state("task-1", label), _state("task-2", label)], label) is None
    assert state_for_label([{"booking": {"id": "task-1", "labels": []}}], label) is None
    assert booking_ids_for_label({"not": "a list"}, label) == []


def test_timeout_lookup_does_not_post_again():
    calls = []

    def handler(request: httpx.Request) -> httpx.Response:
        calls.append((request.method, request.url.path))
        if request.method == "POST":
            raise httpx.ReadTimeout("slow", request=request)
        label = "care-move-request=abc"
        return httpx.Response(200, json=[_state("task-9", label)])

    rmf = RmfClient("http://rmf.test", token="test-token", timeout=0.2, submit_timeout=0.2,
                    transport=httpx.MockTransport(handler))

    async def run():
        with pytest.raises(RmfClientError) as caught:
            await rmf.post_json("/tasks/robot_task", {"request": {}}, timeout=0.2)
        assert caught.value.kind == "timeout"
        recovered = await rmf.recover_submission("abc")
        return recovered

    import asyncio
    recovered = asyncio.run(run())
    assert recovered["state"]["booking"]["id"] == "task-9"
    assert [method for method, _path in calls].count("POST") == 1
    assert calls[1][0] == "GET"


def test_timeout_without_booking_is_unknown_and_not_retried(monkeypatch):
    posts = {"n": 0}

    async def fake_post(path, payload, timeout=None):
        posts["n"] += 1
        raise RmfClientError("timeout", "RMF API timed out: ReadTimeout")

    async def fake_recover(correlation_id):
        return None

    previous_mode = main.MODE
    previous_status = dict(main.RMF_STATUS)
    monkeypatch.setattr(main, "MODE", "rmf")
    monkeypatch.setattr(main, "RMF_API_URL", "http://rmf.test")
    main.RMF_STATUS.update(status="ready", fleet_registered=True, authenticated=True, reachable=True,
                           robots_on_fleet=["pinky1"], graph_waypoints=["N_GYM"])
    monkeypatch.setattr(main.rmf_client, "post_json", fake_post)
    monkeypatch.setattr(main.rmf_client, "recover_submission", fake_recover)
    before = set(main.TASKS)
    try:
        response = client.post("/api/tasks", json={"robot_id": "pinky1", "places": ["N_GYM"]})
        assert response.status_code == 202
        body = response.json()
        assert body["submission"] == "SUBMISSION_UNKNOWN"
        assert body["accepted"] is False
        assert body["rmf_task_id"] is None
        assert posts["n"] == 1
        created = set(main.TASKS) - before
        assert len(created) == 1
    finally:
        for task_id in set(main.TASKS) - before:
            main.TASKS.pop(task_id, None)
        main.MODE = previous_mode
        main.RMF_STATUS.clear()
        main.RMF_STATUS.update(previous_status)
