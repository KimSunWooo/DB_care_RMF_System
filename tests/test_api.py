import time
from fastapi.testclient import TestClient
from app.main import app

client = TestClient(app)

def test_default_three_robots():
    r = client.get('/api/state')
    assert r.status_code == 200
    assert len(r.json()['robots']) == 3


def test_waypoint_validation():
    r = client.post('/api/tasks', json={'robot_id': 'pinky1', 'places': ['NOT_EXIST']})
    assert r.status_code == 400


def test_unknown_robot():
    r = client.post('/api/tasks', json={'robot_id': 'pinky999', 'places': ['N_GYM']})
    assert r.status_code == 404


def test_mock_task_accepts_and_finishes():
    r = client.post('/api/tasks', json={'robot_id': 'pinky1', 'places': ['N_ROOM102']})
    assert r.status_code == 202
    assert r.json()['source'] == 'MOCK'


def test_elevator_unknown_robot():
    r = client.post('/api/elevator/mock-travel', json={'robot_id':'bad', 'target_floor':'7F'})
    assert r.status_code == 404


def test_rmf_preview_direct_robot():
    r = client.post('/api/tasks/preview', json={'robot_id': 'pinky3', 'places': ['N_EV7'], 'rounds': 2})
    assert r.status_code == 200
    assert r.json()['url'].endswith('/tasks/robot_task')
    assert r.json()['body']['type'] == 'robot_task_request'
    assert r.json()['body']['robot'] == 'pinky3'
    assert r.json()['body']['request']['description']['places'] == ['N_EV7']


def test_rmf_preview_fleet_dispatch():
    r = client.post('/api/tasks/preview', json={'places': ['N_ROOM102']})
    assert r.status_code == 200
    assert r.json()['url'].endswith('/tasks/dispatch_task')
    assert r.json()['body']['type'] == 'dispatch_task_request'


def test_rmf_mode_dispatch_uses_api_without_claiming_completion(monkeypatch):
    from app import main
    collected = {}
    async def fake_post(path, payload):
        collected['path'] = path
        collected['payload'] = payload
        return {'success': True, 'state': {'booking': {'id': 'rmf-123'}}}
    monkeypatch.setattr(main, 'MODE', 'rmf')
    monkeypatch.setattr(main, 'RMF_API_URL', 'http://rmf.test')
    previous = dict(main.RMF_STATUS)
    main.RMF_STATUS.update(status='ready', fleet_registered=True, authenticated=True, reachable=True,
                           robots_on_fleet=['pinky1', 'pinky2', 'pinky3'], graph_waypoints=['N_EV7'])
    monkeypatch.setattr(main, 'rmf_post', fake_post)
    try:
        r = client.post('/api/tasks', json={'robot_id': 'pinky3', 'places': ['N_EV7']})
        assert r.status_code == 202
        assert collected['path'] == '/tasks/robot_task'
        assert collected['payload']['robot'] == 'pinky3'
        assert r.json()['status'] == 'rmf_accepted_submission'
        assert r.json()['completed'] is False
        assert r.json()['rmf_task_id'] == 'rmf-123'
    finally:
        main.RMF_STATUS.clear()
        main.RMF_STATUS.update(previous)
        main.MODE = 'mock'


def test_rmf_mode_disables_direct_elevator(monkeypatch):
    from app import main
    monkeypatch.setattr(main, 'MODE', 'rmf')
    r = client.post('/api/elevator/mock-travel', json={'robot_id': 'pinky1', 'target_floor': '7F'})
    assert r.status_code == 403
