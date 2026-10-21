"""Care-Move admin dashboard learning boilerplate.

mock: in-memory simulated robot/elevator state only.
rmf: POST task requests to an already running RMF-Web API server after a
     fleet and navigation-graph check. Neither mode drives hardware.
"""
from __future__ import annotations

import asyncio
import contextlib
import json
import os
import time
import uuid
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

from app.elevator import ElevatorPhase, build_trip_preview, refuse_hardware_command
from app.rmf_client import RmfClient, RmfClientError
from app.rmf_state import apply_fleet_state, mark_stale, utc_now
from app.rmf_token import build_token_provider

ROOT = Path(__file__).resolve().parent.parent
CONFIG = ROOT / "config"


def load_runtime_env(root: Path) -> None:
    """Load gitignored .env once at process start. Does not read .env.example."""
    path = root / ".env"
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        key = key.strip()
        value = value.strip().strip("'").strip('"')
        if key and key not in os.environ:
            os.environ[key] = value


load_runtime_env(ROOT)
MODE = os.getenv("CARE_MOVE_MODE", "mock").lower()
if MODE not in {"mock", "rmf"}:
    raise RuntimeError("CARE_MOVE_MODE must be 'mock' or 'rmf'")
RMF_API_URL = os.getenv("RMF_API_URL", "").rstrip("/")
RMF_API_TOKEN = os.getenv("RMF_API_TOKEN", "")
RMF_API_TIMEOUT = float(os.getenv("RMF_API_TIMEOUT", "5"))
RMF_POLL_INTERVAL_SEC = float(os.getenv("RMF_POLL_INTERVAL_SEC", "2"))
RMF_AUTH_RETRY_SEC = float(os.getenv("RMF_AUTH_RETRY_SEC", "30"))
_next_probe_monotonic = 0.0
_probe_lock = asyncio.Lock()

robot_config = json.loads((CONFIG / "robots.json").read_text(encoding="utf-8"))
waypoint_config = json.loads((CONFIG / "waypoints.json").read_text(encoding="utf-8"))
FLEET = os.getenv("RMF_FLEET_NAME", robot_config["fleet_name"])
WAYPOINTS = {w["name"]: w for w in waypoint_config["waypoints"]}
TASKS: dict[str, dict] = {}
RUNNING: dict[str, asyncio.Task] = {}
ELEVATOR = {"id": "LIFT_A", "floor": "1F", "target_floor": None, "door": "closed",
            "state": "idle", "reserved_by": None, "last_error": None,
            "hardware_commands": False}
elevator_mutex = asyncio.Lock()
token_provider = build_token_provider()
rmf_client = RmfClient(RMF_API_URL, RMF_API_TOKEN, RMF_API_TIMEOUT, provider=token_provider)


def make_robot(raw: dict) -> dict:
    waypoint = raw["initial_waypoint"]
    floor = WAYPOINTS[waypoint]["floor"]
    robot = {
        "id": raw["id"], "label": raw["label"], "fleet": FLEET, "battery": raw["battery"],
        "task_id": None, "task_status": None, "error": None, "last_state_at": None,
        "configured_waypoint": waypoint, "configured_floor": floor, "pose": None,
    }
    if MODE == "mock":
        robot.update(floor=floor, waypoint=waypoint, status="idle", connection="mock",
                     position_fresh=True, position_source="mock")
    else:
        robot.update(floor=None, waypoint=None, status="unverified", connection="unverified",
                     position_fresh=False, position_source="unverified")
    return robot


ROBOTS = {raw["id"]: make_robot(raw) for raw in robot_config["robots"]}
RMF_STATUS: dict = {
    "status": "not_checked",
    "detail": "RMF 연결을 아직 확인하지 않았습니다.",
    "reachable": False,
    "authenticated": False,
    "api_url": RMF_API_URL or None,
    "fleet_name": FLEET,
    "fleet_registered": False,
    "robots_on_fleet": [],
    "missing_robots": list(ROBOTS),
    "graph_waypoints": None,
    "graph_levels": [],
    "endpoints": {},
    "checked_at": None,
    "http_status": None,
    "last_error_kind": None,
    "state_transport": "http_poll",
}


class TaskRequest(BaseModel):
    robot_id: str | None = None  # None means fleet-level automatic dispatch
    places: list[str] = Field(min_length=1, max_length=8)
    rounds: int = Field(default=1, ge=1, le=3)


class ElevatorRequest(BaseModel):
    robot_id: str
    target_floor: Literal["1F", "7F"]


def check_waypoints(places: list[str]):
    bad = [place for place in places if place not in WAYPOINTS]
    if bad:
        raise HTTPException(400, f"Unknown waypoint(s): {', '.join(bad)}")


def public_rmf_status() -> dict:
    published = {key: value for key, value in RMF_STATUS.items() if key != "fleet_state"}
    published["commands_enabled"] = MODE == "rmf" and RMF_STATUS.get("operation_ready") is True
    published["motion_verified"] = False
    published["cached"] = False
    published["token"] = token_provider.status()
    return published


def snapshot() -> dict:
    return {"mode": MODE, "fleet": FLEET, "robots": list(ROBOTS.values()),
            "tasks": list(reversed(list(TASKS.values())))[0:25],
            "elevator": dict(ELEVATOR), "waypoints": list(WAYPOINTS.values()),
            "map_note": waypoint_config["note"],
            "robot_state_source": "MOCK" if MODE == "mock" else "RMF HTTP poll or UNVERIFIED",
            "rmf": public_rmf_status()}


def dispatch_block_reason(robot_id: str | None, places: list[str]) -> str | None:
    if not RMF_API_URL:
        return "RMF_API_URL이 설정되지 않았습니다. 작업 요청을 보내지 않았습니다."
    status = RMF_STATUS.get("status")
    if status != "ready":
        detail = RMF_STATUS.get("detail") or ""
        return f"RMF 작업 요청을 보내지 않았습니다. 연결 상태: {status}. {detail}"
    robots = set(RMF_STATUS.get("robots_on_fleet") or [])
    if robot_id:
        if robot_id not in robots:
            listed = ", ".join(sorted(robots)) or "없음"
            return f"로봇 '{robot_id}'이 Fleet '{FLEET}'에 없습니다. 등록된 로봇: {listed}"
    elif not robots:
        return f"Fleet '{FLEET}'에 배차 가능한 로봇이 없습니다. 작업 요청을 보내지 않았습니다."
    graph = RMF_STATUS.get("graph_waypoints")
    if not isinstance(graph, list):
        return "RMF navigation graph를 확인하지 못했습니다. 작업 요청을 보내지 않았습니다."
    missing = [place for place in places if place not in set(graph)]
    if missing:
        return "RMF navigation graph에 없는 waypoint: " + ", ".join(missing)
    return None


def http_from_rmf(ex: RmfClientError) -> HTTPException:
    code = {"timeout": 504, "unreachable": 503, "auth": 401, "forbidden": 403,
            "server": 502, "rejected": 409, "not_configured": 409, "not_found": 404,
            "client": 502, "invalid_response": 502, "token_unavailable": 401}.get(ex.kind, 502)
    return HTTPException(code, {"kind": ex.kind, "detail": ex.detail, "rmf_status": ex.status_code})


def _schedule_next_probe(status: str) -> None:
    global _next_probe_monotonic
    delay = RMF_AUTH_RETRY_SEC if status in {"auth_failed", "forbidden", "token_missing"} else RMF_POLL_INTERVAL_SEC
    _next_probe_monotonic = time.monotonic() + delay


async def refresh_rmf_observation(apply_robots: bool, force: bool = False) -> dict:
    global _next_probe_monotonic
    if not force and RMF_STATUS.get("checked_at") and time.monotonic() < _next_probe_monotonic:
        cached = public_rmf_status()
        cached["cached"] = True
        return cached
    async with _probe_lock:
        if not force and RMF_STATUS.get("checked_at") and time.monotonic() < _next_probe_monotonic:
            cached = public_rmf_status()
            cached["cached"] = True
            return cached
        previous_success_at = RMF_STATUS.get("last_success_at")
        previous_summary = RMF_STATUS.get("last_success_summary")
        observed = await rmf_client.probe(FLEET, list(ROBOTS))
        fleet_state = observed.pop("fleet_state", None)
        received_at = utc_now()
        observed["checked_at"] = received_at
        if observed.get("data_fresh"):
            observed["last_success_at"] = received_at
            observed["last_success_summary"] = {
                "status": observed.get("status"),
                "fleet_count": observed.get("fleet_count"),
                "robot_count": observed.get("robot_count"),
                "map_data": observed.get("map_data"),
                "at": received_at,
            }
        else:
            observed["last_success_at"] = previous_success_at
            observed["last_success_summary"] = previous_summary
        RMF_STATUS.clear()
        RMF_STATUS.update(observed)
        _schedule_next_probe(str(observed.get("status") or ""))
        if not apply_robots:
            return public_rmf_status()
        if observed.get("data_fresh") and isinstance(fleet_state, dict):
            apply_fleet_state(ROBOTS, fleet_state, received_at)
        elif not observed.get("data_fresh"):
            mark_stale(ROBOTS)
        if observed.get("authenticated"):
            await refresh_open_tasks()
        return public_rmf_status()


async def refresh_open_tasks() -> None:
    for task in list(TASKS.values()):
        rmf_task_id = task.get("rmf_task_id")
        if task.get("source") != "RMF" or not rmf_task_id:
            continue
        if task.get("rmf_status") in {"completed", "canceled", "killed", "failed"}:
            continue
        try:
            state = await rmf_client.get_json(f"/tasks/{rmf_task_id}/state")
        except RmfClientError as ex:
            task["error"] = ex.detail
            task["last_error_kind"] = ex.kind
            continue
        if not isinstance(state, dict):
            task["error"] = "RMF task state 응답 형식이 올바르지 않습니다."
            continue
        task["rmf_status"] = state.get("status")
        task["assigned_to"] = state.get("assigned_to")
        task["error"] = None
        if state.get("status"):
            task["status"] = state["status"]
        task["completed"] = state.get("status") == "completed"
        detail = state.get("detail")
        if state.get("status") in {"failed", "error", "canceled", "killed"} and detail:
            task["error"] = str(detail)[:500]


async def rmf_poll_loop() -> None:
    while True:
        try:
            await refresh_rmf_observation(apply_robots=True)
        except asyncio.CancelledError:
            raise
        except Exception as ex:
            RMF_STATUS.update(status="server_error", detail=str(ex), last_error_kind="server",
                              checked_at=utc_now(), data_fresh=False, operation_ready=False,
                              authenticated=False, robots_on_fleet=[], robot_count=None)
            mark_stale(ROBOTS)
            _schedule_next_probe("server")
        await asyncio.sleep(max(0.2, _next_probe_monotonic - time.monotonic()))


async def simulate_elevator(robot_id: str, target_floor: str):
    """Serialized animation only; NEVER connected to any elevator controller."""
    async with elevator_mutex:
        ELEVATOR.update(reserved_by=robot_id, target_floor=target_floor, state="reserved", door="closed")
        await asyncio.sleep(0.5)
        ELEVATOR.update(state="moving", door="closed")
        await asyncio.sleep(1.0)
        ELEVATOR.update(floor=target_floor, state="arrived", door="open")
        await asyncio.sleep(0.7)
        ELEVATOR.update(state="idle", target_floor=None, door="closed", reserved_by=None)


async def simulate_task(task_id: str):
    task = TASKS[task_id]
    robot = ROBOTS[task["robot_id"]]
    try:
        task["status"] = "executing"
        for _ in range(task["rounds"]):
            for place in task["places"]:
                dest = WAYPOINTS[place]
                if dest["floor"] != robot["floor"]:
                    robot["status"] = "waiting_elevator"
                    await simulate_elevator(robot["id"], dest["floor"])
                    robot["floor"] = dest["floor"]
                robot["status"] = "moving"
                await asyncio.sleep(0.9)
                robot["waypoint"] = place
                robot["floor"] = dest["floor"]
                robot["battery"] = max(0, robot["battery"] - 1)
        task["status"] = "completed"
    except asyncio.CancelledError:
        task["status"] = "cancelled"
        raise
    except Exception as ex:
        task["status"] = "failed"
        task["error"] = str(ex)
    finally:
        robot["status"] = "idle"
        robot["task_id"] = None
        RUNNING.pop(task_id, None)


async def rmf_post(path: str, payload: dict) -> dict:
    try:
        return await rmf_client.post_json(path, payload)
    except RmfClientError as ex:
        raise http_from_rmf(ex) from ex


@asynccontextmanager
async def lifespan(_: FastAPI):
    await token_provider.start()
    poller = asyncio.create_task(rmf_poll_loop()) if MODE == "rmf" else None
    try:
        yield
    finally:
        if poller:
            poller.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await poller
        await token_provider.close()


app = FastAPI(title="Care-Move learning FMS", version="0.2.0", lifespan=lifespan)


@app.get("/")
async def index():
    return FileResponse(ROOT / "app" / "static" / "index.html")


@app.get("/api/health")
async def health():
    return {"ok": True, "mode": MODE, "rmf_api_configured": bool(RMF_API_URL),
            "rmf_status": RMF_STATUS.get("status"), "rmf_reachable": RMF_STATUS.get("reachable"),
            "notes": "ok means this Care-Move process is up, not that RMF or robots are ready"}


@app.get("/api/state")
async def get_state():
    return snapshot()


@app.get("/api/rmf/status")
async def rmf_status():
    try:
        return await refresh_rmf_observation(apply_robots=MODE == "rmf")
    except RmfClientError as ex:
        raise http_from_rmf(ex) from ex


@app.get("/api/elevator/interface")
async def elevator_interface():
    return {
        "commands_enabled": False,
        "phases": [phase.value for phase in ElevatorPhase],
        "paths": ["1F->7F", "7F->1F"],
        "rmf_messages": ["rmf_lift_msgs/LiftRequest", "rmf_lift_msgs/LiftState"],
        "gateway": "HTTP REST preview only; WebSocket status is not connected",
        "preview_example": build_trip_preview(robot_id="pinky1", source_floor="1F",
                                              target_floor="7F", session_id="not-sent"),
    }


def build_rmf_payload(req: TaskRequest) -> tuple[str, dict]:
    """Payload per the running server's RobotTaskRequest / DispatchTaskRequest schemas."""
    request = {"category": "patrol", "description": {"places": req.places, "rounds": req.rounds},
               "requester": "care-move-dashboard", "fleet_name": FLEET}
    if req.robot_id:
        return "/tasks/robot_task", {
            "type": "robot_task_request", "robot": req.robot_id,
            "fleet": FLEET, "request": request
        }
    return "/tasks/dispatch_task", {"type": "dispatch_task_request", "request": request}


@app.post("/api/tasks/preview")
async def task_preview(req: TaskRequest):
    check_waypoints(req.places)
    if req.robot_id and req.robot_id not in ROBOTS:
        raise HTTPException(404, "Unknown robot")
    path, payload = build_rmf_payload(req)
    return {"method": "POST", "url": f"{RMF_API_URL}{path}",
            "body": payload, "dispatch_allowed": dispatch_block_reason(req.robot_id, req.places) is None,
            "block_reason": dispatch_block_reason(req.robot_id, req.places),
            "note": "PREVIEW ONLY: this endpoint never sends the command"}


def record_submission(req: TaskRequest, result: dict) -> dict:
    state = result.get("state") if isinstance(result, dict) else None
    if not isinstance(state, dict) or not isinstance(state.get("booking"), dict) or not state["booking"].get("id"):
        raise HTTPException(502, {"kind": "invalid_response",
                                   "detail": "RMF 응답에 task booking id가 없습니다. 접수 성공으로 기록하지 않았습니다."})
    rmf_task_id = state["booking"]["id"]
    local_id = "rmf-request-" + uuid.uuid4().hex[:8]
    task = {"id": local_id, "rmf_task_id": rmf_task_id, "robot_id": req.robot_id or "AUTO",
            "places": req.places, "rounds": req.rounds, "status": "rmf_accepted_submission",
            "rmf_status": state.get("status"), "source": "RMF", "accepted": True,
            "completed": False, "error": None,
            "rmf_response": {"success": True, "booking_id": rmf_task_id, "rmf_status": state.get("status")}}
    TASKS[local_id] = task
    return task


@app.post("/api/tasks", status_code=202)
async def create_task(req: TaskRequest):
    check_waypoints(req.places)
    if req.robot_id is not None and req.robot_id not in ROBOTS:
        raise HTTPException(404, "Unknown robot")
    if MODE == "mock":
        available = [r for r in ROBOTS.values() if r["status"] == "idle"]
        if req.robot_id:
            robot = ROBOTS[req.robot_id]
            if robot["status"] != "idle":
                raise HTTPException(409, "Robot busy")
        elif available:
            robot = available[0]  # MOCK ONLY: not a real RMF auction
        else:
            raise HTTPException(409, "No idle robot")
        task_id = "mock-" + uuid.uuid4().hex[:10]
        task = {"id": task_id, "robot_id": robot["id"], "places": req.places,
                "rounds": req.rounds, "status": "queued", "source": "MOCK", "error": None}
        TASKS[task_id] = task
        robot["status"] = "assigned"
        robot["task_id"] = task_id
        RUNNING[task_id] = asyncio.create_task(simulate_task(task_id))
        return task
    if RMF_STATUS.get("status") == "not_checked":
        await refresh_rmf_observation(apply_robots=True)
    reason = dispatch_block_reason(req.robot_id, req.places)
    if reason:
        raise HTTPException(409, reason)
    path, payload = build_rmf_payload(req)
    result = await rmf_post(path, payload)
    return record_submission(req, result)


@app.post("/api/tasks/{task_id}/cancel")
async def cancel_task(task_id: str):
    task = TASKS.get(task_id)
    if task is None:
        task = next((item for item in TASKS.values()
                     if item.get("rmf_task_id") == task_id), None)
    if task is None:
        raise HTTPException(404, "Unknown task")
    if task.get("source") == "MOCK":
        running = RUNNING.get(task["id"])
        if running:
            running.cancel()
        task["status"] = "cancelled"
        return task
    rmf_task_id = task.get("rmf_task_id")
    if not rmf_task_id:
        raise HTTPException(409, "RMF task id가 없어 취소 요청을 보내지 않았습니다.")
    try:
        result = await rmf_client.cancel_task(rmf_task_id)
    except RmfClientError as ex:
        raise http_from_rmf(ex) from ex
    task["status"] = "cancel_requested"
    task["completed"] = False
    task["cancel_response"] = {"success": bool(isinstance(result, dict) and result.get("success") is True)}
    return task


@app.post("/api/elevator/mock-travel", status_code=202)
async def elevator_mock_travel(req: ElevatorRequest):
    if MODE != "mock":
        raise HTTPException(403, "Direct elevator test is disabled in RMF mode; use RMF lift adapter")
    if req.robot_id not in ROBOTS:
        raise HTTPException(404, "Unknown robot")
    if elevator_mutex.locked():
        raise HTTPException(409, "Elevator busy")
    asyncio.create_task(simulate_elevator(req.robot_id, req.target_floor))
    return {"accepted": True, "source": "MOCK_ONLY", "robot_id": req.robot_id,
            "target_floor": req.target_floor, "hardware_commands": False}


@app.post("/api/elevator/hardware")
async def elevator_hardware_disabled():
    try:
        refuse_hardware_command()
    except RuntimeError as ex:
        raise HTTPException(403, str(ex)) from ex


@app.websocket("/ws/state")
async def ws_state(socket: WebSocket):
    await socket.accept()
    try:
        while True:
            await socket.send_json(snapshot())
            await asyncio.sleep(0.6)
    except (WebSocketDisconnect, RuntimeError):
        pass
