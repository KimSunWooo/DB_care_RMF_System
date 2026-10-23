"""Care-Move admin dashboard learning boilerplate.

mock: in-memory simulated robot/elevator state only.
rmf: POST task requests to an already running RMF-Web API server after a
     fleet and navigation-graph check. Neither mode drives hardware.
"""
from __future__ import annotations

import asyncio
import contextlib
import hmac
import json
import os
import signal
import socket
import subprocess
import time
import uuid
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Literal

from fastapi import FastAPI, HTTPException, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel, Field

from app.elevator import ElevatorPhase, build_trip_preview, refuse_hardware_command
from app.map_registry import load_registry
from app.rmf_client import RmfClient, RmfClientError, log_task_event
from app.robot_registry import registry as robot_registry
from app.schedule_view import schedule_view
from app.task_store import TaskStore
from app.transport_requests import accept_transport
from app.rmf_state import apply_fleet_state, mark_stale, utc_now
from app.rmf_token import build_token_provider
from app.reconcile_worker import ReconcileService
from app.rmf_environment import fence_physical
from app.task_guard import conflicting_bookings
from app.task_reconcile import judge
from fleet_adapter.physical_motion import (
    DRIVE_DOMAIN_ID, FORWARD_TEST, PHYSICAL_ROBOT_IDS, ROTATE_TEST, TARGET_YAW_RAD, button_block,
    gateway_protocol_block,
)

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
RMF_TASK_SUBMIT_TIMEOUT = float(os.getenv("RMF_TASK_SUBMIT_TIMEOUT", "12"))
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
rmf_client = RmfClient(RMF_API_URL, RMF_API_TOKEN, RMF_API_TIMEOUT, provider=token_provider,
                      submit_timeout=RMF_TASK_SUBMIT_TIMEOUT)
task_store = TaskStore(Path(os.getenv("CARE_MOVE_QUEUE_PATH", str(ROOT / "data" / "task_queue.sqlite"))))
reconciler = ReconcileService(Path(os.getenv(
    "CARE_MOVE_RECONCILE_PATH", str(ROOT / "data" / "reconciliation.sqlite"))))
_WATCHERS: set[asyncio.Task] = set()


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


class PhysicalSelectRequest(BaseModel):
    robot_id: str


class PhysicalTestRequest(BaseModel):
    kind: Literal["forward_5cm", "rotate_180"] = "forward_5cm"
    robot_id: str | None = None


class ElevatorRequest(BaseModel):
    robot_id: str
    target_floor: Literal["1F", "7F"]


def check_waypoints(places: list[str]):
    graph = set(RMF_STATUS.get("graph_waypoints") or []) if MODE == "rmf" else set()
    bad = [place for place in places if place not in WAYPOINTS and place not in graph]
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


class SubmissionUnknown(Exception):
    """The POST timed out and one lookup did not find a unique booking. Do not POST again."""

    def __init__(self, correlation_id: str, detail: str):
        super().__init__(detail)
        self.correlation_id = correlation_id
        self.detail = detail


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
        if task.get("submission") == "SUBMISSION_UNKNOWN" and not rmf_task_id:
            correlation_id = task.get("correlation_id")
            if not correlation_id:
                continue
            try:
                recovered = await rmf_client.recover_submission(str(correlation_id))
            except RmfClientError as ex:
                task["error"] = ex.detail
                task["last_error_kind"] = ex.kind
                continue
            if recovered is None:
                continue
            state = recovered.get("state") if isinstance(recovered, dict) else None
            booking = state.get("booking") if isinstance(state, dict) else None
            if not isinstance(booking, dict) or not booking.get("id"):
                continue
            task["rmf_task_id"] = booking["id"]
            task["accepted"] = True
            task["submission"] = "SUBMISSION_ACCEPTED"
            task["error"] = None
            rmf_task_id = booking["id"]
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
    request = payload.get("request") if isinstance(payload, dict) else None
    correlation_id = ""
    if isinstance(request, dict) and isinstance(request.get("requester"), str):
        requester = request["requester"]
        if requester.startswith("care-move:"):
            correlation_id = requester.split(":", 1)[1]
    started = time.monotonic()
    log_task_event("RMF_HTTP_POST_STARTED", correlation_id=correlation_id or None, path=path)
    try:
        body = await rmf_client.post_json(path, payload, timeout=rmf_client.submit_timeout)
    except RmfClientError as ex:
        elapsed_ms = round((time.monotonic() - started) * 1000)
        if ex.kind != "timeout":
            log_task_event("RMF_HTTP_RESPONSE_RECEIVED", correlation_id=correlation_id or None,
                           http_elapsed_ms=elapsed_ms, error_kind=ex.kind)
            raise http_from_rmf(ex) from ex
        log_task_event("RMF_HTTP_TIMEOUT", correlation_id=correlation_id or None,
                       http_elapsed_ms=elapsed_ms, error_kind="timeout",
                       task_status="SUBMISSION_UNKNOWN")
        if not correlation_id:
            raise SubmissionUnknown("", "RMF API timed out and this request has no correlation id") from ex
        recovered = await rmf_client.recover_submission(correlation_id)
        if recovered is None:
            raise SubmissionUnknown(
                correlation_id,
                "RMF API timed out. 같은 요청은 다시 보내지 않았고, 조회에서도 booking을 확인하지 못했습니다.",
            ) from ex
        return recovered
    elapsed_ms = round((time.monotonic() - started) * 1000)
    state = body.get("state") if isinstance(body, dict) else None
    booking = state.get("booking") if isinstance(state, dict) else None
    log_task_event("RMF_HTTP_RESPONSE_RECEIVED", correlation_id=correlation_id or None,
                   http_elapsed_ms=elapsed_ms,
                   rmf_booking_id=booking.get("id") if isinstance(booking, dict) else None,
                   task_status=state.get("status") if isinstance(state, dict) else None)
    return body


@asynccontextmanager
async def lifespan(_: FastAPI):
    await token_provider.start()
    poller = asyncio.create_task(rmf_poll_loop()) if MODE == "rmf" else None
    reconcile = asyncio.create_task(reconcile_loop()) if os.getenv("CARE_MOVE_RECONCILE_WORKER") == "1" else None
    try:
        yield
    finally:
        for job in (poller, reconcile):
            if job:
                job.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await job
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


def read_physical_gateway() -> dict:
    """One read-only poll. A down gateway is not a pose and is not a motion command."""
    try:
        with socket.create_connection(("127.0.0.1", 17453), timeout=0.4) as sock:
            sock.settimeout(0.4)
            sock.sendall(b'{"op":"poll"}\n')
            data = b""
            while b"\n" not in data:
                chunk = sock.recv(65536)
                if not chunk:
                    break
                data += chunk
        body = json.loads(data.split(b"\n", 1)[0])
    except (OSError, json.JSONDecodeError, TimeoutError):
        return {"reachable": False, "motion_commands": 0, "register_with_rmf": False,
                "pose": None, "detail": "physical dry-run gateway is not connected"}
    if not isinstance(body, dict):
        return {"reachable": False, "motion_commands": 0, "pose": None}
    decision = body.get("decision") if isinstance(body.get("decision"), dict) else {}
    robots = body.get("robots") if isinstance(body.get("robots"), list) else []
    selected_id = body.get("selected_robot_id") or decision.get("robot_id") or "real_pinky1"
    selected = next((item for item in robots if isinstance(item, dict) and item.get("id") == selected_id), None)
    selected_available = bool(selected and selected.get("available")) if robots else body.get("selected_available")
    return {"reachable": True, "motion_commands": body.get("motion_commands", 0),
            "selected_available": selected_available,
            "refused_ops": body.get("refused_ops", 0),
            "register_with_rmf": bool(decision.get("register_with_rmf")),
            "pose": body.get("pose") if body.get("pose_source") == "odom" else None,
            "pose_source": body.get("pose_source"),
            "waypoints": body.get("waypoints") or [],
            "mode": body.get("mode"),
            "robot_id": body.get("selected_robot_id") or decision.get("robot_id") or "real_pinky1",
            "fleet_name": decision.get("fleet_name") or "physical_pinky",
            "robots": robots,
            "selected_robot_id": selected_id,
            "test_kind": body.get("test_kind") or ((body.get("segment") or {}).get("kind") if isinstance(body.get("segment"), dict) else None),
            "turned_rad": body.get("turned_rad"),
            "target_yaw_rad": body.get("target_yaw_rad"),
            "odom_angular": body.get("odom_angular"),
            "cmd_vel_topic": body.get("cmd_vel_topic"),
            "drive_domain": body.get("drive_domain") if body.get("drive_domain") is not None else int(DRIVE_DOMAIN_ID),
            "observation": body.get("observation"),
            "reasons": decision.get("reasons"),
            "domain": body.get("domain") if body.get("domain") is not None else ((body.get("observation") or {}).get("domain") if isinstance(body.get("observation"), dict) else None),
            "motor_ready": body.get("motor_ready"),
            "odom_fresh": body.get("odom_fresh"),
            "goal_phase": body.get("goal_phase") or (body.get("goal") or {}).get("phase"),
            "goal": body.get("goal"),
            "session_id": body.get("session_id") or "",
            "session_open": bool(body.get("session_open")),
            "session_submitted": bool(body.get("session_submitted")),
            "goal_generation": body.get("goal_generation"),
            "session_goal_generation": body.get("session_goal_generation"),
            "gateway_version": body.get("gateway_version"),
            "session_protocol_version": body.get("session_protocol_version"),
            "supported_test_kinds": body.get("supported_test_kinds"),
            "distance_traveled": body.get("distance_traveled"),
            "remaining_distance": body.get("remaining_distance"),
            "odom_velocity": body.get("odom_velocity"),
            "odom_timestamp": body.get("odom_timestamp"),
            "initial_pose": body.get("initial_pose"),
            "target_pose": body.get("target_pose"),
            "target_distance_m": 0.05 if body.get("target_distance_m") is None else body.get("target_distance_m"),
            "last_error": body.get("last_error"),
            "segment": body.get("segment"),
            "detail": "실물 상태입니다. Gazebo 좌표를 사용하지 않습니다."}


def gateway_command(payload: dict, timeout: float = 2.0) -> dict:
    with socket.create_connection(("127.0.0.1", 17453), timeout=timeout) as sock:
        sock.settimeout(timeout)
        sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        data = b""
        while b"\n" not in data:
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
    body = json.loads(data.split(b"\n", 1)[0])
    if not isinstance(body, dict):
        raise ConnectionError("physical gateway reply is not an object")
    return body


def active_physical_task() -> dict | None:
    terminal = {"completed", "canceled", "killed", "failed"}
    for task in TASKS.values():
        if task.get("robot_id") not in PHYSICAL_ROBOT_IDS:
            continue
        if task.get("rmf_status") in terminal or task.get("status") in terminal:
            continue
        if task.get("completed"):
            continue
        return task
    return None


def _adapter_pid() -> int:
    path = Path("/tmp/care-move-e2e/physical-adapter.pid")
    if not path.is_file():
        return 0
    try:
        return int(path.read_text(encoding="utf-8").strip() or "0")
    except ValueError:
        return 0


def _pid_alive(pid: int) -> bool:
    if pid <= 0:
        return False
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def stop_physical_adapter() -> None:
    pid = _adapter_pid()
    if not _pid_alive(pid):
        return
    os.kill(pid, signal.SIGTERM)
    for _ in range(20):
        if not _pid_alive(pid):
            return
        time.sleep(0.1)
    os.kill(pid, signal.SIGKILL)


def start_physical_adapter() -> int:
    """Start the fleet adapter on RMF domain 77. Pinky cmd_vel stays on domain 24."""
    log_path = Path("/tmp/care-move-e2e/physical-adapter.log")
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_path.write_text("", encoding="utf-8")
    env = os.environ.copy()
    env.update({
        "ROS_DOMAIN_ID": "77",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "CYCLONEDDS_URI": "file:///tmp/care-move-e2e/cyclonedds-lo.xml",
        "SIM_RMF_SERVER_URI": "ws://127.0.0.1:8000/_internal",
        "PHYSICAL_FLEET_REGISTER": "1",
        "PYTHONNOUSERSITE": "1",
    })
    log = log_path.open("ab")
    proc = subprocess.Popen(
        ["bash", "-lc",
         "source /opt/ros/jazzy/setup.bash && cd /home/sw/DevSpace/care-move-rmf-starter && exec python3 scripts/physical_fleet_adapter.py"],
        env=env, stdout=log, stderr=log, start_new_session=True,
    )
    Path("/tmp/care-move-e2e/physical-adapter.pid").write_text(str(proc.pid), encoding="utf-8")
    return proc.pid


def adapter_registered(pid: int, robot_id: str) -> bool:
    if robot_id not in PHYSICAL_ROBOT_IDS or not _pid_alive(pid):
        return False
    text = Path("/tmp/care-move-e2e/physical-adapter.log").read_text(encoding="utf-8", errors="replace")
    return f"Successfully added robot [{robot_id}]" in text


_BOOKING_CACHE: dict = {"at": 0.0, "states": None, "error": None}
_FLEET_CACHE: dict = {"at": 0.0, "robots": None, "error": None}


def active_environment() -> str:
    return os.getenv("CARE_MOVE_RMF_ENVIRONMENT", "rmf-a-domain77")


async def load_nonterminal_bookings() -> list:
    """Live RMF list. A timeout is not an empty fleet."""
    now = time.monotonic()
    if now - float(_BOOKING_CACHE["at"] or 0.0) < 2.0 and (
            _BOOKING_CACHE["states"] is not None or _BOOKING_CACHE["error"]):
        if _BOOKING_CACHE["error"]:
            raise RmfClientError(str(_BOOKING_CACHE["error"]), "RMF booking 상태를 확인하지 못했습니다.")
        return list(_BOOKING_CACHE["states"])
    try:
        states = await rmf_client.get_json("/tasks?status=queued,standby,underway,blocked")
    except RmfClientError as exc:
        _BOOKING_CACHE.update(at=now, states=None, error=exc.kind)
        raise
    if not isinstance(states, list):
        _BOOKING_CACHE.update(at=now, states=None, error="invalid_response")
        raise RmfClientError("invalid_response", "RMF task list is not a list")
    _BOOKING_CACHE.update(at=now, states=states, error=None)
    return states


async def load_fleet_robots() -> dict:
    """Robot records from the fleet state. A failure is not an empty task id."""
    now = time.monotonic()
    if now - float(_FLEET_CACHE["at"] or 0.0) < 2.0 and (
            _FLEET_CACHE["robots"] is not None or _FLEET_CACHE["error"]):
        if _FLEET_CACHE["error"]:
            raise RmfClientError(str(_FLEET_CACHE["error"]), "RMF fleet 상태를 확인하지 못했습니다.")
        return dict(_FLEET_CACHE["robots"])
    try:
        state = await rmf_client.get_json(f"/fleets/{PHYSICAL_FLEET_NAME}/state")
    except RmfClientError as exc:
        _FLEET_CACHE.update(at=now, robots=None, error=exc.kind)
        raise
    robots = state.get("robots") if isinstance(state, dict) else None
    if not isinstance(robots, dict):
        _FLEET_CACHE.update(at=now, robots=None, error="invalid_response")
        raise RmfClientError("invalid_response", "RMF fleet state has no robots")
    _FLEET_CACHE.update(at=now, robots=robots, error=None)
    return dict(robots)


async def booking_recovery(robot_id: str, gateway: dict | None = None) -> dict:
    """Return the latest snapshot. This does not add an observation."""
    del gateway
    return reconciler.view(robot_id)


async def judge_robot(robot_id: str, gateway: dict | None, mono_ms: int) -> dict:
    gateway_ok = True if gateway is None else bool(gateway.get("reachable"))
    goal_phase = (gateway or {}).get("goal_phase")
    goal_active = goal_phase in {"moving", "stopping"}
    rmf_ok = True
    fleet_known = True
    states: list = []
    fleet_task_id = ""
    try:
        states = await load_nonterminal_bookings()
    except (RmfClientError, ValueError):
        rmf_ok = False
    if rmf_ok:
        try:
            robots = await load_fleet_robots()
            robot = robots.get(robot_id) if isinstance(robots.get(robot_id), dict) else {}
            fleet_task_id = str(robot.get("task_id") or "")
        except (RmfClientError, ValueError):
            fleet_known = False
    try:
        bookings = conflicting_bookings(states, robot_id) if rmf_ok else []
    except ValueError:
        rmf_ok = False
        bookings = []
    judged = judge(
        robot_id=robot_id,
        rmf_ok=rmf_ok,
        gateway_ok=gateway_ok,
        fleet_known=fleet_known,
        bookings=bookings,
        fleet_task_id=fleet_task_id,
        gateway_goal_active=goal_active,
        at_ms=mono_ms,
        log=reconciler.log,
        window_ms=reconciler.window_ms,
    )
    for row in judged["reports"]:
        row["id"] = row.get("booking_id")
    judged["bookings"] = judged["reports"]
    return judged


async def refresh_recovery() -> dict:
    """Take one observation epoch. A concurrent caller does not add another sample."""
    mono_ms = int(time.monotonic() * 1000)
    if not reconciler.begin_epoch(mono_ms):
        board = reconciler.board()
        board["duplicate"] = True
        board["motion"] = False
        return board
    try:
        gateway = await asyncio.to_thread(read_physical_gateway)
        reports = {}
        for robot_id in PHYSICAL_ROBOT_IDS:
            reports[robot_id] = await judge_robot(robot_id, gateway, mono_ms)
        reconciler.commit_epoch(reports, mono_ms)
    except Exception as exc:
        reconciler.fail_epoch(str(exc))
    return reconciler.board()


async def reconcile_loop() -> None:
    interval = float(os.getenv("CARE_MOVE_RECONCILE_INTERVAL_SEC", "5"))
    while True:
        try:
            await refresh_recovery()
        except Exception as exc:
            reconciler.fail_epoch(str(exc))
        await asyncio.sleep(interval)


@app.get("/api/physical/status")
async def physical_status():
    status = await asyncio.to_thread(read_physical_gateway)
    robot_id = str(status.get("selected_robot_id") or status.get("robot_id") or "")
    recovery = await booking_recovery(robot_id, status) if robot_id else {
        "state": "UNKNOWN", "reconciliation_status": "UNKNOWN", "bookings": [],
        "reason": "선택한 Pinky가 없습니다.", "recovery_required": True,
        "adapter_task_owned": False, "fleet_task_id": ""}
    reason = None if local_test_ui_enabled() else "로컬 시험 UI가 켜져 있지 않습니다."
    if reason is None:
        reason = gateway_protocol_block(status)
    if reason is None and recovery["state"] != "CLEAR":
        reason = recovery["reason"]
    if reason is None:
        reason = button_block(status)
    if reason is None and active_physical_task() is not None:
        reason = "종료되지 않은 실물 Task가 있습니다."
    primary = recovery["bookings"][0] if recovery.get("bookings") else {}
    status["local_test_ui"] = local_test_ui_enabled()
    status["recovery_state"] = "RECOVERY_REQUIRED" if recovery.get("recovery_required") else recovery.get("state")
    status["reconciliation_status"] = recovery.get("reconciliation_status")
    status["unfinished_bookings"] = recovery["bookings"]
    status["unfinished_booking_id"] = primary.get("id") or primary.get("booking_id")
    status["rmf_task_status"] = primary.get("rmf_status") or primary.get("status")
    status["adapter_task_owned"] = recovery.get("adapter_task_owned")
    status["fleet_task_id"] = recovery.get("fleet_task_id")
    status["recovery_reason"] = recovery.get("reason")
    status["dispatch_allowed"] = recovery.get("dispatch_allowed")
    status["test_block"] = reason
    status["test_ready"] = reason is None
    return status


def authorize_admin(request: Request) -> None:
    """Loopback local UI may read recovery. Every other caller needs the operator token."""
    if local_test_ui_enabled():
        reason = drive_request_blocked(request)
        if reason:
            raise HTTPException(403, reason)
        return
    require_operator(request)


async def recovery_snapshot() -> dict:
    gateway = await asyncio.to_thread(read_physical_gateway)
    robots = []
    for robot_id in PHYSICAL_ROBOT_IDS:
        report = await booking_recovery(robot_id, gateway)
        report["robot_id"] = robot_id
        robots.append(report)
    return {"robots": robots, "motion": False}


@app.get("/api/admin/recovery/tasks")
async def recovery_tasks(request: Request):
    authorize_admin(request)
    return await recovery_snapshot()


@app.get("/api/admin/recovery/tasks/{booking_id}")
async def recovery_task(request: Request, booking_id: str):
    authorize_admin(request)
    snapshot = await recovery_snapshot()
    for robot in snapshot["robots"]:
        for booking in robot.get("bookings") or []:
            if booking.get("id") == booking_id or booking.get("booking_id") == booking_id:
                return {**booking, "robot_id": robot["robot_id"], "found": True, "motion": False}
    return {"booking_id": booking_id, "found": False, "reconciliation_status": "UNKNOWN",
            "recovery_required": False, "motion": False}


@app.post("/api/admin/recovery/reconcile")
async def recovery_reconcile(request: Request):
    """One new observation. A duplicate in-flight call does not add a sample or move the robot."""
    authorize_admin(request)
    return await refresh_recovery()


@app.get("/api/admin/recovery/health")
async def recovery_health(request: Request):
    authorize_admin(request)
    board = reconciler.board()
    isolated = _isolated_evidence()
    monitoring = dict(board["monitoring"])
    monitoring["gateway_mode"] = None
    return {
        "observed": board["observed"],
        "environment_id": board["environment_id"],
        "monitoring": monitoring,
        "isolated": isolated,
        "motion": False,
    }


def _isolated_evidence() -> dict:
    path = Path("/tmp/care-move-isolated/result.json")
    if not path.is_file():
        return {"environment_id": "rmf-b-domain91", "status": "UNKNOWN", "verified": False}
    try:
        body = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {"environment_id": "rmf-b-domain91", "status": "UNKNOWN", "verified": False}
    if body.get("environment_id") != "rmf-b-domain91" or body.get("verified") is not True:
        return {"environment_id": "rmf-b-domain91", "status": "UNKNOWN", "verified": False}
    return {
        "environment_id": "rmf-b-domain91",
        "status": "verified",
        "verified": True,
        "mock_robots": body.get("mock_robots") or [],
        "physical_gateway": False,
    }


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


PHYSICAL_ROBOT_ID = "real_pinky1"
PHYSICAL_FLEET_NAME = "physical_pinky"


def fleet_for_robot(robot_id: str | None) -> str:
    if robot_id in PHYSICAL_ROBOT_IDS:
        return PHYSICAL_FLEET_NAME
    return FLEET


def build_rmf_payload(req: TaskRequest, fleet_name: str | None = None) -> tuple[str, dict]:
    """Payload per the running server's RobotTaskRequest / DispatchTaskRequest schemas."""
    selected = fleet_name or fleet_for_robot(req.robot_id)
    request = {"category": "patrol", "description": {"places": req.places, "rounds": req.rounds},
               "requester": "care-move-dashboard", "fleet_name": selected}
    if req.robot_id:
        return "/tasks/robot_task", {
            "type": "robot_task_request", "robot": req.robot_id,
            "fleet": selected, "request": request
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


def stamp_correlation(payload: dict, correlation_id: str) -> None:
    request = payload.setdefault("request", {})
    request["labels"] = [f"care-move-request={correlation_id}"]
    request["requester"] = f"care-move:{correlation_id}"
    request["unix_millis_request_time"] = int(time.time() * 1000)


def record_unknown(req: TaskRequest, correlation_id: str, detail: str) -> dict:
    local_id = "rmf-request-" + uuid.uuid4().hex[:8]
    task = {"id": local_id, "rmf_task_id": None, "correlation_id": correlation_id,
            "robot_id": req.robot_id or "AUTO", "places": req.places, "rounds": req.rounds,
            "status": "submission_unknown", "submission": "SUBMISSION_UNKNOWN",
            "rmf_status": None, "source": "RMF", "accepted": False, "completed": False,
            "error": detail,
            "rmf_response": {"success": False, "booking_id": None}}
    TASKS[local_id] = task
    return task


def record_submission(req: TaskRequest, result: dict, correlation_id: str | None = None) -> dict:
    state = result.get("state") if isinstance(result, dict) else None
    if not isinstance(state, dict) or not isinstance(state.get("booking"), dict) or not state["booking"].get("id"):
        raise HTTPException(502, {"kind": "invalid_response",
                                   "detail": "RMF 응답에 task booking id가 없습니다. 접수 성공으로 기록하지 않았습니다."})
    rmf_task_id = state["booking"]["id"]
    local_id = "rmf-request-" + uuid.uuid4().hex[:8]
    task = {"id": local_id, "rmf_task_id": rmf_task_id, "correlation_id": correlation_id,
            "robot_id": req.robot_id or "AUTO",
            "places": req.places, "rounds": req.rounds, "status": "rmf_accepted_submission",
            "submission": "SUBMISSION_ACCEPTED",
            "rmf_status": state.get("status"), "source": "RMF", "accepted": True,
            "completed": False, "error": None,
            "rmf_response": {"success": True, "booking_id": rmf_task_id, "rmf_status": state.get("status")}}
    TASKS[local_id] = task
    return task


def physical_task_block(places: list[str], robot_id: str | None = None) -> str | None:
    status = read_physical_gateway()
    if not status.get("reachable"):
        return "실물 gateway가 연결되지 않아 작업을 보내지 않았습니다."
    if status.get("mode") != "PHYSICAL_TEST":
        return "실물 gateway가 PHYSICAL_TEST가 아니라 이동 작업을 보내지 않았습니다."
    if str(status.get("domain") or "") not in {DRIVE_DOMAIN_ID, ""}:
        return "실물 주행 도메인이 Pinky ROS domain 24가 아닙니다."
    segment = status.get("segment") if isinstance(status.get("segment"), dict) else {}
    if segment.get("kind") not in (None, "", FORWARD_TEST):
        return "준비된 테스트가 5cm 전진이 아닙니다."
    if robot_id and segment.get("robot_id") and segment.get("robot_id") != robot_id:
        return "준비된 Pinky와 요청 로봇이 다릅니다."
    allowed = set(status.get("waypoints") or [])
    if not allowed or any(place not in allowed for place in places):
        return "측정된 실물 waypoint가 없어 작업을 보내지 않았습니다."
    if status.get("pose_source") != "odom" or not status.get("pose"):
        return "엔코더 odometry가 없어 작업을 보내지 않았습니다."
    if not status.get("session_open"):
        return "준비된 5cm session이 없습니다."
    return None


def rotate_task_payload(robot_id: str) -> tuple[str, dict]:
    request = {
        "category": "compose",
        "description": {
            "category": ROTATE_TEST,
            "detail": "in-place 180 degree encoder test",
            "phases": [{
                "activity": {
                    "category": "perform_action",
                    "description": {
                        "category": "rotate_180",
                        "description": {"yaw_delta_rad": TARGET_YAW_RAD},
                        "unix_millis_action_duration_estimate": 15000,
                    },
                }
            }],
        },
        "requester": "care-move-dashboard",
        "fleet_name": PHYSICAL_FLEET_NAME,
    }
    return "/tasks/robot_task", {
        "type": "robot_task_request",
        "robot": robot_id,
        "fleet": PHYSICAL_FLEET_NAME,
        "request": request,
    }


async def run_physical_test(kind: str, robot_id: str | None) -> dict:
    """Prepare the selected Pinky, register that one robot, then submit one RMF task."""
    if MODE == "mock":
        raise HTTPException(409, "mock 모드에서는 실물 Pinky 작업을 만들지 않습니다.")
    if kind not in {FORWARD_TEST, ROTATE_TEST}:
        raise HTTPException(409, "지원하는 실물 테스트는 5cm 전진과 180도 회전뿐입니다.")
    if robot_id:
        try:
            chosen = await asyncio.to_thread(gateway_command, {"op": "select", "robot_id": robot_id})
        except OSError as exc:
            raise HTTPException(409, f"Pinky 선택에 실패했습니다: {exc}") from exc
        if not chosen.get("ok"):
            raise HTTPException(409, chosen.get("reason") or "Pinky를 선택하지 못했습니다.")
    status = await asyncio.to_thread(read_physical_gateway)
    reason = button_block(status)
    if reason is None:
        reason = gateway_protocol_block(status)
    if reason is None and active_physical_task() is not None:
        reason = "종료되지 않은 실물 Task가 있습니다."
    selected = str(status.get("selected_robot_id") or status.get("robot_id") or "")
    if reason is None and selected:
        recovery = await booking_recovery(selected, status)
        if recovery["state"] != "CLEAR":
            reason = recovery["reason"]
    if reason is None:
        reason = fence_physical(active_environment(), 17453)
    if reason:
        raise HTTPException(409, reason)
    if status.get("mode") != "PHYSICAL_TEST":
        try:
            armed = await asyncio.to_thread(gateway_command, {"op": "arm", "robot_id": selected, "kind": kind})
        except OSError as exc:
            raise HTTPException(409, f"Gateway 모드 전환에 실패했습니다: {exc}") from exc
        if not armed.get("ok"):
            raise HTTPException(409, armed.get("reason") or "PHYSICAL_TEST로 전환하지 못했습니다.")
    try:
        prepared = await asyncio.to_thread(gateway_command, {"op": "prepare", "kind": kind})
    except OSError as exc:
        await asyncio.to_thread(gateway_command, {"op": "disarm", "robot_id": selected})
        raise HTTPException(409, f"실물 gateway 준비에 실패했습니다: {exc}") from exc
    if not prepared.get("ok"):
        await asyncio.to_thread(gateway_command, {"op": "disarm", "robot_id": selected})
        raise HTTPException(409, prepared.get("reason") or "테스트 session을 만들지 못했습니다.")
    selected = str(prepared.get("robot_id") or selected)
    try:
        await asyncio.to_thread(stop_physical_adapter)
        pid = await asyncio.to_thread(start_physical_adapter)
    except OSError as exc:
        await asyncio.to_thread(gateway_command, {"op": "abandon"})
        await asyncio.to_thread(gateway_command, {"op": "disarm", "robot_id": selected})
        raise HTTPException(409, f"Physical Fleet Adapter를 시작하지 못했습니다: {exc}") from exc
    deadline = time.monotonic() + 15
    while time.monotonic() < deadline and not adapter_registered(pid, selected):
        await asyncio.sleep(0.4)
    if not adapter_registered(pid, selected):
        await asyncio.to_thread(gateway_command, {"op": "abandon"})
        await asyncio.to_thread(gateway_command, {"op": "disarm", "robot_id": selected})
        raise HTTPException(409, f"RMF에 {selected}이 등록되지 않아 작업을 보내지 않았습니다.")
    try:
        if kind == ROTATE_TEST:
            correlation_id = uuid.uuid4().hex
            path, payload = rotate_task_payload(selected)
            stamp_correlation(payload, correlation_id)
            log_task_event("TASK_REQUEST_RECEIVED", correlation_id=correlation_id,
                           robot_id=selected, fleet_name=PHYSICAL_FLEET_NAME, waypoint=["rotate_180"],
                           ros_domain=DRIVE_DOMAIN_ID)
            rotate_req = TaskRequest(robot_id=selected, places=["P_START"], rounds=1)
            try:
                result = await rmf_post(path, payload)
            except SubmissionUnknown as ex:
                task = record_unknown(rotate_req, ex.correlation_id, ex.detail)
            else:
                task = record_submission(rotate_req, result, correlation_id)
                log_task_event("RMF_TASK_ACCEPTED", correlation_id=correlation_id,
                               rmf_booking_id=task.get("rmf_task_id"), task_status="SUBMISSION_ACCEPTED")
        else:
            task = await submit_task(TaskRequest(robot_id=selected, places=["P_TEST"], rounds=1))
    except HTTPException:
        await asyncio.to_thread(gateway_command, {"op": "abandon"})
        await asyncio.to_thread(gateway_command, {"op": "disarm", "robot_id": selected})
        raise
    if isinstance(task, dict):
        segment = prepared.get("segment") or {}
        task["session_id"] = prepared.get("session_id")
        task["test_kind"] = kind
        task["target_pose"] = segment.get("P_TURN") if kind == ROTATE_TEST else segment.get("P_TEST")
        task["initial_pose"] = segment.get("P_START")
        task["target_distance_m"] = 0.0 if kind == ROTATE_TEST else 0.05
        task["target_yaw_rad"] = TARGET_YAW_RAD if kind == ROTATE_TEST else None
        _remember_physical_task(task, selected, kind)
        if task.get("session_id"):
            _spawn_watch(selected, str(task["session_id"]))
    return task


def _remember_physical_task(task: dict, robot_id: str, kind: str) -> None:
    booking = task.get("rmf_task_id")
    try:
        task_store.put({
            "request_id": task.get("id") or booking or uuid.uuid4().hex,
            "correlation_id": task.get("correlation_id"),
            "rmf_booking_id": booking,
            "robot_id": robot_id,
            "fleet_id": PHYSICAL_FLEET_NAME,
            "origin": "P_START",
            "destination": "P_TURN" if kind == ROTATE_TEST else "P_TEST",
            "action": kind,
            "trip_type": "ONE_WAY",
            "status": "RMF_ACCEPTED" if booking else "SUBMISSION_UNKNOWN",
            "dispatch_allowed": False,
            "kind": kind,
            "error": None if booking else task.get("error"),
        })
    except ValueError:
        return


def _spawn_watch(robot_id: str, session_id: str) -> None:
    watcher = asyncio.create_task(_return_to_dry_run(robot_id, session_id))
    _WATCHERS.add(watcher)
    watcher.add_done_callback(_WATCHERS.discard)


def dry_run_ready(status: dict, session_id: str) -> bool:
    """Completion requires this session's go generation, not an older succeeded goal."""
    if status.get("session_id") != session_id or not status.get("session_submitted"):
        return False
    generation = status.get("goal_generation")
    if not isinstance(generation, int) or generation != status.get("session_goal_generation"):
        return False
    phase = status.get("goal_phase")
    speed = abs(float(status.get("odom_velocity") or 0.0))
    angular = abs(float(status.get("odom_angular") or 0.0))
    return phase in {"succeeded", "failed", "canceled"} and speed <= 0.01 and angular <= 0.05


def stopped_terminal(status: dict, session_id: str) -> bool:
    """A late disarm is allowed only after go was accepted and the robot is stopped."""
    if status.get("session_id") != session_id or not status.get("session_submitted"):
        return False
    phase = status.get("goal_phase")
    speed = abs(float(status.get("odom_velocity") or 0.0))
    angular = abs(float(status.get("odom_angular") or 0.0))
    if phase in {"moving", "stopping"} or speed > 0.01 or angular > 0.05:
        return False
    return phase in {"succeeded", "failed", "canceled"}


async def _return_to_dry_run(robot_id: str, session_id: str) -> None:
    """After this session's goal stops, put only that Pinky back in DRY_RUN."""
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        await asyncio.sleep(0.5)
        status = await asyncio.to_thread(read_physical_gateway)
        if dry_run_ready(status, session_id):
            await asyncio.to_thread(gateway_command, {"op": "disarm", "robot_id": robot_id})
            return
    status = await asyncio.to_thread(read_physical_gateway)
    if stopped_terminal(status, session_id):
        await asyncio.to_thread(gateway_command, {"op": "disarm", "robot_id": robot_id})


def local_test_ui_enabled() -> bool:
    """Explicit local-dev switch. An unset or partial setting does not allow motion."""
    environment = os.getenv("CARE_MOVE_ENV", "").strip().lower()
    flag = os.getenv("CARE_MOVE_LOCAL_TEST_UI_ENABLED", "").strip().lower()
    return environment == "local" and flag in {"1", "true", "yes"}


def drive_request_blocked(request: Request) -> str | None:
    """None when the request is the loopback test UI. This is not a user login."""
    if not local_test_ui_enabled():
        return "local test ui is not enabled"
    client = request.client.host if request.client else ""
    if client not in {"127.0.0.1", "::1"}:
        return "drive client is not loopback"
    host = (request.headers.get("host") or "").lower()
    if host not in {"127.0.0.1:8011", "localhost:8011", "[::1]:8011"}:
        return "drive host is not local"
    if any(request.headers.get(name) for name in ("x-forwarded-for", "x-forwarded-host", "forwarded")):
        return "proxied drive request"
    origin = request.headers.get("origin")
    if origin and origin not in {"http://127.0.0.1:8011", "http://localhost:8011", "http://[::1]:8011"}:
        return "drive origin is not local"
    return None


def require_operator(request: Request) -> None:
    expected = os.getenv("CARE_MOVE_OPERATOR_TOKEN", "")
    header = request.headers.get("authorization", "")
    if not expected or not hmac.compare_digest(header, f"Bearer {expected}"):
        raise HTTPException(401, "operator token is required")


def authorize_drive(request: Request, kind: str, robot_id: str | None) -> None:
    """Local loopback UI may skip the operator token. Every other caller may not."""
    if local_test_ui_enabled():
        reason = drive_request_blocked(request)
        if reason:
            log_task_event("PHYSICAL_DRIVE_DENIED", kind=kind, robot_id=robot_id or "", reason=reason)
            raise HTTPException(403, reason)
        log_task_event("PHYSICAL_DRIVE_AUTHORIZED", kind=kind, robot_id=robot_id or "", local_test_ui=True)
        return
    try:
        require_operator(request)
    except HTTPException:
        log_task_event("PHYSICAL_DRIVE_DENIED", kind=kind, robot_id=robot_id or "", reason="operator token rejected")
        raise
    log_task_event("PHYSICAL_DRIVE_AUTHORIZED", kind=kind, robot_id=robot_id or "", local_test_ui=False)


@app.post("/api/physical/select")
async def physical_select(req: PhysicalSelectRequest):
    if req.robot_id not in PHYSICAL_ROBOT_IDS:
        raise HTTPException(404, "Unknown robot")
    try:
        reply = await asyncio.to_thread(gateway_command, {"op": "select", "robot_id": req.robot_id})
    except OSError as exc:
        raise HTTPException(409, f"Pinky 선택에 실패했습니다: {exc}") from exc
    if not reply.get("ok"):
        raise HTTPException(409, reply.get("reason") or "Pinky를 선택하지 못했습니다.")
    return reply


@app.get("/api/physical/robots")
async def physical_robots():
    status = await asyncio.to_thread(read_physical_gateway)
    modes = {item.get("id"): item.get("gateway_mode") for item in status.get("robots") or []}
    return {"fleet": PHYSICAL_FLEET_NAME, "drive_domain": int(DRIVE_DOMAIN_ID), "rmf_domain": 77,
            "robots": robot_registry(status.get("robots") or [], modes, status.get("goal_phase") or "idle",
                                     status.get("selected_robot_id") or "")}


@app.get("/api/physical/queue")
async def physical_queue(robot_id: str | None = None):
    rows = task_store.list_robot(robot_id)
    return {"robot_id": robot_id, "tasks": rows,
            "current": [row for row in rows if row["status"] not in {"QUEUED", "PENDING", "COMPLETED", "FAILED", "CANCELED", "INTERRUPTED"}],
            "queued": [row for row in rows if row["status"] == "QUEUED"],
            "history": [row for row in rows if row["status"] in {"COMPLETED", "FAILED", "CANCELED", "INTERRUPTED"}]}


@app.post("/api/physical/queue/{request_id}/cancel")
async def cancel_queued_task(request_id: str):
    try:
        row = task_store.cancel_queued(request_id)
    except ValueError as exc:
        raise HTTPException(409, str(exc)) from exc
    if row is None:
        raise HTTPException(404, "Unknown task")
    return row


@app.get("/api/maps")
async def maps():
    return load_registry(ROOT)


@app.get("/api/rmf/schedule")
async def rmf_schedule():
    return await asyncio.to_thread(schedule_view)


def _transport_robots() -> list[dict]:
    status = read_physical_gateway()
    modes = {item.get("id"): item.get("gateway_mode") for item in status.get("robots") or []}
    return robot_registry(status.get("robots") or [], modes, status.get("goal_phase") or "idle",
                          status.get("selected_robot_id") or "")


@app.post("/api/v1/transport/requests", status_code=202)
async def create_transport(request: Request, body: dict):
    require_operator(request)
    try:
        robots = await asyncio.to_thread(_transport_robots)
        result = accept_transport(task_store, body, WAYPOINTS, robots)
    except ValueError as exc:
        message = str(exc)
        if message == "duplicate request id":
            raise HTTPException(409, message) from exc
        raise HTTPException(400, message) from exc
    return result


@app.get("/api/v1/transport/requests/{request_id}")
async def read_transport(request: Request, request_id: str):
    require_operator(request)
    row = task_store.get(request_id)
    if row is None:
        raise HTTPException(404, "Unknown request")
    return {"request_id": request_id, "accepted": True, "status": row["status"],
            "robot_id": row["robot_id"], "rmf_booking_id": row["rmf_booking_id"],
            "execution": "not_submitted" if row["rmf_booking_id"] is None else "submitted",
            "detail": row["error"]}


@app.post("/api/physical/session", status_code=204)
async def physical_session(request: Request):
    """Check the operator token. This does not arm a robot or open a test session."""
    require_operator(request)


@app.post("/api/physical/test", status_code=202)
async def physical_test(request: Request, req: PhysicalTestRequest):
    authorize_drive(request, req.kind, req.robot_id)
    return await run_physical_test(req.kind, req.robot_id)


@app.post("/api/physical/test-5cm", status_code=202)
async def physical_test_5cm(request: Request):
    """Prepare a 5cm session from the live pose, register the fleet, then submit one RMF task."""
    authorize_drive(request, FORWARD_TEST, None)
    return await run_physical_test(FORWARD_TEST, None)


@app.post("/api/tasks", status_code=202)
async def create_task(request: Request, req: TaskRequest):
    if req.robot_id in PHYSICAL_ROBOT_IDS:
        authorize_drive(request, "task", req.robot_id)
    return await submit_task(req)


async def submit_task(req: TaskRequest):
    if req.robot_id in PHYSICAL_ROBOT_IDS:
        if MODE == "mock":
            raise HTTPException(409, "mock 모드에서는 실물 Pinky 작업을 만들지 않습니다.")
        reason = physical_task_block(req.places, req.robot_id)
        if reason is None:
            reason = gateway_protocol_block(read_physical_gateway())
        if reason is None:
            recovery = await booking_recovery(req.robot_id or "", read_physical_gateway())
            if recovery["state"] != "CLEAR":
                reason = recovery["reason"]
        if reason is None:
            reason = fence_physical(active_environment(), 17453)
        if reason:
            raise HTTPException(409, reason)
        correlation_id = uuid.uuid4().hex
        path, payload = build_rmf_payload(req, PHYSICAL_FLEET_NAME)
        stamp_correlation(payload, correlation_id)
        log_task_event("TASK_REQUEST_RECEIVED", correlation_id=correlation_id,
                       robot_id=req.robot_id, fleet_name=PHYSICAL_FLEET_NAME, waypoint=req.places,
                       ros_domain=DRIVE_DOMAIN_ID)
        try:
            result = await rmf_post(path, payload)
        except SubmissionUnknown as ex:
            return record_unknown(req, ex.correlation_id, ex.detail)
        task = record_submission(req, result, correlation_id)
        log_task_event("RMF_TASK_ACCEPTED", correlation_id=correlation_id,
                       rmf_booking_id=task.get("rmf_task_id"), task_status="SUBMISSION_ACCEPTED")
        return task
    if MODE == "rmf" and RMF_STATUS.get("status") in {None, "not_checked"}:
        await refresh_rmf_observation(apply_robots=True)
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
    reason = dispatch_block_reason(req.robot_id, req.places)
    if reason:
        raise HTTPException(409, reason)
    correlation_id = uuid.uuid4().hex
    path, payload = build_rmf_payload(req)
    stamp_correlation(payload, correlation_id)
    log_task_event("TASK_REQUEST_RECEIVED", correlation_id=correlation_id,
                   robot_id=req.robot_id, fleet_name=FLEET, waypoint=req.places,
                   ros_domain=os.getenv("ROS_DOMAIN_ID"))
    try:
        result = await rmf_post(path, payload)
    except SubmissionUnknown as ex:
        return record_unknown(req, ex.correlation_id, ex.detail)
    task = record_submission(req, result, correlation_id)
    log_task_event("RMF_TASK_ACCEPTED", correlation_id=correlation_id,
                   rmf_booking_id=task.get("rmf_task_id"), task_status="SUBMISSION_ACCEPTED")
    return task


@app.post("/api/tasks/{task_id}/cancel")
async def cancel_task(request: Request, task_id: str):
    task = TASKS.get(task_id)
    if task is None:
        task = next((item for item in TASKS.values()
                     if item.get("rmf_task_id") == task_id), None)
    if task is None:
        raise HTTPException(404, "Unknown task")
    if task.get("robot_id") in PHYSICAL_ROBOT_IDS:
        authorize_drive(request, "cancel", task.get("robot_id"))
        try:
            stopped = await asyncio.to_thread(gateway_command, {"op": "cancel", "robot_id": task.get("robot_id")})
        except OSError as exc:
            log_task_event("PHYSICAL_CANCEL_GATEWAY_UNREACHABLE", robot_id=task.get("robot_id"))
            raise HTTPException(409, f"Gateway 정지 요청에 실패했습니다: {exc}") from exc
        phase = (stopped.get("goal") or {}).get("phase") or stopped.get("phase")
        task["gateway_cancel"] = {"ok": bool(stopped.get("ok")), "phase": phase}
        task["status"] = "cancel_requested"
        task["completed"] = False
        log_task_event("PHYSICAL_CANCEL_REQUESTED", robot_id=task.get("robot_id"), phase=phase)
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
