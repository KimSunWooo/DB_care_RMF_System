"""Start and check an RMF stack that cannot see the operating system.

The script refuses production domains, ports, and database files before it
starts anything. On failure it stops only the processes it started.
"""
from __future__ import annotations

import hashlib
import json
import os
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from app.isolated_env import ISOLATED_STACK, isolation_problems

WORKDIR = Path("/tmp/care-move-isolated")
CONTAINER = "care-move-isolated-rmf-api"
IMAGE = "ghcr.io/open-rmf/rmf-web/api-server:jazzy-nightly"
PRODUCTION_DB = Path("/tmp/care-move-e2e/rmf_api_server.db")
QUEUE = ROOT / "data" / "task_queue.sqlite"
BOOKING = "7c5e2da1-e89a-44e3-9d68-591a027c4922"
SERVER = "ws://127.0.0.1:8001/_internal"


def _sha(path: Path) -> str | None:
    if not path.is_file():
        return None
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _run(args: list[str], timeout: float = 20) -> subprocess.CompletedProcess:
    return subprocess.run(args, text=True, capture_output=True, timeout=timeout, check=False)


def _isolated_env(extra: dict | None = None) -> dict:
    env = {
        "HOME": os.environ.get("HOME", "/tmp"),
        "USER": os.environ.get("USER", "sw"),
        "PATH": "/opt/ros/jazzy/bin:/usr/bin:/bin",
        "LANG": "C.UTF-8",
        "LC_ALL": "C.UTF-8",
        "PYTHONNOUSERSITE": "1",
        "ROS_DOMAIN_ID": "91",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "CYCLONEDDS_URI": "file:///tmp/care-move-isolated/cyclonedds-lo.xml",
        "ISOLATED_RMF_SERVER_URI": SERVER,
    }
    if extra:
        env.update(extra)
    return env


def _nodes(domain: str, rmw: str, cyclone: str | None) -> list[str]:
    env = _isolated_env()
    env["ROS_DOMAIN_ID"] = domain
    env["RMW_IMPLEMENTATION"] = rmw
    if cyclone:
        env["CYCLONEDDS_URI"] = cyclone
    else:
        env.pop("CYCLONEDDS_URI", None)
    command = "source /opt/ros/jazzy/setup.bash && ros2 node list"
    try:
        result = subprocess.run(["bash", "-lc", command], env=env, text=True, capture_output=True, timeout=12, check=False)
    except subprocess.TimeoutExpired as exc:
        return [f"TIMEOUT {exc}"]
    lines = [line.strip() for line in (result.stdout or "").splitlines() if line.strip()]
    if result.returncode != 0 and not lines:
        lines = [f"ERROR {result.stderr.strip()[:180]}"]
    return sorted(lines)


def _gateway() -> dict:
    try:
        with socket.create_connection(("127.0.0.1", 17453), 2) as sock:
            sock.settimeout(2)
            sock.sendall(b'{"op":"poll"}\n')
            data = b""
            while b"\n" not in data:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                data += chunk
        body = json.loads(data.split(b"\n", 1)[0])
    except OSError as exc:
        return {"reachable": False, "error": str(exc)}
    return {
        "reachable": True,
        "mode": body.get("mode"),
        "motion_commands": body.get("motion_commands"),
        "goal_phase": body.get("goal_phase"),
        "pid_hint": body.get("gateway_version"),
    }


def _production_booking() -> dict:
    """Read the legacy booking through Care-Move. The static API token can be expired."""
    try:
        with urllib.request.urlopen("http://127.0.0.1:8011/api/physical/status", timeout=5) as response:
            state = json.loads(response.read().decode("utf-8"))
    except (OSError, urllib.error.URLError, json.JSONDecodeError) as exc:
        return {"readable": False, "error": type(exc).__name__}
    return {
        "readable": True,
        "status": state.get("rmf_task_status"),
        "cancellation": None,
        "id": state.get("unfinished_booking_id"),
        "recovery_state": state.get("recovery_state"),
    }


def _snapshot(label: str) -> dict:
    return {
        "label": label,
        "production_db": _sha(PRODUCTION_DB),
        "queue": _sha(QUEUE),
        "gateway": _gateway(),
        "booking": _production_booking(),
        "nodes_77": _nodes("77", "rmw_cyclonedds_cpp", "file:///tmp/care-move-e2e/cyclonedds-lo.xml"),
        "nodes_24": _nodes("24", "rmw_fastrtps_cpp", None),
    }


def _jwt(secret: str) -> str:
    import base64
    import hashlib
    import hmac
    def pack(value: dict) -> bytes:
        raw = json.dumps(value, separators=(",", ":")).encode("utf-8")
        return base64.urlsafe_b64encode(raw).rstrip(b"=")
    header = pack({"alg": "HS256", "typ": "JWT"})
    payload = pack({
        "preferred_username": "care-move-isolated",
        "aud": "rmf_api_server",
        "iss": "stub",
        "exp": int(time.time()) + 3600,
    })
    signature = hmac.new(secret.encode("utf-8"), header + b"." + payload, hashlib.sha256).digest()
    signed = base64.urlsafe_b64encode(signature).rstrip(b"=")
    return (header + b"." + payload + b"." + signed).decode("ascii")


def _http(method: str, path: str, token: str, body: dict | None = None, timeout: float = 8) -> tuple[int, dict | list | str]:
    data = None if body is None else json.dumps(body).encode("utf-8")
    request = urllib.request.Request(
        f"http://127.0.0.1:8001{path}",
        data=data,
        method=method,
        headers={"Authorization": f"Bearer {token}", "Content-Type": "application/json"},
    )
    try:
        with urllib.request.urlopen(request, timeout=timeout) as response:
            raw = response.read().decode("utf-8")
            parsed: dict | list | str
            try:
                parsed = json.loads(raw) if raw else {}
            except json.JSONDecodeError:
                parsed = raw[:300]
            return response.status, parsed
    except urllib.error.HTTPError as exc:
        raw = exc.read().decode("utf-8", errors="replace")
        try:
            parsed = json.loads(raw)
        except json.JSONDecodeError:
            parsed = raw[:300]
        return exc.code, parsed
    except urllib.error.URLError as exc:
        return 0, str(exc.reason)


def _write_runtime_files(secret: str) -> None:
    WORKDIR.mkdir(parents=True, exist_ok=True)
    database = WORKDIR / "rmf.sqlite"
    if not database.exists():
        database.write_bytes(b"")
    (WORKDIR / "cyclonedds-lo.xml").write_text(
        """<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS>
  <Domain id="91">
    <General>
      <Interfaces>
        <NetworkInterface name="lo" priority="default" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="127.0.0.1"/>
      </Peers>
      <ParticipantIndex>auto</ParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
""",
        encoding="utf-8",
    )
    (WORKDIR / "api_config.py").write_text(
        "from api_server.default_config import config\n"
        "config['db_url'] = 'sqlite:////run/isolated_rmf.db'\n"
        "config['host'] = '127.0.0.1'\n"
        "config['port'] = 8001\n"
        "config['public_url'] = 'http://127.0.0.1:8001'\n"
        f"config['jwt_secret'] = {secret!r}\n",
        encoding="utf-8",
    )
    os.chmod(WORKDIR / "api_config.py", 0o600)


def _spawn(name: str, shell: str, extra: dict | None = None) -> int:
    log = open(WORKDIR / f"{name}.log", "ab")
    process = subprocess.Popen(
        ["bash", "-lc", shell],
        env=_isolated_env(extra),
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    log.close()
    return process.pid


def _cmdline(pid: int) -> str:
    try:
        raw = Path(f"/proc/{pid}/cmdline").read_bytes()
    except OSError:
        return ""
    return raw.replace(b"\0", b" ").decode("utf-8", errors="replace")


def _isolated_pids() -> list[int]:
    listed = _run(["ps", "-eo", "pid,cmd"], timeout=5)
    found = []
    for line in listed.stdout.splitlines():
        if "care_move_isolated_" not in line and "isolated_mock_fleet.py" not in line:
            continue
        if "ps -eo" in line:
            continue
        parts = line.split()
        if not parts or not parts[0].isdigit():
            continue
        found.append(int(parts[0]))
    return found


def _stop(pids: list[int]) -> None:
    _run(["docker", "rm", "-f", CONTAINER], timeout=30)
    for pid in list(pids) + _isolated_pids():
        command = _cmdline(pid)
        if not command:
            continue
        if "physical_pinky" in command or "physical_fleet_adapter" in command:
            continue
        if "isolated_mock_fleet" not in command and "care_move_isolated_" not in command:
            continue
        try:
            os.kill(pid, 15)
        except OSError:
            continue
        for _ in range(10):
            try:
                os.kill(pid, 0)
            except OSError:
                break
            time.sleep(0.1)
        else:
            try:
                os.kill(pid, 9)
            except OSError:
                pass


def _wait_http(token: str) -> bool:
    for _ in range(40):
        code, _body = _http("GET", "/tasks?limit=1", token, timeout=2)
        if code in {200, 401, 404}:
            return code == 200
        time.sleep(0.5)
    return False


def _task_body(robot: str, place: str = "MOCK_EAST") -> dict:
    return {
        "type": "robot_task_request",
        "robot": robot,
        "fleet": "isolated_mock",
        "request": {
            "category": "patrol",
            "description": {"places": [place], "rounds": 1},
            "requester": f"care-move-isolated:{robot}",
            "fleet_name": "isolated_mock",
        },
    }


def _wait_status(token: str, booking_id: str, wanted: set[str], timeout: float) -> dict:
    deadline = time.time() + timeout
    last: dict = {}
    while time.time() < deadline:
        code, body = _http("GET", f"/tasks/{booking_id}/state", token, timeout=5)
        if code == 200 and isinstance(body, dict):
            last = body
            if str(body.get("status") or "") in wanted:
                return body
        time.sleep(0.5)
    return last


def main() -> int:
    if "--stop" in sys.argv:
        pid_file = WORKDIR / "pids.json"
        pids = json.loads(pid_file.read_text()) if pid_file.is_file() else []
        _stop(pids)
        print(json.dumps({"stopped": True, "container": CONTAINER}))
        return 0
    pid_file = WORKDIR / "pids.json"
    if pid_file.is_file():
        _stop(json.loads(pid_file.read_text(encoding="utf-8")))
    problems = isolation_problems(ISOLATED_STACK)
    if problems:
        print(json.dumps({"started": False, "problems": problems}))
        return 2
    import secrets
    secret = secrets.token_urlsafe(24)
    _write_runtime_files(secret)
    before = _snapshot("before")
    (WORKDIR / "before.json").write_text(json.dumps(before, indent=2), encoding="utf-8")
    _run(["docker", "rm", "-f", CONTAINER], timeout=30)
    docker = _run([
        "docker", "run", "-d", "--name", CONTAINER, "--network", "host",
        "-e", "ROS_DOMAIN_ID=91",
        "-e", "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        "-e", "CYCLONEDDS_URI=file:///tmp/care-move-isolated/cyclonedds-lo.xml",
        "-e", "RMF_API_SERVER_CONFIG=/ws/isolated_api_config.py",
        "-v", f"{WORKDIR / 'rmf.sqlite'}:/run/isolated_rmf.db",
        "-v", f"{WORKDIR / 'api_config.py'}:/ws/isolated_api_config.py:ro",
        "-v", f"{WORKDIR / 'cyclonedds-lo.xml'}:/tmp/care-move-isolated/cyclonedds-lo.xml:ro",
        IMAGE,
        "bash", "-c", ". /opt/ros/$ROS_DISTRO/setup.bash && mkdir -p run/cache && python3 -m api_server",
    ], timeout=60)
    if docker.returncode != 0:
        print(json.dumps({"started": False, "stage": "docker", "stderr": docker.stderr[-400:]}))
        return 3
    inspect = _run(["docker", "inspect", CONTAINER, "--format", "{{json .HostConfig.Binds}} {{json .Config.Env}}"])
    mounted = inspect.stdout
    if "rmf_api_server.db" in mounted or "task_queue.sqlite" in mounted or "ROS_DOMAIN_ID=77" in mounted:
        _stop([])
        print(json.dumps({"started": False, "stage": "mount-check"}))
        return 4
    token = _jwt(secret)
    if not _wait_http(token):
        log = _run(["docker", "logs", "--tail", "40", CONTAINER])
        _stop([])
        print(json.dumps({"started": False, "stage": "api", "log": (log.stdout or log.stderr)[-500:]}))
        return 5
    source = "source /opt/ros/jazzy/setup.bash"
    pids = [
        _spawn("schedule", f"{source} && exec ros2 run rmf_traffic_ros2 rmf_traffic_schedule --ros-args -p use_sim_time:=false -r __node:=care_move_isolated_schedule"),
        _spawn("dispatcher", f"{source} && exec ros2 run rmf_task_ros2 rmf_task_dispatcher --ros-args -p server_uri:=ws://127.0.0.1:8001/_internal -r __node:=care_move_isolated_dispatcher"),
    ]
    time.sleep(3)
    pids.append(_spawn(
        "fleet",
        f"{source} && exec python3 {ROOT / 'scripts' / 'isolated_mock_fleet.py'}",
        {"PYTHONPATH": str(ROOT), "MOCK_HOLD_SEC": "0.4"},
    ))
    (WORKDIR / "pids.json").write_text(json.dumps(pids), encoding="utf-8")
    fleet_ready = False
    for _ in range(30):
        code, body = _http("GET", "/fleets/isolated_mock/state", token)
        robots = body.get("robots") if isinstance(body, dict) else None
        if isinstance(robots, dict) and {"mock_pinky1", "mock_pinky2", "mock_pinky3"} <= set(robots):
            fleet_ready = True
            break
        time.sleep(1)
    results: dict = {"fleet_ready": fleet_ready, "tasks": []}
    if not fleet_ready:
        after = _snapshot("after-failed-fleet")
        results["node_leak_77"] = before["nodes_77"] != after["nodes_77"]
        results["node_leak_24"] = before["nodes_24"] != after["nodes_24"]
        (WORKDIR / "result.json").write_text(json.dumps({"verified": False, **results}, indent=2), encoding="utf-8")
        print(json.dumps({"verified": False, "stage": "fleet", **results}))
        return 6

    def submit(robot: str) -> dict:
        code, body = _http("POST", "/tasks/robot_task", token, _task_body(robot), timeout=12)
        booking_id = None
        if isinstance(body, dict):
            booking = body.get("booking") if isinstance(body.get("booking"), dict) else body
            booking_id = body.get("id") or (booking.get("id") if isinstance(booking, dict) else None)
            state = body.get("state") if isinstance(body.get("state"), dict) else None
            if state and isinstance(state.get("booking"), dict):
                booking_id = state["booking"].get("id") or booking_id
        record = {"robot": robot, "http": code, "booking_id": booking_id}
        if booking_id:
            final = _wait_status(token, booking_id, {"completed"}, 25)
            record["status"] = final.get("status")
        results["tasks"].append(record)
        return record

    nodes_91 = _nodes("91", "rmw_cyclonedds_cpp", "file:///tmp/care-move-isolated/cyclonedds-lo.xml")
    first = submit("mock_pinky1")
    second = submit("mock_pinky2")
    queued_a = submit("mock_pinky1")
    queued_b = submit("mock_pinky1")
    try:
        os.kill(pids[-1], 15)
    except OSError:
        pass
    time.sleep(1)
    held = _spawn(
        "fleet-hold",
        f"{source} && exec python3 {ROOT / 'scripts' / 'isolated_mock_fleet.py'}",
        {"PYTHONPATH": str(ROOT), "MOCK_HOLD_SEC": "20"},
    )
    pids.append(held)
    (WORKDIR / "pids.json").write_text(json.dumps(pids), encoding="utf-8")
    time.sleep(4)
    code, body = _http("POST", "/tasks/robot_task", token, _task_body("mock_pinky1", "MOCK_EAST"), timeout=12)
    held_id = None
    if isinstance(body, dict):
        state = body.get("state") if isinstance(body.get("state"), dict) else {}
        booking = state.get("booking") if isinstance(state.get("booking"), dict) else body.get("booking")
        held_id = body.get("id") or (booking.get("id") if isinstance(booking, dict) else None)
    before_kill = {}
    if held_id:
        deadline = time.time() + 12
        while time.time() < deadline:
            code, body = _http("GET", f"/tasks/{held_id}/state", token, timeout=5)
            if code == 200 and isinstance(body, dict):
                before_kill = body
            log_path = WORKDIR / "fleet-hold.log"
            log_text = log_path.read_text(encoding="utf-8", errors="replace") if log_path.is_file() else ""
            if before_kill.get("status") in {"queued", "standby", "underway"} and "Beginning next task" in log_text:
                break
            if before_kill.get("status") == "completed":
                break
            time.sleep(0.3)
    try:
        os.kill(held, 15)
    except OSError:
        pass
    time.sleep(2)
    after_kill = _wait_status(token, held_id, {"completed"}, 3) if held_id else {}
    replay_pid = _spawn(
        "fleet-replay-check",
        f"{source} && exec python3 {ROOT / 'scripts' / 'isolated_mock_fleet.py'}",
        {"PYTHONPATH": str(ROOT), "MOCK_HOLD_SEC": "0.2"},
    )
    pids.append(replay_pid)
    (WORKDIR / "pids.json").write_text(json.dumps(pids), encoding="utf-8")
    time.sleep(6)
    replay_state = _wait_status(token, held_id, {"completed"}, 2) if held_id else {}
    replay_log = (WORKDIR / "fleet-replay-check.log").read_text(encoding="utf-8", errors="replace") if (WORKDIR / "fleet-replay-check.log").is_file() else ""
    try:
        os.kill(replay_pid, 15)
    except OSError:
        pass
    cancel_code, cancel_body = (0, {})
    if held_id:
        cancel_code, cancel_body = _http(
            "POST", "/tasks/cancel_task", token,
            {"type": "cancel_task_request", "task_id": held_id},
            timeout=8,
        )
    preserved = _wait_status(token, held_id, set(), 2) if held_id else {}
    after = _snapshot("after")
    (WORKDIR / "after.json").write_text(json.dumps(after, indent=2), encoding="utf-8")
    verified = (
        first.get("status") == "completed"
        and second.get("status") == "completed"
        and queued_a.get("booking_id")
        and queued_b.get("booking_id")
        and queued_a.get("booking_id") != queued_b.get("booking_id")
        and before["production_db"] == after["production_db"]
        and before["queue"] == after["queue"]
        and before["booking"].get("status") == after["booking"].get("status") == "standby"
        and before["booking"].get("cancellation") == after["booking"].get("cancellation")
        and before["gateway"].get("motion_commands") == after["gateway"].get("motion_commands") == 0
        and before["gateway"].get("mode") == after["gateway"].get("mode") == "PHYSICAL_DRY_RUN"
        and after_kill.get("status") in {"queued", "standby", "underway", "blocked"}
        and replay_state.get("status") == after_kill.get("status")
        and "mock_navigation_finished" not in replay_log
        and cancel_code >= 400
        and preserved.get("status") == after_kill.get("status")
        and before["nodes_77"] == after["nodes_77"]
        and before["nodes_24"] == after["nodes_24"]
        and any("isolated_mock" in line or "care_move_isolated" in line for line in nodes_91)
    )
    report = {
        "environment_id": "rmf-b-domain91",
        "verified": verified,
        "mock_robots": ["mock_pinky1", "mock_pinky2", "mock_pinky3"],
        "physical_gateway": False,
        "tasks": results["tasks"],
        "held_booking": held_id,
        "held_before_kill": before_kill.get("status"),
        "held_after_kill": after_kill.get("status"),
        "replay_status": replay_state.get("status"),
        "replay_log_finished": "mock_navigation_finished" in replay_log,
        "cancel_http": cancel_code,
        "cancel_body": cancel_body if isinstance(cancel_body, dict) else str(cancel_body)[:180],
        "preserved_status": preserved.get("status"),
        "production_db_unchanged": before["production_db"] == after["production_db"],
        "queue_unchanged": before["queue"] == after["queue"],
        "booking_status": after["booking"].get("status"),
        "gateway_motion": after["gateway"].get("motion_commands"),
        "domain77_unchanged": before["nodes_77"] == after["nodes_77"],
        "domain24_unchanged": before["nodes_24"] == after["nodes_24"],
        "domain91_nodes": nodes_91,
    }
    (WORKDIR / "result.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps({key: report[key] for key in report if key != "cancel_body"}))
    return 0 if verified else 7


if __name__ == "__main__":
    raise SystemExit(main())
