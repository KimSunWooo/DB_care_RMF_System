"""Kill an isolated fleet while a mock goal is underway. Never opens port 17453."""
from __future__ import annotations

import importlib.util
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from fleet_adapter.control_lease import LeaseAuthority
from fleet_adapter.mock_control_gateway import MockControlGateway, request, serve

_spec = importlib.util.spec_from_file_location("isolated_rmf_stack", ROOT / "scripts" / "isolated_rmf_stack.py")
_stack = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_stack)
_http = _stack._http
_isolated_env = _stack._isolated_env
_jwt = _stack._jwt
_task_body = _stack._task_body

PORT = 17491
ROBOT = "mock_pinky1"
ENVIRONMENT = "rmf-b-domain91"
SESSION = "underway-lease"
RESULT = Path("/tmp/care-move-isolated/underway_lease_result.json")
LOG = Path("/tmp/care-move-isolated/underway_fleet.log")


def _secret() -> str:
    text = Path("/tmp/care-move-isolated/api_config.py").read_text(encoding="utf-8")
    for line in text.splitlines():
        if "jwt_secret" in line and "=" in line:
            return line.split("=", 1)[1].strip().strip("'").strip('"')
    raise RuntimeError("isolated api secret is missing")


def _spawn(lease: str) -> int:
    env = _isolated_env({
        "PYTHONPATH": str(ROOT),
        "MOCK_HOLD_SEC": "4",
        "CARE_MOVE_MOCK_GATEWAY_PORT": str(PORT),
        "CARE_MOVE_MOCK_LEASE": lease,
        "CARE_MOVE_MOCK_ENVIRONMENT": ENVIRONMENT,
        "CARE_MOVE_MOCK_SESSION": SESSION,
    })
    LOG.parent.mkdir(parents=True, exist_ok=True)
    log = open(LOG, "wb")
    process = subprocess.Popen(
        ["bash", "-lc", f"source /opt/ros/jazzy/setup.bash && exec python3 {ROOT / 'scripts' / 'isolated_mock_fleet.py'}"],
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    log.close()
    return process.pid


def _kill(pid: int) -> None:
    try:
        os.killpg(pid, signal.SIGKILL)
    except OSError:
        try:
            os.kill(pid, signal.SIGKILL)
        except OSError:
            return


def _started(pid: int) -> bool:
    deadline = time.time() + 25
    while time.time() < deadline:
        if not _alive(pid):
            return False
        text = LOG.read_text(encoding="utf-8", errors="replace") if LOG.is_file() else ""
        if '"started": true' in text:
            return True
        if "Segmentation fault" in text or '"started": false' in text:
            return False
        time.sleep(0.3)
    return False


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
    except OSError:
        return False
    return True


def _booking(token: str, booking_id: str) -> dict:
    code, body = _http("GET", f"/tasks/{booking_id}/state", token, timeout=5)
    status = body.get("status") if isinstance(body, dict) else None
    return {"http": code, "status": status}


def main() -> int:
    authority = LeaseAuthority("underway-lease-secret")
    gateway = MockControlGateway(authority, robot_id=ROBOT)
    server = serve(gateway, PORT)
    pid = 0
    replay = 0
    report: dict = {"physical_gateway": False, "cmd_vel": False}
    try:
        issued = request(PORT, {
            "op": "issue_lease",
            "issuer_secret": authority.issuer_secret,
            "environment_id": ENVIRONMENT,
            "robot_id": ROBOT,
            "session_id": SESSION,
            "ttl_s": 20,
        })
        report["generation"] = issued.get("control_generation")
        report["boot_id"] = authority.boot_id
        pid = _spawn(str(issued["lease"]))
        report["fleet_pid"] = pid
        report["fleet_started"] = _started(pid)
        if not report["fleet_started"]:
            tail = LOG.read_text(encoding="utf-8", errors="replace")[-400:] if LOG.is_file() else ""
            report["fleet_log_tail"] = tail
            report["result"] = "BLOCKED"
            return 2
        token = _jwt(_secret())
        code, body = _http("POST", "/tasks/robot_task", token, _task_body(ROBOT, "MOCK_EAST"), timeout=12)
        booking = ""
        if isinstance(body, dict):
            state = body.get("state") if isinstance(body.get("state"), dict) else body
            booking = str((state.get("booking") or {}).get("id") or body.get("id") or "")
            report["submit_status"] = state.get("status")
        report["submit_http"] = code
        report["booking_id"] = booking
        deadline = time.time() + 20
        while time.time() < deadline:
            status = request(PORT, {"op": "poll"})
            if status.get("goal_phase") == "moving":
                break
            time.sleep(0.1)
        else:
            report["result"] = "BLOCKED"
            report["gateway_before_kill"] = request(PORT, {"op": "poll"})
            return 3
        before = request(PORT, {"op": "poll"})
        report["motion_before_kill"] = before["motion_commands"]
        report["phase_before_kill"] = before["goal_phase"]
        _kill(pid)
        pid = 0
        stopped = {}
        deadline = time.time() + 3
        while time.time() < deadline:
            stopped = request(PORT, {"op": "poll"})
            if stopped.get("goal_phase") == "idle":
                break
            time.sleep(0.05)
        report["gateway_after_kill"] = {
            "mode": stopped.get("mode"),
            "goal_phase": stopped.get("goal_phase"),
            "motion_commands": stopped.get("motion_commands"),
            "fault": stopped.get("fault"),
            "lease_watch": stopped.get("lease_watch"),
        }
        if booking:
            report["task_after_kill"] = _booking(token, booking)
        # A replacement fleet without the old lease must not add motion or finish the old goal.
        replay = _spawn_without_lease()
        time.sleep(3)
        after_replay = request(PORT, {"op": "poll"})
        report["motion_after_replay"] = after_replay["motion_commands"]
        report["phase_after_replay"] = after_replay["goal_phase"]
        if booking:
            report["task_after_replay"] = _booking(token, booking)
        same_motion = after_replay["motion_commands"] == before["motion_commands"]
        stopped_clean = stopped.get("goal_phase") == "idle" and stopped.get("fault") == "lease expired"
        task_status = (report.get("task_after_replay") or {}).get("status")
        preserved = task_status != "completed"
        report["result"] = "PASS" if same_motion and stopped_clean and preserved else "FAIL"
        return 0 if report["result"] == "PASS" else 4
    finally:
        if pid:
            _kill(pid)
        if replay:
            _kill(replay)
        server.close()
        RESULT.write_text(json.dumps(report, indent=2), encoding="utf-8")
        print(json.dumps({k: report.get(k) for k in ("result", "booking_id", "phase_before_kill", "motion_before_kill", "task_after_kill", "task_after_replay", "motion_after_replay")}), flush=True)


def _spawn_without_lease() -> int:
    env = _isolated_env({"PYTHONPATH": str(ROOT), "MOCK_HOLD_SEC": "0.2"})
    log = open(LOG.with_name("underway_replay.log"), "wb")
    process = subprocess.Popen(
        ["bash", "-lc", f"source /opt/ros/jazzy/setup.bash && exec python3 {ROOT / 'scripts' / 'isolated_mock_fleet.py'}"],
        env=env,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    log.close()
    return process.pid


if __name__ == "__main__":
    raise SystemExit(main())
