"""Mock fleet for ROS domain 91. Completes navigation in memory and never opens a motor."""
from __future__ import annotations

import json
import os
import signal
import sys
import threading
import time
from datetime import timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

MAP = "isolated_room"
FLEET = "isolated_mock"
ROBOTS = ("mock_pinky1", "mock_pinky2", "mock_pinky3")
SERVER = "ws://127.0.0.1:8001/_internal"


def _control_hold(robot_id: str, hold: float) -> bool:
    """Renew a mock-gateway lease while the task is underway. No port means the old in-memory hold."""
    port = os.environ.get("CARE_MOVE_MOCK_GATEWAY_PORT", "")
    if not port:
        time.sleep(hold)
        return True
    if port in {"17453", "17442", "8000", "8001", "8011"}:
        print(json.dumps({"event": "gateway_port_refused", "cmd_vel": False}), flush=True)
        return False
    token = os.environ.get("CARE_MOVE_MOCK_LEASE", "")
    environment = os.environ.get("CARE_MOVE_MOCK_ENVIRONMENT", "")
    session = os.environ.get("CARE_MOVE_MOCK_SESSION", "")
    if not token or not environment or not session:
        print(json.dumps({"event": "lease_missing", "robot": robot_id, "cmd_vel": False}), flush=True)
        return False
    from fleet_adapter.mock_control_gateway import request

    base = {"lease": token, "environment_id": environment, "robot_id": robot_id, "session_id": session}
    if not request(int(port), {**base, "op": "arm"}).get("ok"):
        print(json.dumps({"event": "arm_rejected", "robot": robot_id, "cmd_vel": False}), flush=True)
        return False
    if not request(int(port), {**base, "op": "go"}).get("ok"):
        print(json.dumps({"event": "go_rejected", "robot": robot_id, "cmd_vel": False}), flush=True)
        return False
    deadline = time.monotonic() + hold
    while time.monotonic() < deadline:
        renewed = request(int(port), {**base, "op": "renew", "ttl_s": 0.6})
        if not renewed.get("ok"):
            print(json.dumps({"event": "renew_failed", "robot": robot_id, "cmd_vel": False}), flush=True)
            return False
        base["lease"] = renewed.get("lease") or base["lease"]
        time.sleep(0.15)
    return True


def _refuse(reason: str) -> int:
    print(json.dumps({"started": False, "reason": reason}), flush=True)
    return 2


def main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") != "91":
        return _refuse("adapter refuses every domain except 91")
    if os.environ.get("RMW_IMPLEMENTATION") != "rmw_cyclonedds_cpp":
        return _refuse("adapter requires cyclonedds")
    uri = os.environ.get("CYCLONEDDS_URI", "")
    if "care-move-isolated" not in uri:
        return _refuse("adapter requires the isolated cyclonedds file")
    if os.environ.get("ISOLATED_RMF_SERVER_URI") != SERVER:
        return _refuse("server uri must be the isolated API on port 8001")
    text = Path(uri.removeprefix("file://")).read_text(encoding="utf-8")
    if 'id="91"' not in text or "127.0.0.1" not in text:
        return _refuse("cyclonedds file is not pinned to domain 91 on loopback")
    if "17453" in text or "/cmd_vel" in text:
        return _refuse("isolated dds file mentions the physical gateway")
    hold = float(os.environ.get("MOCK_HOLD_SEC", "0.4"))
    import rmf_adapter
    import rmf_adapter.easy_full_control as easy

    rmf_adapter.init_rclcpp()
    configuration = easy.FleetConfiguration.from_config_files(
        str(ROOT / "config" / "rmf" / "isolated_mock" / "fleet.yaml"),
        str(ROOT / "config" / "rmf" / "isolated_mock" / "nav.yaml"),
        SERVER,
    )
    if configuration is None:
        return _refuse("fleet configuration rejected")
    configuration.server_uri = SERVER
    adapter = rmf_adapter.Adapter.make("isolated_mock_fleet_adapter", wait_time=timedelta(seconds=8))
    if adapter is None:
        print(json.dumps({"started": False, "reason": "traffic schedule is not available on domain 91", "robots_added": 0}), flush=True)
        return 6
    adapter.start()
    fleet = adapter.add_easy_fleet(configuration)
    guard = threading.Lock()
    current: dict[str, dict] = {robot_id: {} for robot_id in ROBOTS}

    def make_navigate(robot_id: str):
        def navigate(destination, execution) -> None:
            def work() -> None:
                raw = destination.position
                position = raw() if callable(raw) else raw
                pose = [float(position[0]), float(position[1]), float(position[2] if len(position) > 2 else 0.0)]
                if not _control_hold(robot_id, hold):
                    print(json.dumps({"event": "navigation_blocked", "robot": robot_id, "cmd_vel": False}), flush=True)
                    return
                with guard:
                    current[robot_id] = {"execution": execution, "pose": pose, "finished": False}
                try:
                    if execution.okay():
                        execution.finished()
                        with guard:
                            current[robot_id]["finished"] = True
                        print(json.dumps({"event": "mock_navigation_finished", "robot": robot_id, "pose": pose, "cmd_vel": False}), flush=True)
                except Exception as exc:
                    print(json.dumps({"event": "finish_error", "robot": robot_id, "reason": str(exc)}), flush=True)
            threading.Thread(target=work, daemon=True).start()
        return navigate

    def stop(_activity) -> None:
        print(json.dumps({"event": "stop", "cmd_vel": False}), flush=True)

    def action_executor(category, _description, execution) -> None:
        print(json.dumps({"event": "action_refused", "category": str(category)}), flush=True)
        del execution

    handles = {}
    for robot_id in ROBOTS:
        callbacks = easy.RobotCallbacks(make_navigate(robot_id), stop, action_executor)
        state = easy.RobotState(MAP, [0.0, 0.0, 0.0], 0.95)
        handle = fleet.add_robot(robot_id, state, easy.RobotConfiguration(["MOCK_HOME"]), callbacks)
        if handle is not None:
            handle.set_max_merge_waypoint_distance(0.3)
            handle.set_max_merge_lane_distance(0.5)
            handle.set_min_lane_length(1e-8)
        handles[robot_id] = handle
    added = [robot_id for robot_id, handle in handles.items() if handle is not None]
    print(json.dumps({
        "started": len(added) == len(ROBOTS),
        "robots_added": len(added),
        "robots": added,
        "fleet": FLEET,
        "server_uri": SERVER,
        "rmf_domain": 91,
        "cmd_vel": False,
        "physical_gateway": False,
    }), flush=True)
    if len(added) != len(ROBOTS):
        adapter.stop()
        return 7
    stopping = False

    def _stop_process(_signum, _frame) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, _stop_process)
    signal.signal(signal.SIGINT, _stop_process)
    while not stopping:
        time.sleep(0.3)
        for robot_id, handle in handles.items():
            with guard:
                pose = list(current[robot_id].get("pose") or [0.0, 0.0, 0.0])
                execution = current[robot_id].get("execution")
            activity = None
            if execution is not None:
                try:
                    if execution.okay():
                        activity = execution.identifier() if callable(execution.identifier) else execution.identifier
                except Exception:
                    activity = None
            try:
                handle.update(easy.RobotState(MAP, pose, 0.95), activity)
            except Exception as exc:
                print(json.dumps({"event": "update_error", "robot": robot_id, "reason": str(exc)}), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
