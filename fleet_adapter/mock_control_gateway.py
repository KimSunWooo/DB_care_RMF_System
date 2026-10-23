"""Mock gateway that checks a lease before any motion. It never opens a motor or port 17453."""
from __future__ import annotations

import hmac
import json
import socket
import threading
import time

from fleet_adapter.control_lease import LeaseAuthority

FORBIDDEN_PORTS = frozenset({17453, 17442, 8000, 8001, 8011})


class MockControlGateway:
    def __init__(self, authority: LeaseAuthority, *, robot_id: str = "mock_pinky1") -> None:
        self.authority = authority
        self.robot_id = robot_id
        self.mode = "PHYSICAL_DRY_RUN"
        self.phase = "idle"
        self.motion_commands = 0
        self.session_id = ""
        self.fault = ""
        self._lock = threading.Lock()
        self.after_check = None
        self._started = time.monotonic()
        self.events = {
            "lease_expired_at": None,
            "stop_requested_at": None,
            "last_motion_command_at": None,
            "goal_invalidated_at": None,
        }

    def handle(self, payload: dict) -> dict:
        self.expire_motion()
        op = str(payload.get("op") or "")
        if op == "poll":
            return self.status()
        if op == "issue_lease":
            return self._issue(payload)
        if op == "revoke":
            return self._revoke(payload)
        if op == "renew":
            return self._renew(payload)
        if op in {"arm", "prepare", "go"}:
            return self._motion(op, payload)
        if op in {"stop", "disarm", "cancel"}:
            return self._stop(payload)
        return {"ok": False, "reason": "unknown op", "motion_commands": self.motion_commands}

    def status(self) -> dict:
        with self._lock:
            return self._status_unlocked()

    def _status_unlocked(self) -> dict:
        return {
            "ok": True,
            "mode": self.mode,
            "goal_phase": self.phase,
            "motion_commands": self.motion_commands,
            "session_id": self.session_id,
            "gateway_boot_id": self.authority.boot_id,
            "control_generation": self.authority.generation,
            "robot_id": self.robot_id,
            "lease_enforced": True,
            "fault": self.fault,
            "cmd_vel": False,
            "lease_watch": dict(self.events),
        }

    def expire_motion(self, now: float | None = None) -> dict:
        """Stop an active goal when its lease is no longer valid. Does not count as a new command."""
        stamp = time.time() if now is None else now
        with self._lock:
            if self.phase == "moving" and not self.authority.current_valid(self.robot_id, stamp):
                observed = time.time()
                self.phase = "idle"
                self.mode = "PHYSICAL_DRY_RUN"
                self.fault = "lease expired"
                self.session_id = ""
                self.events["lease_expired_at"] = observed
                self.events["stop_requested_at"] = observed
                self.events["goal_invalidated_at"] = observed
            return self._status_unlocked()

    def reboot(self) -> dict:
        with self._lock:
            self.authority.new_boot()
            self.mode = "PHYSICAL_DRY_RUN"
            self.phase = "idle"
            self.session_id = ""
            self.fault = "gateway restarted"
            return self._status_unlocked()

    def _issue(self, payload: dict) -> dict:
        presented = str(payload.get("issuer_secret") or "")
        expected = self.authority.issuer_secret
        if len(presented) != len(expected) or not hmac.compare_digest(presented, expected):
            return {"ok": False, "reason": "lease 발급 권한이 없습니다", "motion_commands": self.motion_commands}
        robot_id = str(payload.get("robot_id") or self.robot_id)
        if robot_id != self.robot_id:
            return {"ok": False, "reason": "이 mock gateway의 로봇이 아닙니다", "motion_commands": self.motion_commands}
        session_id = str(payload.get("session_id") or "")
        if not session_id:
            return {"ok": False, "reason": "session이 없습니다", "motion_commands": self.motion_commands}
        issued = self.authority.issue(
            str(payload.get("environment_id") or ""),
            robot_id,
            session_id,
            ttl_s=float(payload.get("ttl_s") or 30.0),
            now=payload.get("now"),
        )
        with self._lock:
            if self.phase == "moving":
                self.phase = "idle"
                self.mode = "PHYSICAL_DRY_RUN"
            self.session_id = session_id
            self.fault = ""
        issued["motion_commands"] = self.motion_commands
        return issued

    def _revoke(self, payload: dict) -> dict:
        presented = str(payload.get("issuer_secret") or "")
        expected = self.authority.issuer_secret
        if len(presented) != len(expected) or not hmac.compare_digest(presented, expected):
            return {"ok": False, "reason": "lease 회수 권한이 없습니다", "motion_commands": self.motion_commands}
        result = self.authority.revoke(str(payload.get("robot_id") or self.robot_id))
        with self._lock:
            self.phase = "idle"
            self.mode = "PHYSICAL_DRY_RUN"
            self.session_id = ""
            self.fault = "lease revoked"
            result["motion_commands"] = self.motion_commands
            result["goal_phase"] = self.phase
        return result

    def _motion(self, op: str, payload: dict) -> dict:
        robot_id = str(payload.get("robot_id") or "")
        session_id = str(payload.get("session_id") or "")
        token = str(payload.get("lease") or "")
        now = payload.get("now")
        allowed, reason = self.authority.allows(
            token, robot_id, session_id, now, claimed_environment=str(payload.get("environment_id") or ""),
        )
        if not allowed:
            return {"ok": False, "refused": True, "reason": reason, "motion_commands": self.motion_commands}
        if self.after_check is not None:
            self.after_check()
        with self._lock:
            allowed, reason = self.authority.allows(
                token, robot_id, session_id, now,
                claimed_environment=str(payload.get("environment_id") or ""),
            )
            if not allowed:
                return {"ok": False, "refused": True, "reason": reason, "motion_commands": self.motion_commands,
                        "goal_phase": self.phase}
            if robot_id != self.robot_id:
                return {"ok": False, "refused": True, "reason": "robot이 다릅니다", "motion_commands": self.motion_commands}
            if op == "arm":
                self.mode = "PHYSICAL_TEST"
                return {"ok": True, "mode": self.mode, "motion_commands": self.motion_commands}
            if op == "prepare":
                if self.mode != "PHYSICAL_TEST":
                    return {"ok": False, "reason": "ARM 전이 아닙니다", "motion_commands": self.motion_commands}
                return {"ok": True, "motion_commands": self.motion_commands, "session_id": self.session_id}
            if self.mode != "PHYSICAL_TEST" or self.phase == "moving":
                return {"ok": False, "refused": True, "reason": "GO를 받을 상태가 아닙니다",
                        "motion_commands": self.motion_commands}
            self.motion_commands += 1
            self.phase = "moving"
            self.events["last_motion_command_at"] = time.time()
            return {"ok": True, "motion_commands": self.motion_commands, "goal_phase": self.phase,
                    "session_id": self.session_id}

    def _renew(self, payload: dict) -> dict:
        renewed = self.authority.renew(
            str(payload.get("lease") or ""),
            str(payload.get("robot_id") or ""),
            str(payload.get("session_id") or ""),
            str(payload.get("environment_id") or ""),
            ttl_s=float(payload.get("ttl_s") or 0.6),
            now=payload.get("now"),
        )
        renewed["motion_commands"] = self.motion_commands
        renewed["goal_phase"] = self.phase
        return renewed

    def _stop(self, payload: dict) -> dict:
        allowed, reason = self.authority.stop_allowed(payload, str(payload.get("robot_id") or self.robot_id))
        if not allowed:
            return {"ok": False, "refused": True, "reason": reason, "motion_commands": self.motion_commands}
        with self._lock:
            self.phase = "idle"
            self.mode = "PHYSICAL_DRY_RUN"
            self.fault = "stopped"
            return {"ok": True, "mode": self.mode, "goal_phase": self.phase, "motion_commands": self.motion_commands}


def serve(gateway: MockControlGateway, port: int) -> socket.socket:
    if port in FORBIDDEN_PORTS:
        raise RuntimeError(f"refusing to bind production port {port}")
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("127.0.0.1", port))
    server.listen(8)
    server.settimeout(0.2)
    stop = threading.Event()

    def loop() -> None:
        while not stop.is_set():
            try:
                conn, _addr = server.accept()
            except TimeoutError:
                gateway.expire_motion()
                continue
            except OSError:
                return
            with conn:
                conn.settimeout(2)
                try:
                    data = b""
                    while b"\n" not in data:
                        chunk = conn.recv(65536)
                        if not chunk:
                            break
                        data += chunk
                    if not data:
                        continue
                    payload = json.loads(data.split(b"\n", 1)[0])
                    reply = gateway.handle(payload if isinstance(payload, dict) else {})
                except (json.JSONDecodeError, OSError, ValueError):
                    reply = {"ok": False, "reason": "bad request"}
                conn.sendall((json.dumps(reply) + "\n").encode("utf-8"))

    def watch() -> None:
        while not stop.wait(0.05):
            gateway.expire_motion()

    thread = threading.Thread(target=loop, daemon=True)
    watcher = threading.Thread(target=watch, daemon=True)
    thread.start()
    watcher.start()
    return _GatewayServer(server, stop, thread, watcher)


class _GatewayServer:
    def __init__(self, sock: socket.socket, stop: threading.Event, thread: threading.Thread,
                 watcher: threading.Thread) -> None:
        self._sock = sock
        self._stop = stop
        self._thread = thread
        self._watcher = watcher

    def close(self) -> None:
        self._stop.set()
        try:
            self._sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self._sock.close()
        self._thread.join(timeout=2)
        self._watcher.join(timeout=2)


def request(port: int, payload: dict) -> dict:
    with socket.create_connection(("127.0.0.1", port), 2) as sock:
        sock.settimeout(2)
        sock.sendall((json.dumps(payload) + "\n").encode("utf-8"))
        data = b""
        while b"\n" not in data:
            chunk = sock.recv(65536)
            if not chunk:
                break
            data += chunk
    body = json.loads(data.split(b"\n", 1)[0])
    if not isinstance(body, dict):
        raise RuntimeError("mock gateway reply is not an object")
    return body
