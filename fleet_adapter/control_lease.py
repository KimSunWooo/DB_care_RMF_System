"""Signed control leases. Callers cannot mint a generation by naming an environment."""
from __future__ import annotations

import hashlib
import hmac
import json
import os
import threading
import time
import uuid
from pathlib import Path

MOTION_OPS = frozenset({"arm", "prepare", "go"})
_ENFORCED_AUTHORITY: LeaseAuthority | None = None


def bind_enforced_authority(authority: LeaseAuthority | None) -> None:
    global _ENFORCED_AUTHORITY
    _ENFORCED_AUTHORITY = authority


def _b64(raw: bytes) -> str:
    import base64
    return base64.urlsafe_b64encode(raw).rstrip(b"=").decode("ascii")


def _unb64(text: str) -> bytes:
    import base64
    pad = "=" * (-len(text) % 4)
    return base64.urlsafe_b64decode(text + pad)


class LeaseAuthority:
    """One gateway boot. A new boot id invalidates every previously signed lease."""

    def __init__(self, secret: str, generation_path: Path | None = None) -> None:
        self._secret = secret.encode("utf-8")
        self._issuer_secret = hashlib.sha256(b"issuer:" + self._secret).hexdigest()
        self._emergency = hashlib.sha256(b"stop:" + self._secret).hexdigest()
        self._path = generation_path
        self._lock = threading.Lock()
        self.boot_id = uuid.uuid4().hex
        self._generation = self._load_generation()
        self._active: dict[str, dict] = {}

    @property
    def issuer_secret(self) -> str:
        return self._issuer_secret

    @property
    def emergency_stop(self) -> str:
        return self._emergency

    @property
    def generation(self) -> int:
        return self._generation

    def new_boot(self) -> str:
        """Restart. Old tokens die with the previous boot id, and generation does not go backwards."""
        with self._lock:
            self.boot_id = uuid.uuid4().hex
            self._generation += 1
            self._store_generation()
            self._active.clear()
            return self.boot_id

    def issue(self, environment_id: str, robot_id: str, session_id: str, *,
              ttl_s: float = 30.0, now: float | None = None) -> dict:
        stamp = time.time() if now is None else now
        with self._lock:
            self._generation += 1
            self._store_generation()
            lease_id = uuid.uuid4().hex
            record = {
                "environment_id": environment_id,
                "robot_id": robot_id,
                "lease_id": lease_id,
                "control_generation": self._generation,
                "gateway_boot_id": self.boot_id,
                "session_id": session_id,
                "issued_at": stamp,
                "expires_at": stamp + ttl_s,
                "lease_status": "active",
            }
            self._active[robot_id] = dict(record)
            token = self._sign(record)
        return {"ok": True, "lease": token, **record}

    def renew(self, token: str, robot_id: str, session_id: str, claimed_environment: str, *,
              ttl_s: float = 0.6, now: float | None = None) -> dict:
        """Extend the same generation. A dead holder cannot renew, so the old token expires."""
        payload = self._open(token)
        if payload is None:
            return {"ok": False, "reason": "lease가 없거나 변조되었습니다"}
        stamp = time.time() if now is None else now
        with self._lock:
            active = self._active.get(robot_id)
            if not active or payload.get("lease_id") != active.get("lease_id"):
                return {"ok": False, "reason": "활성 lease와 다릅니다"}
            if int(payload.get("control_generation") or -1) != int(active["control_generation"]):
                return {"ok": False, "reason": "이전 generation입니다"}
            if payload.get("gateway_boot_id") != self.boot_id:
                return {"ok": False, "reason": "gateway boot id가 다릅니다"}
            if payload.get("robot_id") != robot_id or payload.get("session_id") != session_id:
                return {"ok": False, "reason": "robot 또는 session이 다릅니다"}
            if claimed_environment != active.get("environment_id"):
                return {"ok": False, "reason": "environment가 다릅니다"}
            if stamp > float(active["expires_at"]):
                return {"ok": False, "reason": "lease가 만료되었습니다"}
            updated = dict(active)
            updated["issued_at"] = stamp
            updated["expires_at"] = stamp + ttl_s
            self._active[robot_id] = updated
            signed = self._sign(updated)
        return {
            "ok": True,
            "lease": signed,
            "expires_at": updated["expires_at"],
            "control_generation": updated["control_generation"],
            "lease_id": updated["lease_id"],
        }

    def revoke(self, robot_id: str) -> dict:
        with self._lock:
            self._generation += 1
            self._store_generation()
            previous = self._active.pop(robot_id, None)
        return {"ok": True, "revoked": bool(previous), "control_generation": self._generation}

    def allows(self, token: str, robot_id: str, session_id: str, now: float | None = None,
               claimed_environment: str = "") -> tuple[bool, str]:
        payload = self._open(token)
        if payload is None:
            return False, "lease가 없거나 변조되었습니다"
        if not claimed_environment or claimed_environment != payload.get("environment_id"):
            return False, "environment가 다릅니다"
        stamp = time.time() if now is None else now
        with self._lock:
            active = self._active.get(robot_id)
            if not active or payload.get("lease_id") != active.get("lease_id"):
                return False, "활성 lease와 다릅니다"
            if int(payload.get("control_generation") or -1) != int(active["control_generation"]):
                return False, "이전 generation입니다"
            if payload.get("gateway_boot_id") != self.boot_id:
                return False, "gateway boot id가 다릅니다"
            if payload.get("robot_id") != robot_id or payload.get("session_id") != session_id:
                return False, "robot 또는 session이 다릅니다"
            if payload.get("session_id") != active.get("session_id"):
                return False, "session이 활성 제어권과 다릅니다"
            if payload.get("environment_id") != active.get("environment_id"):
                return False, "environment가 다릅니다"
            if stamp > float(payload.get("expires_at") or 0):
                return False, "lease가 만료되었습니다"
            return True, ""

    def stop_allowed(self, payload: dict, robot_id: str) -> tuple[bool, str]:
        presented = str(payload.get("emergency_stop") or "")
        if presented and hmac.compare_digest(presented, self._emergency):
            return True, ""
        token = str(payload.get("lease") or "")
        opened = self._open(token)
        if opened is None:
            return False, "정지 자격 증명이 없습니다"
        with self._lock:
            active = self._active.get(robot_id)
            if active and opened.get("lease_id") == active.get("lease_id") and opened.get("robot_id") == robot_id:
                return True, ""
        return False, "이전 환경의 lease로 현재 제어권을 정지할 수 없습니다"

    def current_valid(self, robot_id: str, now: float | None = None) -> bool:
        stamp = time.time() if now is None else now
        with self._lock:
            active = self._active.get(robot_id)
            if not active:
                return False
            return stamp <= float(active["expires_at"]) and active.get("gateway_boot_id") == self.boot_id

    def _sign(self, record: dict) -> str:
        body = _b64(json.dumps(record, sort_keys=True, separators=(",", ":")).encode("utf-8"))
        digest = hmac.new(self._secret, body.encode("ascii"), hashlib.sha256).hexdigest()
        return body + "." + digest

    def _open(self, token: str) -> dict | None:
        if not token or "." not in token:
            return None
        body, digest = token.rsplit(".", 1)
        expected = hmac.new(self._secret, body.encode("ascii"), hashlib.sha256).hexdigest()
        if not hmac.compare_digest(digest, expected):
            return None
        try:
            payload = json.loads(_unb64(body))
        except (ValueError, json.JSONDecodeError):
            return None
        return payload if isinstance(payload, dict) else None

    def _load_generation(self) -> int:
        if self._path is None or not self._path.is_file():
            return 0
        try:
            return int(self._path.read_text(encoding="utf-8").strip() or "0")
        except ValueError:
            return 0

    def _store_generation(self) -> None:
        if self._path is None:
            return
        self._path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self._path.with_suffix(".tmp")
        temporary.write_text(str(self._generation), encoding="utf-8")
        temporary.replace(self._path)


def lease_refusal_if_enforced(payload: dict, op: str, authority: LeaseAuthority | None = None,
                              now: float | None = None) -> dict | None:
    """Used by the physical gateway class. Unset enforcement leaves the current process behavior."""
    if os.environ.get("GATEWAY_LEASE_ENFORCED") != "1":
        return None
    if op not in MOTION_OPS:
        return None
    gate = authority if authority is not None else _ENFORCED_AUTHORITY
    if gate is None:
        return {"ok": False, "refused": True, "reason": "lease gate가 설치되지 않았습니다"}
    authority = gate
    robot_id = str(payload.get("robot_id") or "")
    session_id = str(payload.get("session_id") or "")
    allowed, reason = authority.allows(
        str(payload.get("lease") or ""), robot_id, session_id, now,
        claimed_environment=str(payload.get("environment_id") or ""),
    )
    if allowed:
        return None
    return {"ok": False, "refused": True, "reason": reason}
