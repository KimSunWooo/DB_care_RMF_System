"""One reconciliation epoch at a time. Reads do not add observations."""
from __future__ import annotations

import copy
import fcntl
import json
import sqlite3
import threading
import time
import uuid
from pathlib import Path

from app.task_reconcile import OBSERVATION_WINDOW_MS, ReconciliationLog


LEGACY_ENVIRONMENT = "rmf-a-domain77"


def _utc(now: float | None = None) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(now if now is not None else time.time()))


def _unobserved(robot_id: str) -> dict:
    return {
        "robot_id": robot_id,
        "environment_id": LEGACY_ENVIRONMENT,
        "reconciliation_status": "PENDING_CONFIRMATION",
        "recovery_required": True,
        "dispatch_allowed": False,
        "state": "RECOVERY_REQUIRED",
        "reason": "Reconciliation worker가 아직 이 로봇을 확인하지 못했습니다.",
        "fleet_task_id": "",
        "gateway_goal_active": False,
        "adapter_task_owned": False,
        "bookings": [],
        "reports": [],
        "snapshot_stale": False,
        "observed": False,
    }


class ReconcileService:
    def __init__(self, path: Path, *, min_gap_ms: int = 2_000, stale_ms: int = 20_000,
                 window_ms: int = OBSERVATION_WINDOW_MS) -> None:
        self.path = path
        self.min_gap_ms = min_gap_ms
        self.stale_ms = stale_ms
        self.window_ms = window_ms
        self.environment_id = LEGACY_ENVIRONMENT
        self.generation = uuid.uuid4().hex
        self.log = ReconciliationLog()
        self._lock = threading.Lock()
        self._inflight = False
        self._last_mono_ms: int | None = None
        self._reports: dict[str, dict] = {}
        self._utc: str | None = None
        self._lock_file = None
        self._ready = False

    def begin_epoch(self, mono_ms: int) -> bool:
        with self._lock:
            if self._inflight:
                return False
            if self._last_mono_ms is not None and mono_ms < self._last_mono_ms:
                self._reports = {}
                self._last_mono_ms = None
                self.generation = uuid.uuid4().hex
                self.log = ReconciliationLog()
            if self._last_mono_ms is not None and mono_ms - self._last_mono_ms < self.min_gap_ms:
                return False
            self._inflight = True
        if not self._acquire_process_lock():
            with self._lock:
                self._inflight = False
            return False
        return True

    def commit_epoch(self, reports: dict[str, dict], mono_ms: int, utc: str | None = None) -> None:
        stamped = _utc() if utc is None else utc
        stored = {}
        for robot_id, report in reports.items():
            body = copy.deepcopy(report)
            body["robot_id"] = robot_id
            body["environment_id"] = self.environment_id
            body["observed"] = True
            body["snapshot_stale"] = False
            body["generation"] = self.generation
            stored[robot_id] = body
        with self._lock:
            self._reports = stored
            self._last_mono_ms = int(mono_ms)
            self._utc = stamped
            self._inflight = False
        self._persist(stored, stamped)
        self._release_process_lock()

    def fail_epoch(self, detail: str) -> None:
        failed = {}
        robots = list(self._reports) or ["real_pinky1", "real_pinky2", "real_pinky3"]
        for robot_id in robots:
            failed[robot_id] = {
                "robot_id": robot_id,
                "environment_id": self.environment_id,
                "reconciliation_status": "RMF_UNREACHABLE",
                "recovery_required": True,
                "dispatch_allowed": False,
                "state": "RECOVERY_REQUIRED",
                "reason": f"Reconciliation worker 오류: {detail}",
                "fleet_task_id": "",
                "gateway_goal_active": False,
                "adapter_task_owned": False,
                "bookings": [],
                "reports": [],
                "snapshot_stale": False,
                "observed": False,
                "generation": self.generation,
            }
        with self._lock:
            self._reports = failed
            self._inflight = False
            self._utc = _utc()
        self._release_process_lock()

    def view(self, robot_id: str, now_mono_ms: int | None = None) -> dict:
        now = int(time.monotonic() * 1000) if now_mono_ms is None else int(now_mono_ms)
        with self._lock:
            report = self._reports.get(robot_id)
            committed = self._last_mono_ms
        if report is None or committed is None:
            return _unobserved(robot_id)
        body = copy.deepcopy(report)
        if now - committed > self.stale_ms:
            body["snapshot_stale"] = True
            body["dispatch_allowed"] = False
            body["recovery_required"] = True
            body["state"] = "RECOVERY_REQUIRED"
            body["reason"] = "Reconciliation snapshot이 오래되어 신규 주행을 막습니다."
            if body.get("reconciliation_status") == "CONSISTENT":
                body["reconciliation_status"] = "PENDING_CONFIRMATION"
        body.setdefault("bookings", body.get("reports") or [])
        return body

    def board(self) -> dict:
        with self._lock:
            robots = [copy.deepcopy(item) for item in self._reports.values()]
            committed = self._last_mono_ms
            utc = self._utc
            generation = self.generation
        observed = committed is not None
        orphaned = 0
        active = 0
        for robot in robots:
            for booking in robot.get("bookings") or []:
                if booking.get("reconciliation_status") == "OWNER_MISSING":
                    orphaned += 1
                if str(booking.get("rmf_status") or "").lower() in {"queued", "standby", "underway", "blocked"}:
                    active += 1
        monitoring = {
            "rmf_reachable": None,
            "gateway_reachable": None,
            "active_task_count": active if observed else None,
            "orphaned_task_count": orphaned if observed else None,
            "reconciliation_age_ms": None if not observed else max(0, int(time.monotonic() * 1000) - int(committed or 0)),
            "submission_unknown_count": None,
            "cancellation_timeout_count": None,
            "duplicate_goal_rejection_count": None,
            "gateway_mode": None,
            "worker_alive": observed,
        }
        if observed:
            statuses = {robot.get("reconciliation_status") for robot in robots}
            monitoring["rmf_reachable"] = "RMF_UNREACHABLE" not in statuses
            monitoring["gateway_reachable"] = "GATEWAY_UNREACHABLE" not in statuses
        return {
            "environment_id": self.environment_id,
            "robots": robots,
            "motion": False,
            "generation": generation,
            "observed": observed,
            "utc": utc,
            "duplicate": False,
            "monitoring": monitoring,
            "historical": self.historical(),
        }

    def historical(self) -> list[dict]:
        if not self.path.is_file():
            return []
        try:
            with sqlite3.connect(self.path) as connection:
                rows = connection.execute(
                    "SELECT utc, kind, payload FROM events ORDER BY id DESC LIMIT 20"
                ).fetchall()
        except sqlite3.Error:
            return []
        found = []
        for utc, kind, payload in rows:
            try:
                body = json.loads(payload)
            except json.JSONDecodeError:
                continue
            found.append({"utc": utc, "kind": kind, **body})
        return found

    def _persist(self, reports: dict[str, dict], utc: str) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with sqlite3.connect(self.path) as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS events (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    utc TEXT NOT NULL,
                    kind TEXT NOT NULL,
                    payload TEXT NOT NULL
                )
                """
            )
            for robot_id, report in reports.items():
                for booking in report.get("bookings") or []:
                    if booking.get("reconciliation_status") != "OWNER_MISSING":
                        continue
                    booking_id = booking.get("booking_id") or booking.get("id")
                    previous = connection.execute(
                        "SELECT payload FROM events WHERE kind = ? ORDER BY id DESC LIMIT 1",
                        ("historical_orphaned",),
                    ).fetchone()
                    if previous:
                        try:
                            last = json.loads(previous[0])
                        except json.JSONDecodeError:
                            last = {}
                        if last.get("booking_id") == booking_id and last.get("rmf_status") == booking.get("rmf_status"):
                            continue
                    connection.execute(
                        "INSERT INTO events (utc, kind, payload) VALUES (?, ?, ?)",
                        (utc, "historical_orphaned", json.dumps({
                            "environment_id": self.environment_id,
                            "booking_id": booking_id,
                            "rmf_status": booking.get("rmf_status"),
                            "robot_id": robot_id,
                            "historical_classification": "ORPHANED",
                            "dispatch_allowed_on_environment": False,
                        })),
                    )

    def _acquire_process_lock(self) -> bool:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        lock_path = self.path.with_suffix(".lock")
        handle = open(lock_path, "a")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            handle.close()
            return False
        self._lock_file = handle
        return True

    def _release_process_lock(self) -> None:
        handle = self._lock_file
        self._lock_file = None
        if handle is None:
            return
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()

