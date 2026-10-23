"""Persistent per-robot task queue. Restart does not resubmit old work."""
from __future__ import annotations

import sqlite3
import time
from pathlib import Path

TERMINAL = {"COMPLETED", "FAILED", "CANCELED", "INTERRUPTED"}


class TaskStore:
    def __init__(self, path: Path) -> None:
        self.path = path
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._init()
        self.recover_interrupted()

    def _connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(self.path)
        connection.row_factory = sqlite3.Row
        return connection

    def _init(self) -> None:
        with self._connect() as connection:
            connection.execute(
                """
                CREATE TABLE IF NOT EXISTS tasks (
                    request_id TEXT PRIMARY KEY,
                    correlation_id TEXT,
                    rmf_booking_id TEXT,
                    robot_id TEXT,
                    fleet_id TEXT,
                    origin TEXT,
                    destination TEXT,
                    action TEXT,
                    trip_type TEXT,
                    status TEXT,
                    error TEXT,
                    dispatch_allowed INTEGER NOT NULL DEFAULT 0,
                    kind TEXT,
                    phases TEXT,
                    created_at TEXT,
                    started_at TEXT,
                    completed_at TEXT
                )
                """
            )

    def recover_interrupted(self) -> int:
        """A restarted process must not launch work that was already underway."""
        now = _now()
        with self._connect() as connection:
            pending = connection.execute(
                """
                SELECT COUNT(*)
                  FROM tasks
                 WHERE status IN ('UNDERWAY', 'ASSIGNED', 'RMF_ACCEPTED', 'SUBMISSION_UNKNOWN')
                   AND dispatch_allowed = 1
                """
            ).fetchone()[0]
            if not pending:
                return 0
            cursor = connection.execute(
                """
                UPDATE tasks
                   SET status = 'INTERRUPTED',
                       error = 'process restarted before completion; not resubmitted',
                       completed_at = ?
                 WHERE status IN ('UNDERWAY', 'ASSIGNED', 'RMF_ACCEPTED', 'SUBMISSION_UNKNOWN')
                   AND dispatch_allowed = 1
                """,
                (now,),
            )
            return cursor.rowcount

    def put(self, record: dict) -> dict:
        request_id = str(record["request_id"])
        with self._connect() as connection:
            existing = connection.execute("SELECT request_id FROM tasks WHERE request_id = ?", (request_id,)).fetchone()
            if existing:
                raise ValueError("duplicate request id")
            try:
                connection.execute(
                    """
                    INSERT INTO tasks (
                        request_id, correlation_id, rmf_booking_id, robot_id, fleet_id, origin,
                        destination, action, trip_type, status, error, dispatch_allowed, kind,
                        phases, created_at, started_at, completed_at
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        request_id, record.get("correlation_id"), record.get("rmf_booking_id"),
                        record.get("robot_id"), record.get("fleet_id"), record.get("origin"),
                        record.get("destination"), record.get("action"), record.get("trip_type"),
                        record.get("status") or "QUEUED", record.get("error"),
                        1 if record.get("dispatch_allowed") else 0, record.get("kind"),
                        record.get("phases"), record.get("created_at") or _now(),
                        record.get("started_at"), record.get("completed_at"),
                    ),
                )
            except sqlite3.IntegrityError as exc:
                raise ValueError("duplicate request id") from exc
        return self.get(request_id)

    def get(self, request_id: str) -> dict | None:
        with self._connect() as connection:
            row = connection.execute("SELECT * FROM tasks WHERE request_id = ?", (request_id,)).fetchone()
        return dict(row) if row else None

    def list_robot(self, robot_id: str | None = None) -> list[dict]:
        with self._connect() as connection:
            if robot_id:
                rows = connection.execute(
                    "SELECT * FROM tasks WHERE robot_id = ? ORDER BY created_at, request_id", (robot_id,)
                ).fetchall()
            else:
                rows = connection.execute("SELECT * FROM tasks ORDER BY created_at, request_id").fetchall()
        return [dict(row) for row in rows]

    def cancel_queued(self, request_id: str) -> dict | None:
        with self._connect() as connection:
            row = connection.execute("SELECT status FROM tasks WHERE request_id = ?", (request_id,)).fetchone()
            if row is None:
                return None
            if row["status"] != "QUEUED":
                raise ValueError("only a queued task can be canceled before it starts")
            connection.execute(
                "UPDATE tasks SET status = 'CANCELED', completed_at = ? WHERE request_id = ?",
                (_now(), request_id),
            )
        return self.get(request_id)

    def mark(self, request_id: str, status: str, **fields: object) -> dict | None:
        assignments = ["status = ?"]
        values: list[object] = [status]
        for key, value in fields.items():
            assignments.append(f"{key} = ?")
            values.append(value)
        values.append(request_id)
        with self._connect() as connection:
            connection.execute(f"UPDATE tasks SET {', '.join(assignments)} WHERE request_id = ?", values)
        return self.get(request_id)

    def next_queued(self, robot_id: str) -> dict | None:
        with self._connect() as connection:
            active = connection.execute(
                """
                SELECT request_id FROM tasks
                 WHERE robot_id = ? AND status NOT IN ('QUEUED', 'PENDING', 'COMPLETED', 'FAILED', 'CANCELED', 'INTERRUPTED')
                """,
                (robot_id,),
            ).fetchone()
            if active:
                return None
            row = connection.execute(
                """
                SELECT * FROM tasks
                 WHERE robot_id = ? AND status = 'QUEUED'
                 ORDER BY created_at, request_id
                 LIMIT 1
                """,
                (robot_id,),
            ).fetchone()
        return dict(row) if row else None


def promote_fifo(store: TaskStore, robot_id: str, dispatcher) -> dict | None:
    """Start the oldest queued task only when the caller allows dispatch."""
    nxt = store.next_queued(robot_id)
    if nxt is None or not nxt.get("dispatch_allowed"):
        return nxt
    result = dispatcher(nxt)
    if not result or not result.get("submitted"):
        return nxt
    return store.mark(
        nxt["request_id"], "RMF_ACCEPTED",
        rmf_booking_id=result.get("rmf_booking_id"), started_at=_now(),
    )


def _now() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
