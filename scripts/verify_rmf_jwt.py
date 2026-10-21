"""Read-only RMF JWT check through the Care-Move client.

Does not print tokens, secrets, or Authorization headers.
Does not send task, robot, door, or lift commands.

Default settings come from the process environment and .env:

    .venv/bin/python scripts/verify_rmf_jwt.py

Short-lived rotation check. This does not edit .env. With --watch the process
uses 90/30/10 when those variables are not already exported:

    .venv/bin/python scripts/verify_rmf_jwt.py --watch
"""
from __future__ import annotations

import asyncio
import json
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

if "--watch" in sys.argv:
    os.environ.setdefault("RMF_JWT_TTL_SECONDS", "90")
    os.environ.setdefault("RMF_JWT_ROTATE_INTERVAL_SECONDS", "30")
    os.environ.setdefault("RMF_JWT_EXPIRY_SKEW_SECONDS", "10")

from app.main import refresh_rmf_observation, token_provider  # noqa: E402


def _public(event: str, observed: dict) -> dict:
    token = observed.get("token") if isinstance(observed.get("token"), dict) else token_provider.status()
    body = {
        "event": event,
        "auth_mode": token.get("auth_mode"),
        "token_ready": token.get("token_ready"),
        "generation": token.get("generation"),
        "issued_at": token.get("issued_at"),
        "expires_at": token.get("expires_at"),
        "next_rotation_at": token.get("next_rotation_at"),
        "last_refresh_result": token.get("last_refresh_result"),
        "last_refresh_reason": token.get("last_refresh_reason"),
        "ttl_seconds": token.get("ttl_seconds"),
        "rotate_interval_seconds": token.get("rotate_interval_seconds"),
        "expiry_skew_seconds": token.get("expiry_skew_seconds"),
        "status": observed.get("status"),
        "auth_state": observed.get("auth_state"),
        "fleet_count": observed.get("fleet_count"),
        "robot_count": observed.get("robot_count"),
        "map_data": observed.get("map_data"),
        "map_http_status": observed.get("map_http_status"),
        "data_fresh": observed.get("data_fresh"),
        "operation_ready": observed.get("operation_ready"),
        "http": 200 if observed.get("data_fresh") else None,
    }
    encoded = json.dumps(body)
    if "eyJ" in encoded or "Bearer " in encoded:
        raise RuntimeError("verification output contained credential material")
    return body


async def _fleets(event: str) -> dict:
    observed = await refresh_rmf_observation(apply_robots=False, force=True)
    return _public(event, observed)


async def _run(watch: bool) -> int:
    await token_provider.start()
    records = []
    try:
        initial = await _fleets("fleets")
        records.append(initial)
        if not watch:
            return 0 if initial.get("http") == 200 else 1
        started = time.time()
        ttl = float(token_provider.status()["ttl_seconds"])
        origin = int(initial["generation"])
        seen = {origin}
        deadline = started + ttl + 15
        while time.time() < deadline:
            await asyncio.sleep(1)
            current = token_provider.status()
            generation = int(current["generation"])
            if generation not in seen:
                seen.add(generation)
                records.append({
                    "event": "rotate",
                    "generation": generation,
                    "last_refresh_result": current.get("last_refresh_result"),
                    "last_refresh_reason": current.get("last_refresh_reason"),
                    "expires_at": current.get("expires_at"),
                })
            if generation >= origin + 2 and time.time() >= started + ttl:
                records.append(await _fleets("fleets_after_first_expiry"))
                break
        else:
            records.append({"event": "watch_timeout", "generation": token_provider.generation})
        rotated = any(item.get("event") == "rotate" for item in records)
        rotations = sum(1 for item in records if item.get("event") == "rotate")
        final = records[-1]
        ok = rotations >= 2 and final.get("event") == "fleets_after_first_expiry" and final.get("http") == 200 and rotated
        return 0 if ok else 1
    finally:
        print(json.dumps(records, ensure_ascii=False))
        await token_provider.close()


if __name__ == "__main__":
    raise SystemExit(asyncio.run(_run("--watch" in sys.argv)))
