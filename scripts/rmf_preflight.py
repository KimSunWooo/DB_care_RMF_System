"""Read-only RMF preflight. Does not dispatch tasks or send Nav2 goals.

    .venv/bin/python scripts/rmf_preflight.py
"""
from __future__ import annotations

import asyncio
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import app.main  # noqa: E402,F401  # loads gitignored .env once
from app.rmf_client import RmfClient, RmfClientError  # noqa: E402
from app.rmf_token import build_token_provider  # noqa: E402
from fleet_adapter.config import load_fleet_spec, read_graph_names  # noqa: E402
from fleet_adapter.safety import navigation_block_reason  # noqa: E402
import os  # noqa: E402


def _shape(body):
    if isinstance(body, list):
        return {"kind": "list", "length": len(body)}
    if isinstance(body, dict):
        return {"kind": "dict", "keys": sorted(body)[:16]}
    return {"kind": type(body).__name__}


async def _api() -> list[dict]:
    provider = build_token_provider()
    client = RmfClient(os.getenv("RMF_API_URL", "http://127.0.0.1:8000"), provider=provider)
    rows = []
    for path in ("/fleets", "/tasks", "/building_map", "/time"):
        try:
            body = await client.get_json(path)
            rows.append({"path": path, "http": 200, "body": _shape(body)})
        except RmfClientError as ex:
            rows.append({"path": path, "http": ex.status_code, "kind": ex.kind})
    return rows


def main() -> int:
    spec = load_fleet_spec(ROOT / "config" / "rmf" / "care_move_fleet.yaml", ROOT)
    names = read_graph_names(spec.nav_graph)
    report = {
        "fleet": spec.fleet_name,
        "active_robots": [robot.robot_id for robot in spec.active_robots],
        "disabled_robots": [robot.robot_id for robot in spec.robots if not robot.enabled],
        "graph_alignment": spec.graph_alignment,
        "graph_waypoints": names,
        "server_uri_configured": spec.server_uri.endswith("/_internal"),
        "motion_block": navigation_block_reason(
            graph_alignment=spec.graph_alignment,
            allow_motion=spec.allow_motion,
            simulation_confirmed=spec.simulation_confirmed,
            action_available=False,
            pose_fresh=False,
        ),
        "api": asyncio.run(_api()),
    }
    encoded = json.dumps(report, ensure_ascii=False)
    if "eyJ" in encoded or "Bearer " in encoded:
        raise SystemExit("preflight output contained credential material")
    print(encoded)
    fleets = next(item for item in report["api"] if item["path"] == "/fleets")
    return 0 if fleets.get("http") == 200 else 1


if __name__ == "__main__":
    raise SystemExit(main())
