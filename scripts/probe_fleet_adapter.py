"""Start the care_move adapter only when the schedule node is already up.

Run with the ROS 2 Python, not the project venv:

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    PYTHONPATH="$PWD:${PYTHONPATH:-}" python3 scripts/probe_fleet_adapter.py

Does not add a robot, does not set server_uri, and does not send a Nav2 goal.
Exits after a few seconds. Safe to run while the API server is already running.
"""
from __future__ import annotations

import json
import sys
from datetime import timedelta
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from fleet_adapter.config import load_fleet_spec  # noqa: E402


def main() -> int:
    spec = load_fleet_spec(ROOT / "config" / "rmf" / "care_move_fleet.yaml", ROOT)
    if spec.allow_motion or spec.graph_alignment != "placeholder":
        print(json.dumps({"started": False, "reason": "refusing probe because motion is not locked"}))
        return 2
    try:
        import rmf_adapter
        import rmf_adapter.easy_full_control as easy
    except ImportError as ex:
        print(json.dumps({"started": False, "reason": f"rmf_adapter import failed: {ex.__class__.__name__}"}))
        return 3
    rmf_adapter.init_rclcpp()
    configuration = easy.FleetConfiguration.from_config_files(
        str(ROOT / "config" / "rmf" / "care_move_fleet.yaml"),
        str(spec.nav_graph),
        None,
    )
    if configuration is None:
        print(json.dumps({"started": False, "reason": "fleet configuration rejected"}))
        return 4
    configuration.server_uri = None
    adapter = rmf_adapter.Adapter.make(
        f"{spec.fleet_name}_fleet_adapter", wait_time=timedelta(seconds=3)
    )
    if adapter is None:
        print(json.dumps({
            "started": False,
            "reason": "Adapter.make returned no adapter; rmf_traffic_schedule is not available",
            "robots_added": 0,
            "server_uri_sent": False,
        }))
        return 5
    adapter.start()
    adapter.add_easy_fleet(configuration)
    adapter.stop()
    print(json.dumps({
        "started": True,
        "robots_added": 0,
        "server_uri_sent": False,
        "fleet": spec.fleet_name,
        "active_config_robots": [robot.robot_id for robot in spec.active_robots],
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
