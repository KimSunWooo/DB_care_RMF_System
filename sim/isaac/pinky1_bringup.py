"""Isaac Sim 6.2 bringup for one virtual pinky1.

This process exits before SimulationApp when the launch gate fails.
It does not send cmd_vel. A later run, after the gate passes, imports the
existing Pinky URDF and creates the ROS 2 bridge graph used by this Isaac
Sim build. Do not point it at the placeholder RMF graph.
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from sim.isaac.gates import (  # noqa: E402
    EXPECTED_CMD_VEL_TYPE,
    GAZEBO_PLUGIN_WHEEL_SEPARATION_M,
    URDF_WHEEL_SEPARATION_M,
    WHEEL_RADIUS_M,
    launch_block_reason,
)
from sim.isaac.preflight import available_ram_bytes, hardware_nodes_present, isaac_root  # noqa: E402

URDF = Path("/home/sw/DevSpace/pinky/src/pinky_pro/pinky_description/urdf/robot.urdf.xacro")


def main() -> int:
    domain_raw = os.environ.get("ROS_DOMAIN_ID")
    domain = int(domain_raw) if domain_raw not in (None, "") else None
    reason = launch_block_reason(
        domain_id=domain,
        available_bytes=available_ram_bytes(),
        isaac_installed=isaac_root().joinpath("VERSION").is_file(),
        hardware_nodes_on_domain=hardware_nodes_present(),
    )
    report = {
        "started": False,
        "cmd_vel_sent": False,
        "cmd_vel_type": EXPECTED_CMD_VEL_TYPE,
        "urdf": str(URDF),
        "urdf_exists": URDF.is_file(),
        "wheel_radius_m": WHEEL_RADIUS_M,
        "urdf_wheel_separation_m": URDF_WHEEL_SEPARATION_M,
        "gazebo_plugin_wheel_separation_m": GAZEBO_PLUGIN_WHEEL_SEPARATION_M,
        "block_reason": reason,
    }
    print(json.dumps(report, ensure_ascii=False))
    if reason:
        return 2
    print("launch gate passed, but SimulationApp is still not started by this command")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
