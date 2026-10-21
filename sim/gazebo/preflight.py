"""Report whether a virtual pinky1 Gazebo launch is allowed. Does not start Gazebo."""
from __future__ import annotations

import json
import os
import shutil
import subprocess

from sim.gazebo.gates import gazebo_block_reason


def gz_sim_installed() -> bool:
    if shutil.which("gz") is None:
        return False
    try:
        completed = subprocess.run(
            ["gz", "sim", "--version"],
            check=False,
            capture_output=True,
            text=True,
            timeout=8,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    text = f"{completed.stdout}\n{completed.stderr}"
    return "Gazebo Sim, version 8" in text


def hardware_nodes_present() -> bool:
    """True when this shell can see a ROS graph. The caller must already be on the sim domain."""
    if shutil.which("ros2") is None:
        return False
    try:
        completed = subprocess.run(
            ["ros2", "node", "list"],
            check=False,
            capture_output=True,
            text=True,
            timeout=8,
        )
    except (OSError, subprocess.TimeoutExpired):
        return False
    nodes = [line.strip() for line in completed.stdout.splitlines() if line.strip()]
    return bool(nodes)


def main() -> int:
    domain_raw = os.environ.get("ROS_DOMAIN_ID")
    domain = int(domain_raw) if domain_raw not in (None, "") else None
    installed = gz_sim_installed()
    reason = gazebo_block_reason(
        domain_id=domain,
        gz_installed=installed,
        hardware_nodes_on_domain=hardware_nodes_present(),
    )
    print(json.dumps({
        "gz_sim_installed": installed,
        "ros_domain_id": domain,
        "rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "block_reason": reason,
        "simulation_confirmed": False,
    }, ensure_ascii=False))
    return 0 if reason is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
