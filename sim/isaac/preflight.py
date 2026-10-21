"""Report whether a virtual pinky1 Isaac Sim launch is allowed."""
from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

from sim.isaac.gates import launch_block_reason


def isaac_root() -> Path:
    return Path(os.environ.get("ISAAC_SIM_ROOT", "/home/sw/isaacsim"))


def available_ram_bytes() -> int:
    for line in Path("/proc/meminfo").read_text(encoding="utf-8").splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) * 1024
    return 0


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
    version_path = isaac_root() / "VERSION"
    version = version_path.read_text(encoding="utf-8").strip() if version_path.is_file() else None
    domain_raw = os.environ.get("ROS_DOMAIN_ID")
    domain = int(domain_raw) if domain_raw not in (None, "") else None
    available = available_ram_bytes()
    reason = launch_block_reason(
        domain_id=domain,
        available_bytes=available,
        isaac_installed=version_path.is_file(),
        hardware_nodes_on_domain=hardware_nodes_present(),
    )
    print(json.dumps({
        "isaac_version": version,
        "isaac_root": str(isaac_root()),
        "ros_domain_id": domain,
        "rmw": os.environ.get("RMW_IMPLEMENTATION"),
        "available_ram_bytes": available,
        "block_reason": reason,
        "simulation_confirmed": False,
    }, ensure_ascii=False))
    return 0 if reason is None else 2


if __name__ == "__main__":
    raise SystemExit(main())
