"""Start the domain-90 subscriber and publisher as separate processes."""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _child(script: str, extra: list[str]) -> subprocess.Popen:
    command = " ".join([
        "source /opt/ros/jazzy/setup.bash",
        "&&",
        "export ROS_DOMAIN_ID=90",
        "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
        f'PYTHONPATH="{ROOT}:${{PYTHONPATH:-}}"',
        "&&",
        "python3",
        script,
        *extra,
    ])
    return subprocess.Popen(
        ["bash", "-lc", command],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def _stop(process: subprocess.Popen) -> None:
    if process.poll() is None:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()


def main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") in {"0", "42", "53"}:
        os.environ["ROS_DOMAIN_ID"] = "90"
    subscriber = _child("scripts/odom_mock_subscriber.py", [])
    publisher = None
    restarted = None
    try:
        time.sleep(1.0)
        publisher = _child("scripts/odom_mock_publisher.py", ["--exit-after", "0.2"])
        publisher.wait(timeout=15)
        time.sleep(0.7)
        restarted = _child("scripts/odom_mock_publisher.py", ["--exit-after", "0.2"])
        restarted.wait(timeout=15)
        stdout, stderr = subscriber.communicate(timeout=8)
        print(stdout.strip())
        if subscriber.returncode != 0:
            print(stderr, file=sys.stderr)
            return subscriber.returncode or 3
        report = json.loads(stdout.strip().splitlines()[-1])
        return 0 if report.get("passed") else 3
    finally:
        for process in (publisher, restarted, subscriber):
            if process is not None:
                _stop(process)


if __name__ == "__main__":
    raise SystemExit(main())
