"""Run the mock encoder scenarios in process. Does not publish ROS messages."""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from hardware_safety.session import OdomSession, load_config  # noqa: E402


def main() -> int:
    domain = os.environ.get("ROS_DOMAIN_ID")
    config = load_config(ROOT)
    session = OdomSession("ODOM_MOCK", config.mock_domain_id, config)
    reason = session.arm({"encoder_joints"})
    report = {
        "mode": "ODOM_MOCK",
        "process_domain": domain,
        "mock_domain": config.mock_domain_id,
        "published": False,
        "port_opened": session.motor_port_open,
        "torque_enabled": session.torque_enabled,
        "arm_reason": reason,
    }
    if reason:
        print(json.dumps(report, ensure_ascii=False))
        return 2
    session.push_wheels(0, 0, 0.0)
    session.push_wheels(100, 100, 0.1)
    report["forward_x"] = session.x
    report["forward_theta"] = session.theta
    report["joint_names"] = session.joint_state()["name"]
    report["tf"] = session.tf()
    report["passed"] = session.x > 0.0 and abs(session.theta) < 1e-9
    print(json.dumps(report, ensure_ascii=False))
    return 0 if report["passed"] else 3


if __name__ == "__main__":
    raise SystemExit(main())
