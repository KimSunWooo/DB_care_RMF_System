"""Motor bringup stays out of this file.

pinky_bringup opens /dev/ttyAMA4 and enables torque during initialize_motors.
This launch refuses SENSOR_ONLY, ODOM_MOCK, and HARDWARE alike.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from hardware_safety.session import motor_launch_block  # noqa: E402


def generate_launch_description():
    mode = os.environ.get("HARDWARE_MODE", "HARDWARE")
    raise RuntimeError(motor_launch_block(mode))
