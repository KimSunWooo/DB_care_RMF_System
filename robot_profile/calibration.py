"""Measured differences between the Pinky driver and the URDF.

No radius or joint name is selected as the correction. Both values stay on record
until a physical measurement chooses one.
"""
from __future__ import annotations

BRINGUP_WHEEL_RADIUS_M = 0.027
URDF_WHEEL_RADIUS_M = 0.028
GAZEBO_PLUGIN_WHEEL_RADIUS_M = 0.028
BRINGUP_WHEEL_SEPARATION_M = 0.0961
GAZEBO_PLUGIN_WHEEL_SEPARATION_M = 0.0961
URDF_JOINT_HALF_SPAN_M = 0.04055
URDF_JOINT_SEPARATION_M = URDF_JOINT_HALF_SPAN_M * 2
ENCODER_TICKS_PER_REV = 4096
GEAR_RATIO = None
DRIVER_JOINTS = ("left_wheel_joint", "right_wheel_joint")
URDF_JOINTS = ("l_wheel_joint", "r_wheel_joint")
CHOSEN_CORRECTION = None


def calibration_status() -> dict:
    return {
        "wheel_radius_m": {
            "bringup": BRINGUP_WHEEL_RADIUS_M,
            "urdf_and_gazebo_plugin": URDF_WHEEL_RADIUS_M,
            "chosen": CHOSEN_CORRECTION,
        },
        "wheel_separation_m": {
            "bringup_and_gazebo_plugin": BRINGUP_WHEEL_SEPARATION_M,
            "urdf_joint_origins": URDF_JOINT_SEPARATION_M,
            "chosen": CHOSEN_CORRECTION,
        },
        "joints": {
            "driver": list(DRIVER_JOINTS),
            "urdf": list(URDF_JOINTS),
            "chosen": CHOSEN_CORRECTION,
        },
        "encoder_ticks_per_rev": ENCODER_TICKS_PER_REV,
        "gear_ratio": GEAR_RATIO,
    }
