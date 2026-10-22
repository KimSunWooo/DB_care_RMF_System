"""Read-only encoder plan. Calling it does not open a port or write a register."""
from __future__ import annotations

from hardware_safety.session import SafetyError


# Addresses already present in the Pinky driver source. Watchdog is not among them.
SOURCE_REGISTERS = {
    "operating_mode": 11,
    "torque_enable": 64,
    "goal_velocity": 104,
    "profile_acceleration": 108,
    "present_velocity": 128,
    "present_position": 132,
}


class ReadOnlyPlan:
    """Describe a future read. execute() is refused until a separate approval."""

    def __init__(self) -> None:
        self.port_opened = False
        self.writes: list[tuple[str, int]] = []
        self.reads: list[str] = []
        self.model_name = None
        self.bus_watchdog_supported = None

    def execute(self) -> None:
        raise SafetyError(
            "encoder read stays closed: a read still uses the Dynamixel bus and needs approval"
        )

    def blockers(self) -> list[str]:
        return [
            "servo model name is unknown",
            "protocol 2.0 is what the current driver selects, but the servo manual is not in this repo",
            "torque state would be address 64 only after the model control table is confirmed",
            "present position is address 132 in the current driver, and reading it is still bus traffic",
            "the current initialize_motors reboots the servo and enables torque before any read",
            "opening /dev/ttyAMA4 conflicts with a driver that already owns the port",
            "bus watchdog support is unknown and no register is programmed",
            "a failed read does not stop a motor that is already in velocity mode",
            "physical e-stop is not verified",
        ]
