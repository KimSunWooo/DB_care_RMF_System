"""In-memory Dynamixel bus for the read-only procedure.

This object never opens /dev/ttyAMA4. A write raises and is recorded as refused.
"""
from __future__ import annotations

from hardware_safety.session import SafetyError

# Reads the future approved procedure may request. Writes stay empty.
READ_PLAN = (
    ("model_number", 0, 2),
    ("firmware", 6, 1),
    ("id", 7, 1),
    ("torque_enable", 64, 1),
    ("present_position", 132, 4),
)
WRITE_ADDRESSES = {
    11: "operating_mode",
    64: "torque_enable",
    98: "bus_watchdog",
    104: "goal_velocity",
    108: "profile_acceleration",
}


class FakeDynamixelBus:
    def __init__(self, devices: dict[int, dict[int, bytes]]) -> None:
        self.devices = devices
        self.opened = False
        self.path = ""
        self.writes: list[tuple[int, int, bytes]] = []
        self.reads: list[tuple[int, int, int]] = []
        self.aborted = ""

    def open(self, path: str) -> None:
        if path.startswith("/dev"):
            raise SafetyError("refusing a real serial path")
        self.opened = True
        self.path = path

    def read(self, servo_id: int, address: int, length: int) -> bytes:
        if not self.opened:
            raise SafetyError("fake bus is closed")
        self.reads.append((servo_id, address, length))
        device = self.devices.get(servo_id)
        payload = None if device is None else device.get(address)
        if payload is None or len(payload) != length:
            self.aborted = f"unexpected response from id {servo_id} address {address}"
            raise SafetyError(self.aborted)
        return payload

    def write(self, servo_id: int, address: int, payload: bytes) -> None:
        self.writes.append((servo_id, address, payload))
        name = WRITE_ADDRESSES.get(address, str(address))
        raise SafetyError(f"refusing write to {name}")


def _u16(value: int) -> bytes:
    return int(value).to_bytes(2, "little")


def _i32(value: int) -> bytes:
    return int(value).to_bytes(4, "little", signed=True)


def simulated_pair(model_a: int, model_b: int, torque_a: int = 0, torque_b: int = 0) -> dict[int, dict[int, bytes]]:
    """Injected stand-in devices. The numbers are not observations from a robot."""
    def device(model: int, servo_id: int, torque: int) -> dict[int, bytes]:
        return {
            0: _u16(model),
            6: bytes([1]),
            7: bytes([servo_id]),
            64: bytes([torque]),
            132: _i32(0),
        }
    return {1: device(model_a, 1, torque_a), 2: device(model_b, 2, torque_b)}


def run_fake_identification(bus: FakeDynamixelBus) -> dict:
    """Read model, torque, and position. Abort without a write if torque is on."""
    bus.open("fake://dynamixel")
    found = []
    for servo_id in (1, 2):
        model = int.from_bytes(bus.read(servo_id, 0, 2), "little")
        firmware = bus.read(servo_id, 6, 1)[0]
        echoed_id = bus.read(servo_id, 7, 1)[0]
        torque = bus.read(servo_id, 64, 1)[0]
        if torque != 0:
            bus.aborted = f"id {servo_id} torque is {torque}; not writing a disable"
            raise SafetyError(bus.aborted)
        if echoed_id != servo_id:
            bus.aborted = f"id {servo_id} echoed {echoed_id}"
            raise SafetyError(bus.aborted)
        position = int.from_bytes(bus.read(servo_id, 132, 4), "little", signed=True)
        found.append({
            "id": servo_id,
            "model_number": model,
            "firmware": firmware,
            "torque_enable": torque,
            "present_position": position,
            "source": "fake_bus",
        })
    return {
        "port": bus.path,
        "real_port_opened": False,
        "writes": list(bus.writes),
        "servos": found,
        "model_confirmed": False,
    }
