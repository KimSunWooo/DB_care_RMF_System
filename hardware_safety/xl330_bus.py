"""Read-only XL330 bus. Velocity, torque, and watchdog writes are refused."""
from __future__ import annotations

import subprocess

from hardware_safety.real_driver import REAL_PORT, ServoSample

READS = (
    ("firmware", 6, 1),
    ("operating_mode", 11, 1),
    ("torque", 64, 1),
    ("hardware_error", 70, 1),
    ("bus_watchdog", 98, 1),
    ("velocity", 128, 4),
    ("position", 132, 4),
)


class Xl330ReadBus:
    def __init__(self, path: str = REAL_PORT) -> None:
        if path != REAL_PORT:
            raise RuntimeError("refusing a serial path other than /dev/ttyAMA4")
        self.path = path
        self.port = None
        self.packet = None
        self.port_open = False
        self.refused: list[str] = []
        self.voltage: dict[int, int | None] = {}

    def open(self) -> None:
        try:
            held = subprocess.check_output(["lsof", self.path], text=True, stderr=subprocess.DEVNULL)
        except (subprocess.CalledProcessError, FileNotFoundError):
            held = ""
        if "COMMAND" in held:
            raise RuntimeError("ttyAMA4 is already open")
        from dynamixel_sdk import PacketHandler, PortHandler
        self.port = PortHandler(self.path)
        self.packet = PacketHandler(2.0)
        if not self.port.openPort() or not self.port.setBaudRate(1000000):
            raise RuntimeError("failed to open ttyAMA4")
        self.port_open = True

    def close(self) -> None:
        if self.port is not None and self.port_open:
            self.port.closePort()
        self.port_open = False

    def read_samples(self) -> list[ServoSample]:
        if not self.port_open:
            raise ConnectionError("bus is closed")
        samples = []
        for servo_id in (1, 2):
            model, comm, _err = self.packet.ping(self.port, servo_id)
            if comm != 0:
                raise ConnectionError(f"ping failed for id {servo_id}")
            values = {"servo_id": servo_id, "model": model}
            for name, address, length in READS:
                values[name] = self._read(servo_id, address, length)
            voltage, vcomm, _err = self.packet.read2ByteTxRx(self.port, servo_id, 144)
            self.voltage[servo_id] = voltage if vcomm == 0 else None
            samples.append(ServoSample(
                servo_id=servo_id,
                model=int(values["model"]),
                firmware=int(values["firmware"]),
                torque=int(values["torque"]),
                hardware_error=int(values["hardware_error"]),
                bus_watchdog=int(values["bus_watchdog"]),
                position=int(values["position"]),
                velocity=int(values["velocity"]),
                operating_mode=int(values["operating_mode"]),
            ))
        return samples

    def _read(self, servo_id: int, address: int, length: int) -> int:
        if length == 1:
            value, comm, _err = self.packet.read1ByteTxRx(self.port, servo_id, address)
        elif length == 2:
            value, comm, _err = self.packet.read2ByteTxRx(self.port, servo_id, address)
        else:
            value, comm, _err = self.packet.read4ByteTxRx(self.port, servo_id, address)
        if comm != 0:
            raise ConnectionError(f"read failed id {servo_id} address {address}")
        if length == 4 and value > 2**31:
            value -= 2**32
        return int(value)

    def write_goal_velocity(self, _rpm_left: float, _rpm_right: float) -> bool:
        self.refused.append("velocity")
        return False

    def write_stop(self) -> bool:
        self.refused.append("stop")
        return False

    def write_torque(self, _enabled: int) -> bool:
        self.refused.append("torque")
        return False

    def write_bus_watchdog(self, _value: int) -> None:
        self.refused.append("watchdog")
        raise RuntimeError("refusing a bus watchdog write")
