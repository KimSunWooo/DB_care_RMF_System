"""Scripted wheel samples for the domain-90 mock. No ROS and no serial port."""
from __future__ import annotations

from hardware_safety.session import OdomSession, max_wheel_ticks


def wheel_samples() -> list[tuple[float, int, int]]:
    """Straight, then a turn, then a hold. Each step stays inside the tick cap."""
    samples = [(0.0, 0, 0)]
    left = right = 0
    clock = 0.0
    for _ in range(20):
        clock += 0.05
        left += 40
        right += 40
        samples.append((round(clock, 5), left, right))
    for _ in range(20):
        clock += 0.05
        left += 30
        right -= 30
        samples.append((round(clock, 5), left, right))
    for _ in range(10):
        clock += 0.05
        samples.append((round(clock, 5), left, right))
    return samples


def replay(session: OdomSession) -> OdomSession:
    for clock, left, right in wheel_samples():
        allowed = True if session.last_wheels is None else abs(left - session.last_wheels[0]) <= max_wheel_ticks(session.config, 0.05)
        if not allowed:
            raise RuntimeError("scripted sample exceeds the encoder cap")
        if not session.push_wheels(left, right, clock):
            raise RuntimeError(session.last_reject or session.state)
    return session
