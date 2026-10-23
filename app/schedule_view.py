"""Read-only view of RMF schedule visualization. Does not start a schedule node."""
from __future__ import annotations

import socket
from pathlib import Path

_PACKAGE = Path("/opt/ros/jazzy/share/rmf_visualization_schedule")
_PORTS = (8006, 7878)


def _port_open(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.2):
            return True
    except OSError:
        return False


def schedule_view() -> dict:
    installed = _PACKAGE.is_dir()
    listening = [port for port in _PORTS if _port_open(port)]
    connected = bool(listening)
    reason = None
    if not installed:
        reason = "rmf_visualization_schedule is not installed"
    elif not connected:
        reason = "rmf_visualization_schedule is installed, but its websocket server is not running"
    return {
        "package_installed": installed,
        "websocket_connected": connected,
        "listening_ports": listening,
        "trajectories": [],
        "conflicts": [],
        "source": "rmf_visualization_schedule" if connected else None,
        "reason": reason,
    }
