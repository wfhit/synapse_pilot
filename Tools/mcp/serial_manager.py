"""Serial port auto-detection and exclusive access management."""

import asyncio
from contextlib import asynccontextmanager

import serial.tools.list_ports

# Per-port locks keyed by device path
_locks: dict[str, asyncio.Lock] = {}


def detect_ports() -> list[dict]:
    """Return list of USB serial ports with metadata."""
    ports = []
    for p in serial.tools.list_ports.comports():
        ports.append({
            "device": p.device,
            "name": p.name,
            "description": p.description,
            "hwid": p.hwid,
            "manufacturer": p.manufacturer,
            "vid": p.vid,
            "pid": p.pid,
        })
    return ports


def resolve_port(port: str | None) -> str:
    """Resolve a port path, auto-detecting if None.

    Matches the existing HIL pattern: grep('USB UART') and require
    exactly one match for auto-detection.

    Raises ValueError if auto-detection finds zero or multiple ports.
    """
    if port is not None:
        return port

    ports = list(serial.tools.list_ports.grep("USB UART"))

    if len(ports) == 0:
        raise ValueError(
            "No USB serial ports detected. Check USB connection."
        )
    if len(ports) > 1:
        devices = [p.device for p in ports]
        raise ValueError(
            f"Multiple USB serial ports found: {devices}. "
            "Specify a port explicitly."
        )

    return ports[0].device


@asynccontextmanager
async def acquire(port: str):
    """Async context manager for exclusive serial port access."""
    if port not in _locks:
        _locks[port] = asyncio.Lock()

    async with _locks[port]:
        yield port
