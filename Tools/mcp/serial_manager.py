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

    Searches for PX4 boards by matching known USB identifiers:
    - CDC/ACM devices (ttyACM*) which PX4 boards typically enumerate as
    - USB UART adapters used with some boards

    Raises ValueError if auto-detection finds zero or multiple ports.
    """
    if port is not None:
        return port

    # Look for PX4/CUAV/flight controller CDC ACM devices first
    px4_patterns = ["PX4", "CUAV", "FMU", "Matek", "NXT"]
    ports = []
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "") + " " + (p.manufacturer or "")
        if any(pat.lower() in desc.lower() for pat in px4_patterns):
            ports.append(p)
        elif p.device.startswith("/dev/ttyACM"):
            # ttyACM devices are CDC/ACM — likely a PX4 board
            ports.append(p)

    # Fall back to USB UART if no ACM devices found
    if not ports:
        ports = list(serial.tools.list_ports.grep("USB UART"))

    if len(ports) == 0:
        raise ValueError(
            "No PX4 serial ports detected. Check USB connection."
        )
    if len(ports) > 1:
        devices = [f"{p.device} ({p.description})" for p in ports]
        raise ValueError(
            f"Multiple serial ports found: {devices}. "
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
