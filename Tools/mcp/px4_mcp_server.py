"""PX4 MCP Server for firmware development.

Provides tools to build, upload, and interact with PX4 firmware on
any supported board (wheel loader, CUAV, Holybro, etc.) via Claude Code.

Usage:
    python3 Tools/mcp/px4_mcp_server.py
"""

import asyncio
import json
import os
import shutil
import sys
from pathlib import Path

from mcp.server.fastmcp import FastMCP

# Ensure all logging goes to stderr (stdout is reserved for MCP JSON-RPC)
import logging
logging.basicConfig(stream=sys.stderr, level=logging.INFO)

# Local modules
sys.path.insert(0, str(Path(__file__).parent))
import nsh_client
import serial_manager

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
DOCKER_IMAGE = "px4io/px4-dev:v1.16.0-rc1-258-g0369abd556"

BOARD_TARGETS = {
    # --- Wheel loader boards ---
    "wheel_loader_cuav-x7plus-wl_default": {
        "role": "Main coordinator",
        "chip": "STM32H743",
        "description": "CUAV X7+ flight controller, runs vla_proxy, operation_mode, health_monitor, arm_manager, operator_interface, strategy_executor",
    },
    "wheel_loader_holybro-v6xrt-wl_default": {
        "role": "Alt coordinator (Ethernet)",
        "chip": "MIMXRT1176",
        "description": "Holybro V6X-RT flight controller, same modules as cuav-x7plus-wl but with Ethernet support",
    },
    "wheel_loader_nxt-dual-wl-front_default": {
        "role": "Front actuator control",
        "chip": "STM32H743",
        "description": "NXT Dual board for front body: drivetrain_controller, steering_controller, traction_controller, tilt_control",
    },
    "wheel_loader_nxt-dual-wl-rear_default": {
        "role": "Rear actuator control",
        "chip": "STM32H743",
        "description": "NXT Dual board for rear body: drivetrain_controller, steering_controller, traction_controller, boom_control, driver_lamp_controller, load_lamp_controller",
    },
    # --- Standard PX4 boards ---
    "cuav_x7pro_default": {
        "role": "Flight controller",
        "chip": "STM32H753",
        "description": "CUAV X7Pro flight controller (FMUv6X class)",
    },
    "cuav_nora_default": {
        "role": "Flight controller",
        "chip": "STM32H743",
        "description": "CUAV Nora flight controller",
    },
    "cuav_7-nano_default": {
        "role": "Flight controller",
        "chip": "STM32H743",
        "description": "CUAV V7 Nano flight controller",
    },
    "cuav_x25-evo_default": {
        "role": "Flight controller",
        "chip": "STM32H743",
        "description": "CUAV X25 Evo flight controller",
    },
}

mcp = FastMCP("px4", log_level="WARNING")


# ---------------------------------------------------------------------------
# Resources
# ---------------------------------------------------------------------------


@mcp.resource("px4://boards")
def get_boards() -> str:
    """List all supported board targets with roles and descriptions."""
    return json.dumps(BOARD_TARGETS, indent=2)


@mcp.resource("px4://build/{target}/status")
def get_build_status(target: str) -> str:
    """Check whether firmware exists for a target, with file size and mtime."""
    if target not in BOARD_TARGETS:
        valid = list(BOARD_TARGETS.keys())
        return json.dumps({"error": f"Unknown target '{target}'. Valid targets: {valid}"})

    firmware = PROJECT_ROOT / "build" / target / f"{target}.px4"

    if not firmware.exists():
        return json.dumps({
            "target": target,
            "built": False,
            "message": f"No firmware found. Run build_firmware('{target}') first.",
        })

    stat = firmware.stat()
    return json.dumps({
        "target": target,
        "built": True,
        "path": str(firmware),
        "size_bytes": stat.st_size,
        "modified": stat.st_mtime,
    })


# ---------------------------------------------------------------------------
# Tools
# ---------------------------------------------------------------------------


@mcp.tool()
async def build_firmware(target: str, clean: bool = False) -> str:
    """Build PX4 firmware for a board target.

    Args:
        target: Board target name (e.g. 'cuav_x7pro_default', 'wheel_loader_cuav-x7plus-wl_default')
        clean: If True, run 'make clean' before building
    """
    if target not in BOARD_TARGETS:
        valid = list(BOARD_TARGETS.keys())
        return f"Unknown target '{target}'. Valid targets: {valid}"

    if not shutil.which("docker"):
        return "Docker not found. Ensure Docker is installed and in PATH."

    src_dir = str(PROJECT_ROOT)
    ccache_dir = os.path.expanduser("~/.ccache")
    os.makedirs(ccache_dir, exist_ok=True)

    build_cmd = f"make {target}"
    if clean:
        build_cmd = f"make clean && make {target}"

    docker_cmd = [
        "docker", "run", "--rm",
        "-w", src_dir,
        f"--user={os.getuid()}:{os.getgid()}",
        f"--env=CCACHE_DIR={ccache_dir}",
        f"--volume={ccache_dir}:{ccache_dir}:rw",
        f"--volume={src_dir}:{src_dir}:rw",
        DOCKER_IMAGE,
        "/bin/bash", "-c",
        f"git config --global --add safe.directory '*' && {build_cmd}",
    ]

    try:
        proc = await asyncio.create_subprocess_exec(
            *docker_cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=600)
        output = stdout.decode("utf-8", errors="replace")
    except asyncio.TimeoutError:
        proc.kill()
        return "Build timed out after 600 seconds."
    except Exception as e:
        return f"Build failed to start: {e}"

    if proc.returncode != 0:
        # Return last 30 lines on failure
        lines = output.splitlines()
        tail = "\n".join(lines[-30:])
        return f"Build failed (exit code {proc.returncode}).\n\nLast 30 lines:\n{tail}"

    firmware = PROJECT_ROOT / "build" / target / f"{target}.px4"
    if firmware.exists():
        size = firmware.stat().st_size
        return f"Build succeeded. Firmware: {firmware} ({size} bytes)"
    else:
        return f"Build completed (exit 0) but firmware file not found at {firmware}."


@mcp.tool()
async def upload_firmware(target: str, port: str | None = None) -> str:
    """Upload firmware to a PX4 board via USB bootloader.

    Args:
        target: Board target name
        port: Serial port (auto-detected if not specified)
    """
    if target not in BOARD_TARGETS:
        valid = list(BOARD_TARGETS.keys())
        return f"Unknown target '{target}'. Valid targets: {valid}"

    firmware = PROJECT_ROOT / "build" / target / f"{target}.px4"
    if not firmware.exists():
        return f"Firmware file not found. Run build_firmware('{target}') first."

    try:
        port = serial_manager.resolve_port(port)
    except ValueError as e:
        return str(e)

    uploader = PROJECT_ROOT / "Tools" / "px_uploader.py"
    cmd = [
        sys.executable, str(uploader),
        "--port", port,
        "--baud-bootloader", "115200",
        "--baud-flightstack", "57600",
        str(firmware),
    ]

    try:
        proc = await asyncio.create_subprocess_exec(
            *cmd,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        stdout, _ = await asyncio.wait_for(proc.communicate(), timeout=120)
        output = stdout.decode("utf-8", errors="replace")
    except asyncio.TimeoutError:
        proc.kill()
        return "Upload timed out after 120 seconds."
    except PermissionError:
        return f"Permission denied on {port}. Add user to dialout group: sudo usermod -aG dialout $USER"
    except Exception as e:
        return f"Upload failed: {e}"

    if proc.returncode != 0:
        return f"Upload failed (exit code {proc.returncode}).\n\n{output}"

    return f"Upload succeeded.\n\n{output}"


@mcp.tool()
async def nsh_command(
    command: str, port: str | None = None, timeout: float = 60
) -> str:
    """Run an NSH command on a connected PX4 board.

    Args:
        command: The NSH command to execute (e.g. 'top once', 'param show')
        port: Serial port (auto-detected if not specified)
        timeout: Command timeout in seconds
    """
    try:
        port = serial_manager.resolve_port(port)
    except ValueError as e:
        return str(e)

    async with serial_manager.acquire(port):
        try:
            result = await asyncio.to_thread(
                nsh_client.run_nsh_command, port, command, timeout
            )
        except nsh_client.NshTimeoutError as e:
            return str(e)
        except nsh_client.NshError as e:
            return f"NSH error: {e}"
        except PermissionError:
            return f"Permission denied on {port}. Add user to dialout group: sudo usermod -aG dialout $USER"
        except Exception as e:
            return f"Serial error: {e}"

    status = "OK" if result.return_code == 0 else f"ERROR (code {result.return_code})"
    return f"[{status}]\n{result.output}"


@mcp.tool()
async def list_serial_ports() -> str:
    """List all detected serial ports."""
    ports = serial_manager.detect_ports()
    if not ports:
        return "No serial ports detected."
    return json.dumps(ports, indent=2, default=str)


@mcp.tool()
async def get_device_info(port: str | None = None) -> str:
    """Get device information from a connected PX4 board.

    Runs 'ver all' and 'param show SYS*' to identify the board.

    Args:
        port: Serial port (auto-detected if not specified)
    """
    try:
        port = serial_manager.resolve_port(port)
    except ValueError as e:
        return str(e)

    async with serial_manager.acquire(port):
        parts = []
        for cmd in ["ver all", "param show SYS*"]:
            try:
                result = await asyncio.to_thread(
                    nsh_client.run_nsh_command, port, cmd, timeout=20
                )
                parts.append(f"$ {cmd}\n{result.output}")
            except Exception as e:
                parts.append(f"$ {cmd}\nError: {e}")

    return "\n".join(parts)


@mcp.tool()
async def monitor_boot(
    port: str | None = None, timeout: float = 180
) -> str:
    """Monitor board boot sequence after reset or firmware upload.

    Waits for the NSH prompt to appear, capturing the full boot log.

    Args:
        port: Serial port (auto-detected if not specified)
        timeout: Boot timeout in seconds (default 180)
    """
    try:
        port = serial_manager.resolve_port(port)
    except ValueError as e:
        return str(e)

    async with serial_manager.acquire(port):
        try:
            result = await asyncio.to_thread(
                nsh_client.monitor_boot, port, timeout
            )
        except nsh_client.NshTimeoutError as e:
            return str(e)
        except PermissionError:
            return f"Permission denied on {port}. Add user to dialout group: sudo usermod -aG dialout $USER"
        except Exception as e:
            return f"Serial error: {e}"

    status = "Boot complete" if result.return_code == 0 else "Boot timeout"
    return f"[{status}]\n{result.output}"


@mcp.tool()
async def run_hil_test(
    test_name: str, port: str | None = None, timeout: float = 300
) -> str:
    """Run a hardware-in-the-loop test on a connected PX4 board.

    Args:
        test_name: Test name (e.g. 'param', 'perf', 'float', 'hrt')
        port: Serial port (auto-detected if not specified)
        timeout: Test timeout in seconds (default 300)
    """
    try:
        port = serial_manager.resolve_port(port)
    except ValueError as e:
        return str(e)

    async with serial_manager.acquire(port):
        try:
            result = await asyncio.to_thread(
                nsh_client.run_hil_test, port, test_name, timeout
            )
        except nsh_client.NshTimeoutError as e:
            return str(e)
        except PermissionError:
            return f"Permission denied on {port}. Add user to dialout group: sudo usermod -aG dialout $USER"
        except Exception as e:
            return f"Serial error: {e}"

    if result.return_code == 0:
        status = f"{test_name} PASSED"
    elif result.return_code == 1:
        status = f"{test_name} FAILED"
    else:
        status = f"{test_name} ERROR (code {result.return_code})"

    return f"[{status}]\n{result.output}"


@mcp.tool()
async def uorb_listen(
    topic: str, count: int = 1, port: str | None = None
) -> str:
    """Listen to a uORB topic on a connected PX4 board.

    Args:
        topic: uORB topic name (e.g. 'vehicle_status', 'sensor_accel')
        count: Number of messages to capture (default 1)
        port: Serial port (auto-detected if not specified)
    """
    cmd = f"listener {topic} {count}"
    return await nsh_command(cmd, port=port, timeout=30)


@mcp.tool()
async def get_dmesg(port: str | None = None) -> str:
    """Read the kernel/system log (dmesg) from a connected PX4 board.

    Args:
        port: Serial port (auto-detected if not specified)
    """
    return await nsh_command("dmesg", port=port, timeout=10)


if __name__ == "__main__":
    mcp.run(transport="stdio")
