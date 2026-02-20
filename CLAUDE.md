# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SynapsePilot is a robot platform for VLA (Vision-Language-Action) models, built on open drone hardware. It forks PX4 v1.17.0-alpha1 and restructures it to support robot types far beyond drones.

**Why not ArduPilot or stock PX4?** ArduPilot supports many robot types but lacks PX4's modular architecture, making it hard to understand and extend. PX4 has great modularity but is drone-centric, making non-drone robots difficult. SynapsePilot takes PX4's modular base and separates robot-specific logic from the common platform, so adding a new robot type (like a wheel loader) means adding modules in `src/<robot_type>/` without touching the platform core.

The first supported robot type is an autonomous wheel loader (`src/wheel_loader/`), running on NuttX RTOS with a distributed multi-board architecture. Standard PX4 boards (CUAV, Holybro, Matek, etc.) are also fully supported.

## Build Commands

**All builds must use Docker** — never build on the host directly.

```bash
# Wheel loader hardware targets
./Tools/docker_run.sh "make wheel_loader_cuav-x7plus-wl_default"      # Main coordinator
./Tools/docker_run.sh "make wheel_loader_holybro-v6xrt-wl_default"    # Alt coordinator (Ethernet)
./Tools/docker_run.sh "make wheel_loader_nxt-dual-wl-front_default"   # Front actuator board
./Tools/docker_run.sh "make wheel_loader_nxt-dual-wl-rear_default"    # Rear actuator board

# Standard PX4 boards
./Tools/docker_run.sh "make cuav_x7pro_default"
./Tools/docker_run.sh "make cuav_nora_default"

# SITL simulation
./Tools/docker_run.sh "make px4_sitl_default"

# Tests
./Tools/docker_run.sh "make tests"
./Tools/docker_run.sh "make test_unit"

# Clean
./Tools/docker_run.sh "make clean"
./Tools/docker_run.sh "make distclean"
```

Docker image for NuttX builds: `px4io/px4-dev:v1.16.0-rc1-258-g0369abd556` (set in `Tools/docker_run.sh`).

Alternatively, invoke Docker directly (useful from MCP or scripts):

```bash
docker run --rm -v <repo-root>:<repo-root> -w <repo-root> \
  px4io/px4-dev:v1.16.0-rc1-258-g0369abd556 make <target>
```

## Firmware Upload

Upload built `.px4` firmware to a connected board via USB bootloader:

```bash
python3 Tools/px_uploader.py --port /dev/ttyACM0 build/<target>/<target>.px4
```

The uploader sends a MAVLink reboot-to-bootloader command, then programs via the bootloader protocol (SYNC → GET_DEVICE → CHIP_ERASE → PROG_MULTI → GET_CRC → REBOOT).

## NSH Serial Console

NuttX Shell (NSH) access depends on the `SYS_USB_AUTO` parameter:

- **`SYS_USB_AUTO=1`** (auto-detect): Send 3 consecutive carriage returns (`\r\r\r` = `0x0D 0x0D 0x0D`) to trigger NSH on the USB ACM port. This is what PX4's `cdcacm_autostart` scans for (see `src/drivers/cdcacm_autostart/cdcacm_autostart.cpp:451`).
- **`SYS_USB_AUTO=2`** (default): MAVLink runs on USB. Use `Tools/mavlink_shell.py` or pymavlink `SERIAL_CONTROL` messages (devnum=10) to access NSH.

Baudrate: **57600**. Connect with `screen /dev/ttyACM0 57600` after setting `SYS_USB_AUTO=1`.

## Claude Code Skills

Six slash commands are available in `.claude/skills/`:

| Skill | Usage | What it does |
|-------|-------|-------------|
| `/build` | `/build cuav-x7pro` | Build firmware. Shortnames: `nxt-front`, `nxt-rear`, `cuav-wl`, `holybro`, `cuav-x7pro`, `cuav-nora`, `sitl`, `tests` |
| `/upload` | `/upload cuav-x7pro` | Upload `.px4` firmware to connected board via USB bootloader |
| `/nsh` | `/nsh ver all` | Run NSH command on connected board (requires `SYS_USB_AUTO=1`) |
| `/board-status` | `/board-status` | List connected PX4 boards via USB (VID:PID, device, mode) |
| `/board-diff` | `/board-diff cuav-x7pro cuav-wl` | Compare config files between two board variants |
| `/ulog` | `/ulog` or `/ulog download` or `/ulog analyze <file>` | Enable ulog, download latest `.ulg` from SD card, and analyze with pyulog |

## MCP Server

The `px4` MCP server (`Tools/mcp/px4_mcp_server.py`) provides tools for build, upload, NSH commands, serial port listing, device info, boot monitoring, HIL testing, and uORB listening.

**Setup**: Requires conda environment `px4-mcp`:

```bash
conda create -n px4-mcp python=3.11
conda activate px4-mcp
pip install -r Tools/mcp/requirements.txt   # mcp>=1.2.0, pyserial>=3.5
```

Config in `.mcp.json` points to `/home/frank/miniconda3/envs/px4-mcp/bin/python3`.

## Distributed Board Architecture

The wheel loader uses 4 board types. Modules are split across boards — know which board your module runs on before making changes.

| Board | Role | Key Modules |
|-------|------|-------------|
| cuav-x7plus-wl | Main coordinator | vla_proxy, operation_mode, health_monitor, arm_manager, operator_interface, strategy_executor, uorb_uart_bridge |
| holybro-v6xrt-wl | Alt coordinator (Ethernet) | Same as cuav-x7plus-wl |
| nxt-dual-wl-front | Front actuator control | drivetrain_controller, steering_controller, traction_controller, tilt_control, uorb_uart_proxy |
| nxt-dual-wl-rear | Rear actuator control | drivetrain_controller, steering_controller, traction_controller, boom_control, driver_lamp_controller, load_lamp_controller, uorb_uart_proxy |

Inter-board communication: `uorb_uart_bridge` (master on main board) ↔ `uorb_uart_proxy` (slave on sub-boards) serializes uORB topics over UART.

Full allocation table: `boards/wheel_loader/MODULE_ALLOCATION.md`

## Key Architecture Facts

- **Drivetrain**: 2 drivetrain components (front body, rear body), NOT 4 individual wheels
- **Actuation**: All-electric (H-bridge motor drivers + encoders). No hydraulics.
- **uORB**: Async pub/sub messaging for all inter-module communication. Topics defined in `msg/*.msg`, wheel loader topics in `msg/wheel_loader/`
- **Module pattern**: Most modules inherit `ModuleBase<T>` + `ModuleParams` + `px4::ScheduledWorkItem` or `px4::WorkItem`
- **Parameters**: Defined in `module.yaml` per module. Names max 16 chars, SNAKE_CASE with category prefix

## Module Structure

```cmake
px4_add_module(
    MODULE wheel_loader__my_module    # double underscore separator
    MAIN my_module                     # shell command name
    SRCS my_module.cpp
    DEPENDS uORB::topics
    MODULE_CONFIG module.yaml
)
```

Enable in board config (`.px4board`): `CONFIG_MODULES_<NAME>=y` or `CONFIG_DRIVERS_<NAME>=y`

## Serial Port Management

Ports are mapped through `module.yaml` `serial_config` sections and board `.px4board` Kconfig entries. `Tools/serial/generate_config.py` auto-generates startup scripts (`rc.serial`) and parameter definitions. Port indexes are immutable — never reuse or change them. Key port categories: GPS1-5 (201-205), TEL1-5 (101-105), UBR1-4 (801-804, UART bridge), SRV1-2 (601-602, smart servo).

Detailed serial port conventions are in `.github/copilot-instructions.md`.

## Code Style

- C/C++/CMake/Kconfig: tabs (width 8), max line 120
- YAML: spaces (indent 2)
- Shell: tabs (indent 2), max line 80
- Time: use literals (`100_ms`, `5_s`, `1_min`), never raw integers like `1000000`
- Logging: `PX4_INFO()`, `PX4_WARN()`, `PX4_ERR()`, `PX4_DEBUG()`

## Git Workflow

The `main` branch is protected. Always create a feature branch and open a PR.

**Before editing any code**, always check the current git branch (`git branch --show-current`). If on `main`, create a new feature branch first — never commit directly to `main`.

## CI

Workflows in `.github/workflows/` run on `frank_private_runner`:

- **build.yml**: Scans all board targets, builds in Docker matrix with ccache, packages artifacts
- **unit-test.yml**: Runs `make tests` + `make tests_coverage`, uploads to Codecov
- **sitl-test.yml**: SITL builds for iris, tailsitter, standard_vtol
- **functional-test.yml**: MAVROS mission tests, failsafe sim, ROS integration

## NXT-Dual Board Details

The NXT-Dual boards (front and rear) use STM32H743VI (Cortex-M7), board ID 1013.

### Serial Port Mapping (SERIAL_DISABLE_REORDERING=y)

With `CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y`, NuttX assigns `/dev/ttyS*` based on peripheral enable order, NOT renumbered:

| Device | Peripheral | Usage |
|--------|-----------|-------|
| /dev/ttyS0 | USART1 | uORB UART Proxy (inter-board comms) |
| /dev/ttyS1 | USART3 | Available |
| /dev/ttyS2 | UART4 | Available |
| /dev/ttyS3 | UART8 | NuttX console |

The mapping is confirmed in `boards/wheel_loader/nxt-dual-wl-front/src/board_config.h` (`PROXY_CLIENT_UART_PORT "/dev/ttyS0"`).

### Work Queue Stack Limits

The `hp_default` work queue has `SCHED_HPWORKSTACKSIZE=1280` in defconfig (2776 bytes actual at runtime). Modules running on this work queue (including `uorb_uart_proxy`) must keep stack usage minimal:
- Use class member variables for buffers, not stack-allocated arrays
- Avoid large local arrays (e.g., `TopicInfo *topics[256]` = 2048 bytes — this caused a stack overflow crash)
- Verified safe: 980/2776 bytes with all proxy buffers as class members

### Bringup Status (Feb 2026)

All modules compile and run on nxt-dual-wl-front. The following are auto-started at boot via `rc.board_extras`:
- `quadrature_encoder`, `hbridge`, `limit_sensor`

The following are compiled in but NOT auto-started (still commented out in `rc.board_extras`):
- `tilt_control` — needs physical tilt hardware connected
- `wheel_controller` — needs front wheel drive hardware
- `uorb_uart_proxy` — verified working via manual `uorb_uart_proxy start` on NSH; auto-start deferred until main board bridge is ready

## Common Pitfalls

- After pull: `git submodule update --init --recursive`
- uORB topic not found at build time: `make clean` (headers are generated)
- Module not building: verify it's enabled in the target's `.px4board`
- Frame size limit: NuttX enforces `-Wframe-larger-than=2048` — avoid large stack locals. For work queue items, keep stack well under the work queue's `SCHED_HPWORKSTACKSIZE`. Move buffers to class members.
- Serial config: `serial_config` in `module.yaml` must have a valid `name` field or the build's `generate_config.py` will fail
- NSH not responding on USB: check `SYS_USB_AUTO` — if set to 2, use MAVLink shell; if set to 1, send `\r\r\r`
- UART parameter min values: if a UART device param needs to allow `/dev/ttyS0` (index 0), ensure `module.yaml` sets `min: 0` and `default: 0`, not `min: 1`
- Docker build file ownership: Docker builds create root-owned files. Use `docker run ... chown -R $(id -u):$(id -g) build/` to fix permissions before host-side operations
