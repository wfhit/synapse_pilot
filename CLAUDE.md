# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

SynapsePilot is a robot platform for VLA (Vision-Language-Action) models, built on open drone hardware. It forks PX4 v1.17.0-alpha1 and restructures it to support robot types far beyond drones.

**Why not ArduPilot or stock PX4?** ArduPilot supports many robot types but lacks PX4's modular architecture, making it hard to understand and extend. PX4 has great modularity but is drone-centric, making non-drone robots difficult. SynapsePilot takes PX4's modular base and separates robot-specific logic from the common platform, so adding a new robot type (like a wheel loader) means adding modules in `src/<robot_type>/` without touching the platform core.

The first supported robot type is an autonomous wheel loader (`src/wheel_loader/`), running on NuttX RTOS with a distributed multi-board architecture.

## Build Commands

**All builds must use Docker** — never build on the host directly.

```bash
# Wheel loader hardware targets
./Tools/docker_run.sh "make wheel_loader_cuav-x7plus-wl_default"      # Main coordinator
./Tools/docker_run.sh "make wheel_loader_holybro-v6xrt-wl_default"    # Alt coordinator (Ethernet)
./Tools/docker_run.sh "make wheel_loader_nxt-dual-wl-front_default"   # Front actuator board
./Tools/docker_run.sh "make wheel_loader_nxt-dual-wl-rear_default"    # Rear actuator board

# SITL simulation
./Tools/docker_run.sh "make px4_sitl_default"

# Tests
./Tools/docker_run.sh "make tests"
./Tools/docker_run.sh "make test_unit"

# Clean
./Tools/docker_run.sh "make clean"
./Tools/docker_run.sh "make distclean"
```

Docker image for NuttX builds: `px4io/px4-dev-nuttx-focal:2022-08-12`

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

## Code Style

- C/C++/CMake/Kconfig: tabs (width 8), max line 120
- YAML: spaces (indent 2)
- Shell: tabs (indent 2), max line 80
- Time: use literals (`100_ms`, `5_s`, `1_min`), never raw integers like `1000000`
- Logging: `PX4_INFO()`, `PX4_WARN()`, `PX4_ERR()`, `PX4_DEBUG()`

## Git Workflow

The `main` branch is protected. Always create a feature branch and open a PR.

## Common Pitfalls

- After pull: `git submodule update --init --recursive`
- uORB topic not found at build time: `make clean` (headers are generated)
- Module not building: verify it's enabled in the target's `.px4board`
- Frame size limit: NuttX enforces `-Wframe-larger-than=2048` — avoid large stack locals
- Serial config: `serial_config` in `module.yaml` must have a valid `name` field or the build's `generate_config.py` will fail
