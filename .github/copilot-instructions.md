# SynapsePilot AI Coding Agent Instructions

## Project Overview

SynapsePilot is a PX4 v1.17.0-alpha1 fork focused on **multi-vehicle-type support** and **articulated chassis control**, specifically for wheel loader autonomous vehicles. This is a real-time embedded autopilot system combining NuttX RTOS with distributed board architecture.

## Git Workflow

**The `main` branch is protected.** Do not commit directly to `main`. Always:
1. Create a new feature branch first: `git checkout -b feature/your-feature-name`
2. Make your changes and commit to the feature branch
3. Push the feature branch and create a pull request to merge into `main`

## Build System

**All builds must use Docker environment** - do not build directly on host.

### Essential Build Commands

```bash
# SITL simulation (most common for development)
./Tools/docker_run.sh "make px4_sitl_default"

# Hardware targets (wheel loader boards)
./Tools/docker_run.sh "make wheel_loader_cuav-x7plus-wl_default"
./Tools/docker_run.sh "make wheel_loader_holybro-v6xrt-wl_default"
./Tools/docker_run.sh "make wheel_loader_nxt-dual-wl-front_default"
./Tools/docker_run.sh "make wheel_loader_nxt-dual-wl-rear_default"

# Clean build
./Tools/docker_run.sh "make clean"
./Tools/docker_run.sh "make distclean"  # Nuclear option - removes all build artifacts

# Upload to hardware (requires device access)
./Tools/docker_run.sh "make <target>_default upload"
```

### Docker Build Environment

**Recommended for consistent builds across platforms.** Use pre-built Docker images - **never build the image yourself**.

```bash
# Run build in container using helper script (auto-pulls image if needed)
./Tools/docker_run.sh "make px4_sitl_default"
./Tools/docker_run.sh "make wheel_loader_cuav-x7plus-wl_default"

# VS Code Dev Containers
# Open project in VS Code, it auto-detects .devcontainer/devcontainer.json
# Uses pre-built image: px4io/px4-dev-nuttx-focal:2022-08-12
```

**Docker images used:**
- Default NuttX builds: `px4io/px4-dev-nuttx-focal:2022-08-12`
- General builds: `px4io/px4-dev:v1.16.0-rc1-258-g0369abd556`
- Clang tools: `px4io/px4-dev-clang:2021-02-04`
- Simulation: `px4io/px4-dev-simulation-bionic:2021-12-11`

**Docker environment includes:**
- Cross-compilation toolchains (ARM GCC for NuttX)
- Python dependencies for build scripts
- CMake, Ninja, ccache
- Pre-configured ccache in `~/.ccache` (bind-mounted)

**Important**: `docker_run.sh` auto-mounts source with matching UID/GID to avoid permission issues.

### Build System Architecture

- **CMake-based** with custom PX4 functions in `cmake/px4_*.cmake`
- **Makefile wrapper** translates `make <target>_default` → CMake configure + build
- **Ninja** preferred (auto-detected), falls back to Unix Makefiles
- **Board configs**: `.px4board` files in `boards/<vendor>/<board>/` use Kconfig syntax
- **Module inclusion**: Modules enabled via `CONFIG_MODULES_<NAME>=y` or `CONFIG_DRIVERS_<NAME>=y` in `.px4board`

### Adding New Modules

Modules use `px4_add_module()` in CMakeLists.txt:

```cmake
px4_add_module(
    MODULE wheel_loader__my_module
    MAIN my_module          # Creates 'my_module' shell command
    SRCS
        my_module.cpp
        my_module.hpp
    DEPENDS
        uORB::topics       # Always needed for uORB
    MODULE_CONFIG
        module.yaml        # Parameter definitions
)
```

**Critical convention**: Module name format `<category>__<name>` (double underscore) where category is `modules`, `drivers`, `systemcmds`, or custom like `wheel_loader`.

## Architecture Patterns

### uORB Message Passing (Core Communication)

**uORB** is the async pub/sub system. All inter-module communication uses uORB topics defined in `msg/*.msg`.

```cpp
// Modern C++ style (preferred)
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>

uORB::Publication<vehicle_status_s> _status_pub{ORB_ID(vehicle_status)};
uORB::Subscription _battery_sub{ORB_ID(battery_status)};

// Publishing
vehicle_status_s msg{};
msg.timestamp = hrt_absolute_time();
msg.arming_state = 1;
_status_pub.publish(msg);

// Subscribing
battery_status_s bat{};
if (_battery_sub.update(&bat)) {
    // New data available
}
```

**Multi-instance topics** (for multiple sensors of same type):

```cpp
uORB::PublicationMulti<sensor_gps_s> _gps_pub{ORB_ID(sensor_gps)};
uORB::SubscriptionMultiArray<sensor_gps_s> _gps_subs{ORB_ID::sensor_gps};
```

### Module Execution Patterns

**WorkItem/ScheduledWorkItem** pattern (most modules):

```cpp
class MyModule : public ModuleBase<MyModule>, public px4::WorkItem
{
public:
    MyModule() : WorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers) {}
    
    void Run() override {
        // Called when scheduled
        if (should_exit()) { return; }
        
        // Do work, publish results
        ScheduleDelayed(100_ms); // Schedule next run
    }
};
```

**Work queues** in `px4::wq_configurations`: `rate_ctrl`, `nav_and_controllers`, `hp_default`, `lp_default`, `test1`, `test2`.

### Wheel Loader Distributed Architecture

**Critical**: Wheel loader uses **4 board types** with different module allocations (see `boards/wheel_loader/MODULE_ALLOCATION.md`):

1. **cuav-x7plus-wl / holybro-v6xrt-wl**: Main coordinator boards
   - Runs: `articulated_chassis` (vehicle coordination), `vla_proxy`, `arm_manager`, `health_monitor`, `operation_mode`, `strategy_executor`
   - `articulated_chassis` sub-modules:
     - `drivetrain_controller`: Controls both front and rear wheel speeds for the articulated body
     - `steering_controller`: Articulated steering angle control
     - `traction_controller`: Slip estimation & torque distribution
   
2. **nxt-dual-wl-front**: Front control board  
   - Runs: `tilt_control` (bucket tilt control)
   - Drivers: H-bridge motors, quadrature encoders, magnetic encoders, limit switches
   
3. **nxt-dual-wl-rear**: Rear control board
   - Runs: `boom_control` (boom control), `load_lamp_controller`, `driver_lamp_controller`
   - Drivers: H-bridge motors, quadrature encoders, magnetic encoders, limit switches, ST3215 smart servo (steering)

**Inter-board communication**: `uorb_uart_bridge` and `uorb_uart_proxy` modules serialize uORB topics over UART between boards.

**Important Notes**:
- **Drivetrain architecture**: Only **2 drivetrain components** (front and rear), NOT 4 individual wheels. Each drivetrain controls multiple wheels on that section.
- **Actuation system**: **All-electric actuation** using H-bridge motor drivers and encoders. There are NO hydraulic systems - boom and tilt use electric motors, not hydraulic cylinders.

### Wheel Loader Custom uORB Topics

All wheel loader topics in `msg/wheel_loader/*.msg`:
- `ChassisSetpoint`, `DrivetrainSetpoint`, `SteeringSetpoint`, `TractionSetpoint`
- `BoomSetpoint`, `TiltSetpoint` 
- `OperationModeStatus`, `StrategyStatus`
- `SensorMagEncoder`, `SensorQuadEncoder`, `SensorLimitSwitch`

## Critical Conventions

### Time Management

```cpp
#include <px4_platform_common/time.h>

hrt_absolute_time()  // Microseconds since boot (uint64_t)
```

**NEVER** use `1000000` for time conversions - use time literals:

```cpp
100_ms    // 100 milliseconds
5_s       // 5 seconds  
1_min     // 1 minute
```

### Parameter Naming

Parameters use `SNAKE_CASE` with category prefixes:
- `WL_TILT_KP` - Wheel Loader Tilt controller P gain
- `EKF2_AID_MASK` - EKF2 sensor fusion mask

Defined in `module.yaml` files within modules.

**Important Constraints**:
- **Parameter name length limit**: Maximum 16 characters (enforced by PX4 parameter system)
- **Units must be valid**: Use standard SI units or recognized abbreviations (e.g., `m/s`, `rad`, `deg`, `m/s^2`, `N`, `%`, `ms`, `s`). Invalid units will cause build or runtime errors.

### Shell Commands

Modules with `MAIN <name>` create shell commands. Access via:
```bash
# Run PX4 shell in Docker (for SITL)
./Tools/docker_run.sh "make px4_sitl_default"
# Then in nsh prompt:
my_module start
my_module status
my_module stop
```

### Board-Specific Code

Board headers in `boards/<vendor>/<board>/src/board_config.h` define GPIO, timers, etc.

```cpp
#include <board_config.h>  // Always include for hardware definitions
```

Timer configuration in `boards/<vendor>/<board>/src/timer_config.cpp` allocates PWM outputs.

## Testing & Debugging

**All testing, debugging, and running must be done in Docker environment** to ensure consistency.

### SITL Testing

```bash
# Always use docker_run.sh for SITL testing
./Tools/docker_run.sh "make px4_sitl default gazebo-classic"  # With Gazebo
./Tools/docker_run.sh "make px4_sitl_default jmavsim"         # Lightweight simulator

# Run tests in Docker
./Tools/docker_run.sh "make tests"
```

### Unit Tests

```cpp
#include <unit_test.h>

class MyTest : public UnitTest {
    bool run_tests() override;
};

// Register test
REGISTER_UNIT_TEST(MyTest)
```

```bash
# Run unit tests in Docker
./Tools/docker_run.sh "make test_unit"
```

### Logging

```cpp
#include <px4_platform_common/log.h>

PX4_INFO("Info message: %d", value);
PX4_WARN("Warning: %s", str);
PX4_ERR("Error occurred");
PX4_DEBUG("Debug only in debug builds");
```

## Integration Points

### MAVLink

MAVLink module in `src/modules/mavlink/` handles GCS communication. Custom messages require:
1. MAVLink XML definition in `mavlink/` submodule
2. Stream configuration in `mavlink_main.cpp`

### External Modules

Set `EXTERNAL_MODULES_LOCATION` to add out-of-tree modules:
```bash
make px4_sitl_default EXTERNAL_MODULES_LOCATION=/path/to/modules
```

## Common Pitfalls

1. **Submodule issues**: After pull, always `git submodule update --init --recursive`
2. **Module not building**: Check `.px4board` has `CONFIG_<CATEGORY>_<MODULE>=y`
3. **uORB topic not found**: Run `make clean` - uORB headers are generated
4. **Time conversions**: Use time literals (`100_ms`), not raw integers
5. **Build configuration**: `build/<target>/` contains generated config - check `make menuconfig` output

## Key Files for Reference

- `boards/wheel_loader/MODULE_ALLOCATION.md` - Module distribution across boards
- `platforms/WHEEL_LOADER_MODIFICATIONS.md` - Platform-specific wheel loader changes
- `cmake/px4_add_module.cmake` - Module build function documentation
- `platforms/common/uORB/uORB.h` - uORB API reference
- `src/modules/mavlink/` - MAVLink integration patterns

## Serial Port Management

**Serial ports are specially managed** through a centralized configuration system in `Tools/serial/`.

### Port Naming Convention

Ports are categorized by function (defined in `Tools/serial/generate_config.py`):
- **GPS1-GPS5**: GPS receivers (index 201-205)
- **TEL1-TEL5**: Telemetry/MAVLink links (index 101-105)
- **RC**: RC receiver input (index 300)
- **SRV1-SRV2**: Smart servo communication (index 601-602)
- **URT6**: Generic UART (index 6)
- **UBR1-UBR4**: UART bridge ports for WK2132 I2C-to-UART (index 801-804)
- **TST1**: Test port (index 701)

### Board Configuration (.px4board files)

Map logical ports to physical devices in `boards/<vendor>/<board>/default.px4board`:

```kconfig
CONFIG_BOARD_SERIAL_GPS1="/dev/ttyS0"
CONFIG_BOARD_SERIAL_TEL1="/dev/ttyS1"
CONFIG_BOARD_SERIAL_TEL2="/dev/ttyS3"
CONFIG_BOARD_SERIAL_SRV1="/dev/ttyS4"
```

**Available Kconfig options** (see `Kconfig` menu "Serial ports"):
- `CONFIG_BOARD_SERIAL_URT6`
- `CONFIG_BOARD_SERIAL_GPS1` through `GPS5`
- `CONFIG_BOARD_SERIAL_TEL1` through `TEL5`
- `CONFIG_BOARD_SERIAL_RC`
- `CONFIG_BOARD_SERIAL_WIFI`
- `CONFIG_BOARD_SERIAL_EXT2`

### Module Serial Configuration (module.yaml)

Add `serial_config` section to automatically generate startup scripts:

```yaml
module_name: my_module
serial_config:
    - command: my_module start -d ${SERIAL_DEV} -b ${BAUD}
      port_config_param:
        name: MY_MOD_DEV
        group: My Module
      baudrate_config_param:
        name: MY_MOD_BAUD
        group: My Module

parameters:
    - group: My Module
      definitions:
        MY_MOD_DEV:
            description:
                short: My Module Serial Port
            type: int32
            default: 0  # 0 = disabled
            # Values auto-filled from serial_ports dictionary
```

**Template variables available**:
- `${SERIAL_DEV}`: Device path (e.g., `/dev/ttyS1`)
- `${BAUD_PARAM}`: Parameter name for baudrate (e.g., `SER_GPS1_BAUD`)
- `${BAUD}`: Actual baudrate value
- `${i}`: Instance number (for multi-instance modules)

### Auto-Generated Files

The `Tools/serial/generate_config.py` script generates:

1. **ROMFS startup script** (`rc.serial` and `rc.serial_port`):
   - Iterates through configured ports
   - Sets `$SERIAL_DEV` and `$BAUD_PARAM` variables
   - Executes module commands from `module.yaml`

2. **Parameter definitions** (`serial_params.c`):
   - `SER_<TAG>_BAUD` parameters for each port
   - `<MODULE>_DEV` parameters for port selection

### Example: MAVLink Configuration

From `src/modules/mavlink/module.yaml`:

```yaml
serial_config:
    - command: |
        if [ $SERIAL_DEV != ethernet ]; then
            set MAV_ARGS "-d ${SERIAL_DEV} -b p:${BAUD_PARAM} -m p:MAV_${i}_MODE"
        else
            set MAV_ARGS "-u p:MAV_${i}_UDP_PRT -o p:MAV_${i}_REMOTE_PRT"
        fi
        mavlink start -i ${i} ${MAV_ARGS}
      port_config_param:
        name: MAV_${i}_CONFIG
      baudrate_config_param:
        name: SER_${PORT}_BAUD
      num_instances: *max_num_config_instances
```

### Example: Wheel Loader UART Bridge

From `src/wheel_loader/uorb_uart_proxy/module.yaml`:

```yaml
serial_config:
    - command: uorb_uart_proxy start -d ${SERIAL_DEV} -b ${BAUD}
      port_config_param:
        name: UORB_PX_DEV
      baudrate_config_param:
        name: UORB_PX_BAUD
```

Then in board config:
```kconfig
CONFIG_BOARD_SERIAL_UBR1="/dev/ttyS2"  # UART Bridge 1 for inter-board communication
```

### Important Notes

1. **Port indexes are immutable** - never reuse or change indexes (QGC compatibility)
2. **Default baudrate 0** means auto-detect (GPS modules)
3. **Ethernet support**: Use `ethernet_supported` flag, device becomes `"ethernet"`
4. **Multiple instances**: Use `num_instances` for modules like MAVLink with multiple concurrent ports
5. **Constrained flash**: Use `--constrained-flash` to reduce verbosity in generated scripts

## When Modifying Code

- **Hardware access**: Check `board_config.h` first for existing definitions
- **New messages**: Add to `msg/*.msg`, uORB auto-generates C++ headers
- **New modules**: Create CMakeLists.txt with `px4_add_module()`, enable in target `.px4board`
- **Cross-board features**: Consider uORB topic routing via UART bridge modules
- **Parameters**: Add to `module.yaml`, access via `param_find()` and `param_get()`
- **Serial ports**: Add `serial_config` to `module.yaml`, map in board's `.px4board`

## Documentation References

- PX4 Dev Guide: https://docs.px4.io/main/en/development/
- uORB Messaging: https://docs.px4.io/main/en/middleware/uorb.html
- Module Development: https://docs.px4.io/main/en/modules/
- Serial Config Generator: `Tools/serial/generate_config.py`
