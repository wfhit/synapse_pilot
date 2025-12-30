# PX4 Platform Modifications for Wheel Loader

This document details all platform-level modifications made to support the Wheel Loader project. These modifications extend PX4's hardware abstraction layer to support custom peripherals required by wheel loader hardware.

## Overview

The wheel loader implementation required extending PX4's platform support with drivers for external peripheral chips not available in standard autopilot hardware. These modifications are designed to be reusable across different STM32 and NXP platforms.

## Modified Platform Directories

```
platforms/nuttx/src/px4/
├── stm/
│   ├── stm32_common/          # Shared STM32 implementations
│   │   ├── include/px4_arch/
│   │   │   └── wk2132.h       # WK2132 register definitions & API
│   │   └── serial/
│   │       └── wk2132.c       # WK2132 I2C UART driver implementation
│   ├── stm32f1/include/px4_arch/
│   │   └── wk2132.h           # STM32F1 platform wrapper
│   ├── stm32f3/include/px4_arch/
│   │   └── wk2132.h           # STM32F3 platform wrapper
│   ├── stm32f4/include/px4_arch/
│   │   └── wk2132.h           # STM32F4 platform wrapper
│   ├── stm32f7/include/px4_arch/
│   │   └── wk2132.h           # STM32F7 platform wrapper
│   └── stm32h7/include/px4_arch/
│       └── wk2132.h           # STM32H7 platform wrapper
└── nxp/
    └── rt117x/
        ├── include/px4_arch/
        │   └── wk2132.h       # RT117x platform wrapper
        └── serial/
            └── wk2132.c       # RT117x-specific WK2132 implementation
```

---

## 1. WK2132 I2C-to-UART Bridge Driver

### Purpose

The **WK2132** is an I2C-to-Quad-UART bridge chip that provides up to **4 additional hardware UART ports** via a single I2C bus. This is critical for wheel loader applications that require extensive serial communication with:

- **Hydraulic actuators** (smart servo communication)
- **Implement controllers** (boom, tilt, bucket, etc.)
- **CAN-to-Serial bridges**
- **Diagnostic/debug interfaces**

Standard autopilot hardware typically has 6-8 UARTs, but wheel loader distributed control systems require 12+ serial ports.

### Features

- **4 independent UART channels** per chip (supports multiple chips on same I2C bus)
- **Full NuttX serial driver integration** - appears as standard `/dev/ttySx` devices
- **Hardware FIFOs** - 256-byte TX/RX FIFOs per channel for efficient buffering
- **Flexible baud rates** - 300 bps to 2 Mbps
- **Standard serial features**:
  - Configurable data bits (5-8)
  - Parity control (none, odd, even, mark, space)
  - Stop bits (1 or 2)
  - Hardware flow control (RTS/CTS)
- **Interrupt-driven operation** - efficient CPU usage with I2C polling worker
- **Multi-chip support** - up to 16 UARTs (4 WK2132 chips) on single I2C bus

### Hardware Integration

Typical wheel loader boards connect WK2132 via I2C:

```
┌─────────────────┐         ┌──────────────┐
│   STM32H7/      │  I2C    │   WK2132     │  UART1 → Boom Controller
│   RT117x        ├─────────┤              │  UART2 → Tilt Controller
│   Autopilot     │ SDA/SCL │  I2C→4×UART  │  UART3 → Hydraulic Servo
│                 │   INT   │              │  UART4 → Diagnostics
└─────────────────┘         └──────────────┘
```

### Implementation Details

#### Common Implementation (STM32 family)

**Location**: [platforms/nuttx/src/px4/stm/stm32_common/serial/wk2132.c](platforms/nuttx/src/px4/stm/stm32_common/serial/wk2132.c)

- **1470 lines** of fully interrupt-driven serial driver
- Implements standard NuttX `uart_ops_s` interface:
  - `setup`, `shutdown`, `attach`, `detach`
  - `receive`, `send`, `rxint`, `txint`
  - `rxavailable`, `txready`, `txempty`
  - `ioctl` (for termios configuration)

**Key Implementation Features**:

1. **FIFO-optimized transfers** - bulk I2C reads/writes for efficiency
2. **Automatic page switching** - handles WK2132's paged register architecture
3. **Error recovery** - frame errors, parity errors, FIFO overflow handling
4. **Dynamic baud rate calculation** - supports full range with prescaler logic
5. **Sleep mode support** - low-power operation when inactive

#### Register Definitions

**Location**: [platforms/nuttx/src/px4/stm/stm32_common/include/px4_arch/wk2132.h](platforms/nuttx/src/px4/stm/stm32_common/include/px4_arch/wk2132.h)

Complete WK2132 register map (262 lines):

- **Global registers**: GENA, GRST, GMUT, GIER, GIFR
- **Page 0 registers**: SCR, LCR, FCR, SIER, SIFR, TFCNT, RFCNT, FSR, LSR, FDAT
- **Page 1 registers**: BAUD1, BAUD0, PRES, RFTL, TFTL
- **Bit definitions** for all control and status fields
- **Public API** for initialization and device registration

#### Platform-Specific Headers

Each STM32 variant includes a simple wrapper that imports the common implementation:

```c
#pragma once
#include "../../../stm32_common/include/px4_arch/wk2132.h"
```

**Wrapper locations**:
- [platforms/nuttx/src/px4/stm/stm32f1/include/px4_arch/wk2132.h](platforms/nuttx/src/px4/stm/stm32f1/include/px4_arch/wk2132.h)
- [platforms/nuttx/src/px4/stm/stm32f3/include/px4_arch/wk2132.h](platforms/nuttx/src/px4/stm/stm32f3/include/px4_arch/wk2132.h)
- [platforms/nuttx/src/px4/stm/stm32f4/include/px4_arch/wk2132.h](platforms/nuttx/src/px4/stm/stm32f4/include/px4_arch/wk2132.h)
- [platforms/nuttx/src/px4/stm/stm32f7/include/px4_arch/wk2132.h](platforms/nuttx/src/px4/stm/stm32f7/include/px4_arch/wk2132.h)
- [platforms/nuttx/src/px4/stm/stm32h7/include/px4_arch/wk2132.h](platforms/nuttx/src/px4/stm/stm32h7/include/px4_arch/wk2132.h)

#### NXP RT117x Implementation

**Location**: [platforms/nuttx/src/px4/nxp/rt117x/serial/wk2132.c](platforms/nuttx/src/px4/nxp/rt117x/serial/wk2132.c)

Platform-specific implementation for NXP i.MX RT117x MCUs with RT-specific I2C handling.

**Header**: [platforms/nuttx/src/px4/nxp/rt117x/include/px4_arch/wk2132.h](platforms/nuttx/src/px4/nxp/rt117x/include/px4_arch/wk2132.h)

### Usage Example

In board initialization code (e.g., `boards/wheel_loader/nxt-dual-wl-rear/src/init.c`):

```c
#include <px4_arch/wk2132.h>

int board_app_initialize(uintptr_t arg)
{
    /* Initialize WK2132 on I2C1, base address 0x60, exposing 4 ports */
    /* Creates /dev/ttyS6, /dev/ttyS7, /dev/ttyS8, /dev/ttyS9 */
    int ret = wk2132_register_devices(
        1,      /* I2C bus 1 */
        0x60,   /* I2C address */
        6,      /* Base TTY number */
        4       /* Number of ports */
    );
    
    if (ret < 0) {
        syslog(LOG_ERR, "WK2132 initialization failed: %d\n", ret);
        return ret;
    }
    
    /* UARTs now available for module use */
    return OK;
}
```

In wheel loader modules:

```c
/* Open WK2132 UART for boom controller communication */
int fd = open("/dev/ttyS6", O_RDWR | O_NOCTTY);

struct termios tio;
tcgetattr(fd, &tio);
cfsetspeed(&tio, B115200);
tio.c_cflag = CS8 | CLOCAL | CREAD;
tcsetattr(fd, TCSANOW, &tio);

/* Use standard read()/write() calls */
write(fd, command_buffer, command_length);
```

### API Reference

#### Public Functions

```c
/**
 * Initialize a WK2132 UART port
 * @param i2c        I2C master interface
 * @param base_addr  I2C base address of WK2132 chip (0x60-0x6F)
 * @param port       UART port number (1-4)
 * @return           UART device structure or NULL on failure
 */
FAR struct uart_dev_s *wk2132_uart_init(
    FAR struct i2c_master_s *i2c,
    uint8_t base_addr,
    uint8_t port
);

/**
 * Register WK2132 serial devices as /dev/ttyS* devices
 * @param i2c_bus       I2C bus number (0-based)
 * @param i2c_base_addr I2C base address of WK2132 chip
 * @param base_tty      Base TTY number (e.g., 6 for /dev/ttyS6)
 * @param num_ports     Number of ports to register (1-4)
 * @return              OK on success, negative on failure
 */
int wk2132_register_devices(
    int i2c_bus,
    uint8_t i2c_base_addr,
    int base_tty,
    int num_ports
);

/**
 * Board-specific WK2132 initialization (typically called from board init)
 * Uses board-defined configuration macros
 * @return              OK on success, negative error code on failure
 */
int wk2132_board_init(void);

/**
 * Get WK2132 device path for a specific port
 * @param port_num      Port number (0-3)
 * @param path          Buffer to store device path
 * @param path_len      Length of path buffer
 * @return              OK on success, negative on error
 */
int wk2132_get_device_path(int port_num, char *path, size_t path_len);
```

### Configuration Macros

Boards define these in their board-specific headers:

```c
/* Example from boards/wheel_loader/nxt-dual-wl-rear/src/board_config.h */
#define BOARD_WK2132_I2C_BUS        1
#define BOARD_WK2132_I2C_ADDR       0x60
#define BOARD_WK2132_BASE_TTY       6
#define BOARD_WK2132_NUM_PORTS      4
```

### Register Architecture

The WK2132 uses a **paged register architecture**:

- **Global registers** (0x00-0x11): Chip-wide control
- **Channel-specific registers** accessed via:
  - **SPAGE** register (0x03) selects Page 0 or Page 1
  - Channel selection bits in I2C address (C1, C0)
  - **Page 0**: UART control, status, FIFO access
  - **Page 1**: Baud rate, FIFO trigger levels

**Example register access pattern**:
```
1. Write SPAGE = 0x00 (select Page 0)
2. Write to address 0x04 (C1=0,C0=0) → UART1 SCR register
3. Write to address 0x06 (C1=0,C0=1) → UART2 SCR register
4. Write SPAGE = 0x01 (select Page 1)
5. Write to address 0x04 (C1=0,C0=0) → UART1 BAUD1 register
```

---

## 2. Board-Specific Implementations

### Timer Configuration

All wheel loader boards define custom timer configurations for PWM output to hydraulic valves and motor controllers.

**Files**:
- [boards/wheel_loader/cuav-x7plus-wl/src/timer_config.cpp](boards/wheel_loader/cuav-x7plus-wl/src/timer_config.cpp)
- [boards/wheel_loader/holybro-v6xrt-wl/src/timer_config.cpp](boards/wheel_loader/holybro-v6xrt-wl/src/timer_config.cpp)
- [boards/wheel_loader/nxt-dual-wl-front/src/timer_config.cpp](boards/wheel_loader/nxt-dual-wl-front/src/timer_config.cpp)
- [boards/wheel_loader/nxt-dual-wl-rear/src/timer_config.cpp](boards/wheel_loader/nxt-dual-wl-rear/src/timer_config.cpp)

**Purpose**: Define PWM outputs for:
- Proportional hydraulic valve control
- Motor speed controllers
- LED/lamp brightness control

---

## 3. Integration with Wheel Loader Modules

### Module Dependencies on Platform Extensions

The following wheel loader modules directly depend on these platform modifications:

#### WK2132 Serial Dependencies

| Module | Device Usage | Purpose |
|--------|--------------|---------|
| `boom_control` | `/dev/ttySx` | Boom actuator serial commands |
| `tilt_control` | `/dev/ttySx` | Tilt actuator serial commands |
| `arm_manager` | `/dev/ttySx` | Multi-implement coordination |
| `interactor_interface` | `/dev/ttySx` | External device communication |
| `articulated_chassis/*_controller` | `/dev/ttySx` | Low-level controller bridge |

#### Timer/PWM Dependencies

| Module | Timer Usage | Purpose |
|--------|-------------|---------|
| `boom_control` | PWM outputs | Proportional valve control |
| `tilt_control` | PWM outputs | Proportional valve control |
| `lamp_*_controller` | PWM outputs | LED brightness (work lights, signals) |

---

## 4. Testing Platform Modifications

### WK2132 Verification

Test script example (run on target hardware):

```bash
# Check device creation
ls -l /dev/ttyS*

# Test loopback (requires hardware TX-RX jumper)
stty -F /dev/ttyS6 115200 cs8 -parenb -cstopb
echo "test" > /dev/ttyS6 &
cat /dev/ttyS6
```

### Debugging I2C Communication

Enable I2C debug logging in NuttX configuration:

```
CONFIG_DEBUG_I2C=y
CONFIG_DEBUG_I2C_INFO=y
```

Monitor I2C transactions:
```bash
dmesg | grep -i "wk2132\|i2c"
```

---

## 5. Future Platform Extensions

### Potential Additions

1. **CAN-based serial bridges** - Support for CAN-to-UART adapters
2. **Quadrature encoder drivers** - For precise position feedback (STM32 timer QEI mode)
3. **Custom ADC mappings** - Hydraulic pressure sensors, load cells
4. **Industrial I/O expanders** - PCA9685 (PWM), MCP23017 (GPIO)

### Guidelines for Adding Platform Extensions

When adding new platform-level drivers:

1. **Place in appropriate platform directory**:
   - STM32 common: `platforms/nuttx/src/px4/stm/stm32_common/`
   - Platform-specific: `platforms/nuttx/src/px4/stm/<variant>/`

2. **Follow NuttX driver conventions**:
   - Implement standard device interfaces (`uart_dev_s`, `file_operations`, etc.)
   - Use NuttX semaphores and work queues for concurrency
   - Support NuttX configuration system (Kconfig)

3. **Create platform wrappers**:
   - Add headers in each relevant platform's `include/px4_arch/`
   - Allow compile-time platform selection

4. **Document thoroughly**:
   - Update this file with new driver details
   - Add usage examples
   - Document hardware connections

5. **Test across platforms**:
   - Verify on multiple STM32 variants if possible
   - Test with different I2C/SPI bus speeds
   - Validate error handling (bus errors, timeouts)

---

## Summary

### What Was Added

| Component | Files Added | Purpose | Lines of Code |
|-----------|-------------|---------|---------------|
| WK2132 Driver | 9 files | I2C-to-Quad-UART serial expansion | ~1700 LOC |
| Timer Configs | 4 files | Board-specific PWM configurations | ~200 LOC/board |

### Platform Coverage

- ✅ **STM32F1** - Legacy support
- ✅ **STM32F3** - Legacy support  
- ✅ **STM32F4** - Common autopilot platform
- ✅ **STM32F7** - High-performance autopilots
- ✅ **STM32H7** - Current-generation autopilots (CUAV X7+, Holybro V6X)
- ✅ **NXP RT117x** - Real-time embedded platform (NXT boards)

### Key Benefits

1. **Reusability** - WK2132 driver works across all supported platforms
2. **Maintainability** - Common implementation, platform-specific wrappers
3. **Scalability** - Supports multiple chips, up to 16 additional UARTs
4. **Standard compliance** - Full NuttX serial driver interface
5. **Performance** - Interrupt-driven with hardware FIFOs

---

## References

- **WK2132 Datasheet**: I2C to Quad UART Bridge IC specification
- **NuttX Serial Driver Guide**: `nuttx/drivers/serial/README.txt`
- **PX4 Platform Architecture**: `platforms/nuttx/README.md`
- **Wheel Loader Module Allocation**: [boards/wheel_loader/MODULE_ALLOCATION.md](boards/wheel_loader/MODULE_ALLOCATION.md)

---

**Document Version**: 1.0  
**Last Updated**: 2025  
**Maintained By**: PX4 Wheel Loader Development Team
