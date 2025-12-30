# Wheel Loader Module Allocation Reference

## Board Architecture Overview

The wheel loader system uses a distributed control architecture with four board types:

1. **CUAV X7Plus-WL** - Main vehicle coordinator (STM32H7, WK2132 multi-UART)
2. **Holybro V6XRT-WL** - Main vehicle coordinator (i.MX RT1176, Ethernet)
3. **NXT-Dual-WL-Front** - Front wheel & tilt/bucket control (STM32H7)
4. **NXT-Dual-WL-Rear** - Rear wheel & boom control (STM32H7)

---

## Complete Module Allocation Table

### Wheel Loader Specific Modules

| Module | Description | cuav-x7plus-wl | holybro-v6xrt-wl | nxt-dual-wl-front | nxt-dual-wl-rear |
|--------|-------------|:--------------:|:----------------:|:-----------------:|:----------------:|
| **articulated_chassis** | Whole vehicle coordination & wheel control | ✅ | ✅ | ❌ | ❌ |
| ├─ **drivetrain_controller** | Individual wheel speed control with PID | ✅ | ✅ | ❌ | ❌ |
| ├─ **steering_controller** | Articulated steering angle control | ✅ | ✅ | ❌ | ❌ |
| └─ **traction_controller** | Slip estimation & torque distribution | ✅ | ✅ | ❌ | ❌ |
| **vla_proxy** | VLA (Vehicle Level Autonomy) interface | ✅ | ✅ | ❌ | ❌ |
| **arm_manager** | System arming/disarming & safety | ✅ | ✅ | ❌ | ❌ |
| **health_monitor** | System-wide health monitoring | ✅ | ✅ | ❌ | ❌ |
| **interactor_interface** | RC/MAVLink command handling | ✅ | ✅ | ❌ | ❌ |
| **operation_mode** | Mode management (manual/auto/etc) | ✅ | ✅ | ❌ | ❌ |
| **strategy_executor** | High-level automation strategies | ✅ | ✅ | ❌ | ❌ |
| **tilt_control** | Bucket tilt hydraulic control | ❌ | ❌ | ✅ | ❌ |
| **boom_control** | Boom hydraulic control | ❌ | ❌ | ❌ | ✅ |
| **load_lamp_controller** | Work area lighting control | ❌ | ❌ | ❌ | ✅ |
| **driver_lamp_controller** | Operator lighting control | ❌ | ❌ | ❌ | ✅ |
| **nav_lamp_controller** | Status/navigation lighting | ✅ | ✅ | ❌ | ❌ |
| **uorb_uart_bridge** | Main board uORB bridge (master) | ✅ | ✅ | ❌ | ❌ |
| **uorb_uart_proxy** | Sub-board uORB proxy (slave) | ✅ | ✅ | ✅ | ❌ |

### Wheel Loader Specific Drivers

| Driver | Description | cuav-x7plus-wl | holybro-v6xrt-wl | nxt-dual-wl-front | nxt-dual-wl-rear |
|--------|-------------|:--------------:|:----------------:|:-----------------:|:----------------:|
| **hbridge** | H-bridge motor drivers (DRV8701) | ❌ | ❌ | ✅ | ✅ |
| **quadrature_encoder** | Quadrature encoder position feedback | ❌ | ❌ | ✅ | ✅ |
| **limit_sensor** | Safety limit switches | ❌ | ❌ | ✅ | ✅ |
| **mag_encoder_as5600** | AS5600 magnetic encoder | ❌ | ❌ | ✅ | ✅ |
| **smart_servo_st3215** | ST3215 smart servo (boom) | ❌ | ❌ | ❌ | ✅ |

### Standard PX4 Modules (Common to All)

| Module | cuav-x7plus-wl | holybro-v6xrt-wl | nxt-dual-wl-front | nxt-dual-wl-rear |
|--------|:--------------:|:----------------:|:-----------------:|:----------------:|
| **attitude_estimator_q** | ❌ | ❌ | ✅ | ✅ |
| **battery_status** | ✅ | ✅ | ❌ | ❌ |
| **dataman** | ✅ | ✅ | ✅ | ✅ |
| **ekf2** | ✅ | ✅ | ✅ | ✅ |
| **esc_battery** | ✅ | ✅ | ❌ | ❌ |
| **events** | ✅ | ✅ | ✅ | ✅ |
| **gyro_calibration** | ✅ | ✅ | ✅ | ✅ |
| **gyro_fft** | ❌ | ❌ | ✅ | ✅ |
| **landing_target_estimator** | ✅ | ✅ | ✅ | ✅ |
| **load_mon** | ✅ | ✅ | ✅ | ✅ |
| **local_position_estimator** | ✅ | ✅ | ✅ | ✅ |
| **logger** | ✅ | ✅ | ✅ | ✅ |
| **mag_bias_estimator** | ✅ | ✅ | ✅ | ✅ |
| **manual_control** | ✅ | ✅ | ❌ | ❌ |
| **mavlink** | ✅ | ✅ | ✅ | ✅ |
| **rc_update** | ✅ | ✅ | ❌ | ❌ |
| **sensors** | ✅ | ✅ | ✅ | ✅ |
| **temperature_compensation** | ✅ | ✅ | ✅ | ✅ |
| **uxrce_dds_client** | ✅ | ✅ | ✅ | ✅ |

---

## Board-Specific Details

### CUAV X7Plus-WL (Main Coordinator)
**Role:** Primary vehicle coordinator with WK2132 multi-UART expansion
**Key Features:**
- GPS, RC input, MAVLink telemetry
- CAN bus coordination
- WK2132 multi-UART for distributed sub-controllers
- Full sensor suite (GPS, IMUs, barometers, magnetometers)

**Modules:** 
- Coordination: articulated_chassis, vla_proxy, uorb_uart_bridge, uorb_uart_proxy
- Management: arm_manager, health_monitor, interactor_interface, operation_mode, strategy_executor
- Lighting: nav_lamp_controller
- Standard PX4: Full suite with GPS, RC, MAVLink

---

### Holybro V6XRT-WL (Main Coordinator)
**Role:** Alternative main vehicle coordinator with Ethernet connectivity
**Key Features:**
- Ethernet connectivity
- GPS, RC input, MAVLink telemetry
- CAN bus coordination
- Full sensor suite (GPS, IMUs, barometers, magnetometers)

**Modules:** 
- Coordination: articulated_chassis, vla_proxy, uorb_uart_bridge, uorb_uart_proxy
- Management: arm_manager, health_monitor, interactor_interface, operation_mode, strategy_executor
- Lighting: nav_lamp_controller
- Standard PX4: Full suite with GPS, RC, MAVLink, Ethernet

---

### NXT-Dual-WL-Front (Front Actuator Controller)
**Role:** Front wheel and bucket/tilt control
**Hardware:**
- AS5600 magnetic encoder (tilt angle)
- Quadrature encoder (front wheel position)
- H-bridge motor drivers (DRV8701)
- Limit sensors (bucket load/dump positions)
- BMI088 IMU, SPL06 barometer

**Modules:**
- Actuator Control: tilt_control
- Communication: uorb_uart_proxy (receives commands from main board)
- Drivers: hbridge, quadrature_encoder, limit_sensor, mag_encoder_as5600
- Standard PX4: Minimal set (attitude, sensors, logging)

**Communications:** 
- Receives commands via uORB UART from main board
- Publishes sensor/actuator status back to main board

---

### NXT-Dual-WL-Rear (Rear Actuator Controller)
**Role:** Rear wheel and boom control with work lighting
**Hardware:**
- AS5600 magnetic encoder (boom position)
- ST3215 smart servo (boom actuation)
- Quadrature encoder (rear wheel position)
- H-bridge motor drivers
- Load lamps (work area lighting)
- Driver lamps (operator lighting)
- Limit sensors (boom position)
- BMI088 IMU, SPL06 barometer

**Modules:**
- Actuator Control: boom_control, load_lamp_controller, driver_lamp_controller
- Drivers: hbridge, quadrature_encoder, limit_sensor, mag_encoder_as5600, smart_servo_st3215
- Standard PX4: Minimal set (attitude, sensors, logging)

**Communications:** 
- Receives commands via MAVLink or direct uORB subscriptions
- Publishes sensor/actuator status

---

## Communication Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Main Controller Board                      │
│            (CUAV X7Plus-WL / Holybro V6XRT-WL)              │
│                                                               │
│  Modules: articulated_chassis, vla_proxy, arm_manager,      │
│           health_monitor, interactor_interface,              │
│           operation_mode, strategy_executor,                 │
│           nav_lamp_controller, uorb_uart_bridge              │
│                                                               │
│  Inputs: GPS, RC, MAVLink, CAN, Ethernet                    │
└───────────────┬─────────────────────────────┬───────────────┘
                │ uORB via UART               │
                │ (Commands)                  │
                │                             │
    ┌───────────▼─────────────┐   ┌──────────▼──────────────┐
    │  NXT-Dual-WL-Front      │   │  NXT-Dual-WL-Rear       │
    │  (Front Actuator)       │   │  (Rear Actuator)        │
    │                         │   │                          │
    │  Modules:               │   │  Modules:                │
    │  - tilt_control         │   │  - boom_control          │
    │  - uorb_uart_proxy      │   │  - load_lamp_controller  │
    │                         │   │  - driver_lamp_controller│
    │  Drivers:               │   │                          │
    │  - hbridge              │   │  Drivers:                │
    │  - quadrature_encoder   │   │  - hbridge               │
    │  - limit_sensor         │   │  - quadrature_encoder    │
    │  - mag_encoder_as5600   │   │  - limit_sensor          │
    │                         │   │  - mag_encoder_as5600    │
    │  Controls:              │   │  - smart_servo_st3215    │
    │  - Front wheel drive    │   │                          │
    │  - Bucket tilt          │   │  Controls:               │
    └─────────────────────────┘   │  - Rear wheel drive      │
                                  │  - Boom hydraulics       │
                                  │  - Work/driver lamps     │
                                  └──────────────────────────┘
```

---

## Module Dependencies & Requirements

### System Management (Main Boards Only)
- **arm_manager** - Requires: health_monitor_status, operation_mode_status
- **health_monitor** - Publishes: health_monitor_status
- **interactor_interface** - Requires: RC input, MAVLink
- **operation_mode** - Publishes: operation_mode_status
- **strategy_executor** - Requires: health_monitor, operation_mode

### Coordination (Main Boards Only)
- **articulated_chassis** - Requires: GPS, IMU, wheel feedback
- **vla_proxy** - Interface to external VLA systems
- **uorb_uart_bridge** - Master side of distributed uORB

### Actuator Control
- **tilt_control** - Requires: mag_encoder_as5600, quadrature_encoder, hbridge, limit_sensor
- **boom_control** - Requires: mag_encoder_as5600, smart_servo_st3215, limit_sensor

---

## Build Targets

```bash
# Main coordinator boards
make wheel_loader_cuav-x7plus-wl_default
make wheel_loader_holybro-v6xrt-wl_default

# Sub-controller boards
make wheel_loader_nxt-dual-wl-front_default
make wheel_loader_nxt-dual-wl-rear_default
```

---

## Configuration Files

- **CUAV X7Plus-WL:** `boards/wheel_loader/cuav-x7plus-wl/default.px4board`
- **Holybro V6XRT-WL:** `boards/wheel_loader/holybro-v6xrt-wl/default.px4board`
- **NXT-Dual-WL-Front:** `boards/wheel_loader/nxt-dual-wl-front/default.px4board`
- **NXT-Dual-WL-Rear:** `boards/wheel_loader/nxt-dual-wl-rear/default.px4board`

---

## Kconfig Integration

All wheel loader modules are integrated via:
- **Main Kconfig:** `src/wheel_loader/Kconfig`
- **Module Kconfig:** Referenced from `src/modules/Kconfig`

Individual module Kconfig files:
- `src/wheel_loader/articulated_chassis/Kconfig`
  - `src/wheel_loader/articulated_chassis/drivetrain_controller/Kconfig`
  - `src/wheel_loader/articulated_chassis/steering_controller/Kconfig`
  - `src/wheel_loader/articulated_chassis/traction_controller/Kconfig`
- `src/wheel_loader/boom_control/Kconfig`
- `src/wheel_loader/tilt_control/Kconfig`
- `src/wheel_loader/arm_manager/Kconfig`
- `src/wheel_loader/health_monitor/Kconfig`
- `src/wheel_loader/interactor_interface/Kconfig`
- `src/wheel_loader/operation_mode/Kconfig`
- `src/wheel_loader/strategy_executor/Kconfig`
- `src/wheel_loader/load_lamp_controller/Kconfig`
- `src/wheel_loader/driver_lamp_controller/Kconfig`
- `src/wheel_loader/nav_lamp_controller/Kconfig`
- `src/wheel_loader/uorb_uart_bridge/Kconfig`
- `src/wheel_loader/uorb_uart_proxy/Kconfig`
- `src/wheel_loader/vla_proxy/Kconfig`

---

**Document Version:** 1.0  
**Last Updated:** December 30, 2025  
**Status:** ✅ All modules correctly allocated and verified
