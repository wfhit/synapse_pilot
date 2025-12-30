# NXT-Dual-WL-Front Board

## Overview

The NXT-Dual-WL-Front is a specialized controller board for wheel loader applications, designed to control:
- **Front wheel** - Steering and propulsion control for the front axle
- **Tilt** - Hydraulic tilt control for digging and loading operations

This board is based on the nxt-dual-wl platform and includes specialized modules and configurations for front-end wheel loader operations.

## Key Features

- Dual wheel quadrature encoder support for precise front wheel control
- AS5600 magnetic encoder for tilt position feedback
- H-bridge motor drivers (DRV8701) for tilt hydraulic control
- BMI088 IMU for attitude and motion sensing
- SPL06 barometric pressure sensor
- CAN bus communication for coordination with rear controller
- PWM outputs for servo and motor control

## Modules Enabled

- **tilt_control** - Dedicated tilt position and hydraulic control
- **front_wheel_control** - Front axle steering and drive control
- Standard PX4 navigation and control modules

## Pin Configuration

### PWM Outputs
- PWM1-2: Front wheel motor control
- PWM3-4: Bucket hydraulic valve control
- PWM5-8: Auxiliary outputs

### I2C Bus
- I2C1: AS5600 magnetic encoder (bucket position)
- I2C2: External sensor expansion

### CAN Bus
- CAN1: Inter-controller communication (front ↔ rear)
- CAN2: External device communication

## Parameters

Key parameters for front wheel loader control:
- `TILT_CTRL_EN`: Enable tilt control module
- `WL_FRONT_WHEEL_EN`: Enable front wheel control
- `WL_FRONT_WHEEL_KP/KI/KD`: Front wheel PID parameters

## Usage

This board should be mounted in the front section of the wheel loader and connected to:
1. Front wheel drive motors
2. Bucket hydraulic valves
3. Front wheel position encoders
4. Bucket position sensor
5. CAN bus to rear controller
