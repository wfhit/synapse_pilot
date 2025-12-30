# Steering Controller

## Overview

The Steering Controller module provides steering control for articulated wheel loaders using the ST3125 servo controller. It implements position-based control with essential safety features.

## Key Features

- **ST3125 Servo Integration**: Direct position commands to ST3125 servo with internal PID control
- **Limit Sensors**: Hardware limit sensor integration for position safety
- **Safety Management**: Servo fault detection and emergency stop functionality
- **Timeout Handling**: Command and feedback timeout monitoring

## Architecture

### Control Loop
- 50Hz control rate for responsive steering
- Position commands sent to ST3125 servo
- Servo handles internal PID control and position feedback

### Safety Features
- Position limit monitoring with limit sensors
- Servo fault detection (overcurrent, temperature, communication)
- Command timeout handling (returns to center position)
- Emergency stop integration

## Parameters

### Basic Control
- `STEER_MAX_ANG`: Maximum steering angle (±45° default)

### ST3125 Servo Configuration
- `STEER_ST3125_ID`: CAN ID for the servo (1 default)
- `STEER_ST3125_CR`: Current limit protection (2.0A default)

### Limit Sensors
- `STEER_LT_LF_ID`: Left limit sensor instance ID (0 default)
- `STEER_LT_RT_ID`: Right limit sensor instance ID (1 default)

### Safety Timeouts
- `STEER_CMD_TO`: Command timeout (500ms default)
- `STEER_FB_TO`: Servo feedback timeout (100ms default)

## uORB Topics

### Subscriptions
- `steering_setpoint`: Steering angle commands
- `robotic_servo_status`: ST3125 servo status
- `limit_sensor`: Position limit sensor data
- `parameter_update`: Parameter updates

### Publications
- `robotic_servo_setpoint`: Commands to ST3125 servo

## Usage

### Starting the Module
```bash
steering_controller start
```

### Checking Status
```bash
steering_controller status
```

### Stopping the Module
```bash
steering_controller stop
```

## Implementation Details

### Control Pipeline
1. **Input Processing**: Read steering setpoints
2. **Safety Checks**: Verify position limits and sensor states
3. **Servo Command**: Send position command to ST3125

### Safety State Machine
- **Normal Operation**: All systems healthy, full control authority
- **Emergency Stop**: Fault detected, move to center position

## Troubleshooting

### Common Issues

1. **Servo Not Responding**
   - Check CAN connection and servo ID parameter
   - Verify servo power and configuration
   - Check feedback timeout counter

2. **Safety Violations**
   - Check limit sensor configuration
   - Verify sensor health

### Diagnostic Commands
```bash
# View detailed status
steering_controller status
```

## Integration Notes

- Requires ST3125 servo driver for CAN communication
- Integrates with limit sensor module for position safety
- Compatible with standard PX4 vehicle status topics
