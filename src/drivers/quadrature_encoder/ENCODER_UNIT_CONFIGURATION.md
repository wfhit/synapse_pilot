# Encoder Unit Configuration

This document explains the parameter system for quadrature encoder configuration and how to achieve proper unit conversion.

## Parameter Overview

The quadrature encoder driver uses two main types of parameters:

### Overflow Count Parameter (`QENC_X_PPR`)

The overflow count parameter controls when the hardware counter automatically wraps around:
- **Purpose**: Prevents counter overflow in hardware
- **Value**: Maximum counter value before reset (0 = no auto-reset)
- **Units**: Pulse count
- **Example**: For a 1024-pulse encoder with full revolution tracking = `QENC0_PPR = 1024`

> **Note**: Small overflow values (< 100) can cause frequent counter wraps. If your encoder has low overflow count, ensure the resolution parameter compensates appropriately for desired output precision.

### Resolution Parameter (`QENC_X_RES`)

The resolution parameter defines the conversion from encoder pulses to desired output units:
- **Purpose**: Convert raw pulse counts to meaningful position/velocity units
- **Value**: Units per pulse (e.g., radians per pulse, meters per pulse)
- **Units**: [desired_units]/pulse
- **Example**: For radians output = `QENC0_RES = 0.000006135` (2π/1024)

## Unit Calculation Formula

```
Final Position = (Delta Pulses) × Resolution Parameter
Final Velocity = (Delta Pulses / Delta Time) × Resolution Parameter
```

## Parameter Interaction

- **Overflow Count Parameter**: Controls hardware counter wraparound behavior
- **Resolution Parameter**: Conversion factor from pulses to desired units
- **Direction Reverse Parameter**: Multiplies output by -1 when enabled

## Configuration Examples

### Rotational Encoder (1024 PPR, output in radians)
```
QENC0_PPR = 1024        # Counter wraps at 1024 (one full revolution)
QENC0_RES = 0.006135922 # 2π/1024 radians per pulse
```

### Linear Encoder (500 pulses/mm, output in meters)
```
QENC0_PPR = 0           # No counter wrap (continuous linear motion)
QENC0_RES = 0.000002    # 0.002mm = 0.000002m per pulse
```

### High-Resolution Angular (10000 PPR, output in degrees)
```
QENC0_PPR = 10000       # Counter wraps at 10000
QENC0_RES = 0.036       # 360°/10000 = 0.036 degrees per pulse
```

## Implementation Notes

### Parameter Naming Clarification

- **`QENC_X_PPR`** (formerly `pulses_per_revolution`): Actually controls counter overflow behavior
- **`QENC_X_RES`** (formerly `scale`): Represents the true encoder resolution in units per pulse

This naming makes the parameter purposes more intuitive:
- **Overflow count**: When the counter should wrap around
- **Resolution**: What each pulse represents in real-world units

### Direction Control

The `QENC_X_REV` parameter reverses the direction interpretation:
- `0`: Normal direction (default)
- `1`: Reversed direction (multiply output by -1)

### Hardware Integration

The driver interfaces with the platform layer which handles:
- GPIO interrupt configuration
- Counter overflow detection
- Thread-safe data access
- Event-based position reset via `QuadEncoderReset` messages

### Message Output

The `sensor_quad_encoder` message contains:
- `timestamp`: High-resolution timestamp
- `instance`: Encoder instance ID
- `position`: Accumulated position in configured units
- `velocity`: Instantaneous velocity in configured units per second

All calculations use double precision to maintain accuracy over extended operation.

## Current Message Units

The `sensor_quad_encoder` message publishes data with these units:

### Position Field
- **Rotary Encoders**: `1/million radians` (micro-radians)
- **Linear Encoders**: `millimeters`
- **Data Type**: `float64` for extended range and precision

### Velocity Field
- **Rotary Encoders**: `1/million radians per second` (micro-radians/s)
- **Linear Encoders**: `millimeters per second`
- **Data Type**: `float64` for precision at low speeds

## Configuration Examples

### Example 1: Rotary Encoder (Wheel Position)
```
Hardware: 1024 PPR optical encoder on vehicle wheel
Wheel radius: 0.3 meters
Desired output: Position in micro-radians

Configuration:
QE_0_RESOLUTION = 1024.0     # Encoder hardware specification
QE_0_SCALE = 6135.923        # (2π / 1024) × 1,000,000 for micro-radians
```

### Example 2: Linear Encoder (Boom Extension)
```
Hardware: Linear encoder with 0.1mm resolution
Desired output: Position in millimeters

Configuration:
QE_0_RESOLUTION = 10.0       # 10 pulses per mm
QE_0_SCALE = 0.1            # 0.1 mm per pulse
```

### Example 3: High-Resolution Rotary Encoder
```
Hardware: 4096 PPR magnetic encoder
Desired output: Position in micro-radians

Configuration:
QE_0_RESOLUTION = 4096.0     # Encoder specification
QE_0_SCALE = 1533.981        # (2π / 4096) × 1,000,000 for micro-radians
```

### Example 4: Low-Resolution Encoder (Compensation with Scale)
```
Hardware: 64 PPR simple encoder (low resolution)
Desired output: Position in micro-radians with good precision

Configuration:
QE_0_RESOLUTION = 64.0       # Low encoder specification
QE_0_SCALE = 98174.77        # (2π / 64) × 1,000,000 for micro-radians

Note: Large scale value compensates for low resolution, providing
      98,175 micro-radians (≈5.6°) per pulse for adequate precision
```

## Parameter Tuning Guidelines

### Step 1: Set Resolution Parameter
- Use the actual hardware specification of your encoder
- For rotary: pulses per revolution (PPR)
- For linear: pulses per millimeter
- **Important**: Prefer resolution values ≥ 100 when possible
- If hardware resolution < 100, consider using gear ratios or optical multiplication

### Step 2: Verify Units
- Check that `position` values are reasonable for your application
- Verify `velocity` calculations match expected ranges
- Test with known movements to validate scaling
- For low-resolution encoders, ensure scale provides adequate granularity

## Advanced Configuration

### Counter Overflow Handling
- Automatic wraparound at resolution boundary
- Delta calculation handles overflow transparently
- Position accumulates continuously beyond single revolution

## Troubleshooting

### Velocity Seems Incorrect
- Check scale parameter calculation
- Verify resolution matches encoder specification
- Ensure adequate polling rate (100Hz recommended)

### Low Resolution Encoder Issues
- **Symptom**: Jerky or quantized position readings
- **Cause**: Resolution < 100 PPR provides insufficient granularity
- **Solutions**:
  - Increase scale parameter to amplify small movements
  - Consider mechanical gear ratios to multiply effective resolution
  - Use interpolation techniques in application layer if needed
  - Upgrade to higher-resolution encoder when possible

### Frequent Counter Overflows
- **Symptom**: Unexpected position jumps or resets
- **Cause**: Very low resolution values cause frequent wraparounds
- **Solutions**:
  - Increase resolution parameter if encoder supports it
  - Disable auto-reset by setting resolution to 0
  - Handle overflow in application layer with continuous accumulation

## Implementation Notes

### Thread Safety
- Parameters can be updated at runtime
- Changes take effect on next encoder reading
- No restart required for parameter updates

### Performance Considerations
- Scale calculation happens at full polling rate
- Use appropriate scale values to avoid precision loss
- Higher resolution encoders provide better accuracy

---

**Note**: This configuration system provides maximum flexibility while maintaining real-time performance and precision suitable for robotics applications.
