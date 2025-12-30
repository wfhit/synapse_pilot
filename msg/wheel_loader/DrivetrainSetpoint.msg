# Wheel setpoint for individual wheel control

uint64 timestamp               # Time since system start (microseconds)

float32 wheel_speed_rad_s      # Desired wheel speed in rad/s
float32 wheel_acceleration_rad_s2  # Maximum acceleration in rad/s²
float32 wheel_torque_nm        # Desired wheel torque in Nm (alternative to speed)

# Control mode
uint8 control_mode             # Control mode selection
bool enable_traction_control   # Enable traction control for this wheel
bool emergency_stop            # Emergency stop this wheel immediately

# Control mode constants
uint8 MODE_DISABLED = 0
uint8 MODE_SPEED_CONTROL = 1
uint8 MODE_TORQUE_CONTROL = 2
uint8 MODE_COAST = 3

# Safety limits
float32 speed_limit_rad_s      # Absolute speed limit for safety
float32 torque_limit_nm        # Absolute torque limit for safety
