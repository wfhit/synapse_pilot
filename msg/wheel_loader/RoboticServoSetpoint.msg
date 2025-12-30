# Robotic servo setpoint message for smart servos like ST3125

uint64 timestamp          # time since system start (microseconds)
uint8 id                  # Servo ID (1-253)
uint8 command_type        # Command type: 0=position, 1=velocity, 2=current, 3=extended_position

# Position control parameters
float32 goal_position     # Target position (radians)
float32 goal_velocity     # Target velocity (rad/s, 0 = max speed)
float32 goal_current      # Target current limit (Amperes)

# PID gains (optional, 0 = use servo defaults)
uint16 position_p_gain    # Position P gain
uint16 position_i_gain    # Position I gain
uint16 position_d_gain    # Position D gain

# Control flags
bool torque_enable        # Enable/disable servo torque
bool led_enable          # Enable/disable servo LED
uint8 control_mode       # Control mode: 0=position, 1=velocity, 2=current, 3=extended_position, 4=multi_turn

# Limits
float32 moving_threshold  # Moving detection threshold
uint16 punch             # Minimum current to start movement
uint16 current_limit     # Maximum current limit
uint16 velocity_limit    # Maximum velocity limit
uint16 max_position      # Maximum position limit
uint16 min_position      # Minimum position limit
