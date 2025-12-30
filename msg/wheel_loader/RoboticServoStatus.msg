# Status message for smart servos like ST3125

uint64 timestamp          # time since system start (microseconds)
uint8 id                  # Servo ID (ST3125 servo ID)
float32 position          # Current position (radians)
float32 velocity          # Current velocity (rad/s)
float32 current           # Current draw (Amperes)
float32 voltage           # Supply voltage (Volts)
float32 temperature       # Temperature (Celsius)
uint16 error_flags        # Error status flags
bool torque_enabled       # Torque enable status
float32 goal_position     # Target position (radians)
float32 load              # Current load (-1.0 to 1.0)

# ST3125-specific fields for steering control
float32 position_error    # Position error (goal - current) in radians
bool moving               # Servo is currently moving
bool position_reached     # Target position has been reached
uint8 hardware_alarm      # Hardware alarm status
uint8 shutdown_alarm      # Shutdown alarm status
