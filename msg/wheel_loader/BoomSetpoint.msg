# Wheel Loader Boom Control Setpoint
# Output from S-curve planner for boom motion

uint64 timestamp		# time since system start (microseconds)

# Position control
float32 position		# Boom position/angle [m or rad]
float32 velocity		# Boom velocity [m/s or rad/s]
float32 acceleration		# Boom acceleration [m/s^2 or rad/s^2]

# Control mode
uint8 control_mode		# Control mode
uint8 MODE_POSITION = 0		# Position control
uint8 MODE_VELOCITY = 1		# Velocity control
uint8 MODE_DIRECT = 2		# Direct actuator control

# Limits (for safety)
float32 max_velocity		# Maximum velocity limit
float32 max_acceleration	# Maximum acceleration limit

# Status flags
bool setpoint_valid		# Setpoint is valid
bool trajectory_complete	# Reached target position
