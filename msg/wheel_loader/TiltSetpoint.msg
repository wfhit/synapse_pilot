# Wheel Loader Tilt Control Setpoint
# Output from S-curve planner for bucket tilt motion

uint64 timestamp		# time since system start (microseconds)

# Position control
float32 angle			# Tilt angle [rad]
float32 angular_velocity	# Tilt angular velocity [rad/s]
float32 angular_acceleration	# Tilt angular acceleration [rad/s^2]

# Control mode
uint8 control_mode		# Control mode
uint8 MODE_POSITION = 0		# Position control
uint8 MODE_VELOCITY = 1		# Velocity control
uint8 MODE_DIRECT = 2		# Direct actuator control

# Limits (for safety)
float32 max_velocity		# Maximum angular velocity limit
float32 max_acceleration	# Maximum angular acceleration limit

# Status flags
bool setpoint_valid		# Setpoint is valid
bool trajectory_complete	# Reached target angle
