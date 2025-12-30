# Wheel Loader Chassis Control Setpoint
# Output from MPC controller for chassis motion

uint64 timestamp		# time since system start (microseconds)

# Control setpoints (velocity control)
float32 velocity_x		# Forward velocity [m/s]
float32 velocity_y		# Lateral velocity [m/s] (usually 0 for non-holonomic)
float32 yaw_rate		# Yaw rate [rad/s]
float32 articulation_rate	# Articulation angle rate [rad/s]

# Feedforward acceleration (optional)
float32 acceleration_x		# Forward acceleration [m/s^2]
float32 acceleration_y		# Lateral acceleration [m/s^2]

# Target state (for logging/monitoring)
float32 target_x		# Target position x [m]
float32 target_y		# Target position y [m]
float32 target_heading		# Target heading [rad]
float32 target_articulation	# Target articulation angle [rad]

# Control flags
bool position_valid		# Position setpoint is valid
bool velocity_valid		# Velocity setpoint is valid
bool articulation_valid		# Articulation setpoint is valid
