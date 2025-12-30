# Wheel loader actuator status and health
# Reports position, velocity, command tracking, and health for each actuator

uint64 timestamp		# time since system start (microseconds)

# Chassis actuators (left/right drive motors)
bool chassis_left_valid
float32 chassis_left_velocity_cmd	# Commanded velocity (m/s)
float32 chassis_left_velocity_actual	# Actual velocity (m/s)
float32 chassis_left_error		# Tracking error (m/s)
uint8 chassis_left_quality		# Health quality 0-100%

bool chassis_right_valid
float32 chassis_right_velocity_cmd
float32 chassis_right_velocity_actual
float32 chassis_right_error
uint8 chassis_right_quality

# Boom actuator (hydraulic)
bool boom_valid
float32 boom_position_cmd		# Commanded position (m)
float32 boom_position_actual		# Actual position (m)
float32 boom_velocity_actual		# Actual velocity (m/s)
float32 boom_error			# Position tracking error (m)
uint8 boom_quality			# Health quality 0-100%

# Tilt actuator (hydraulic)
bool tilt_valid
float32 tilt_angle_cmd			# Commanded angle (rad)
float32 tilt_angle_actual		# Actual angle (rad)
float32 tilt_rate_actual		# Actual rate (rad/s)
float32 tilt_error			# Angle tracking error (rad)
uint8 tilt_quality			# Health quality 0-100%

# Articulation actuator (hydraulic)
bool articulation_valid
float32 articulation_angle_cmd		# Commanded angle (rad)
float32 articulation_angle_actual	# Actual angle (rad)
float32 articulation_rate_actual	# Actual rate (rad/s)
float32 articulation_error		# Angle tracking error (rad)
uint8 articulation_quality		# Health quality 0-100%

# Timeout tracking (milliseconds since last update)
uint32 chassis_left_timeout_ms
uint32 chassis_right_timeout_ms
uint32 boom_timeout_ms
uint32 tilt_timeout_ms
uint32 articulation_timeout_ms
