# Calibration data message for wheel loader calibrations
# Published by calibration tasks with calibration results

uint64 timestamp		# time since system start (microseconds)

# Calibration type
uint8 CALIB_STEERING = 0	# Steering servo calibration
uint8 CALIB_ENCODER = 1		# Encoder to wheel speed
uint8 CALIB_ACTUATOR = 2	# Linear actuator position

uint8 calibration_type		# Type of calibration

# Calibration status
bool success			# Calibration completed successfully
bool in_progress		# Calibration in progress
uint8 progress_percent		# Progress percentage (0-100)

# Steering servo calibration results
float32 steering_center		# Center position (rad)
float32 steering_min		# Minimum angle (rad)
float32 steering_max		# Maximum angle (rad)
float32 steering_scale		# Scale factor (rad per command unit)
float32 steering_deadband	# Deadband near center (rad)

# Motor encoder calibration results
float32 left_encoder_scale	# Left wheel: pulses per meter
float32 right_encoder_scale	# Right wheel: pulses per meter
float32 encoder_noise_threshold	# Noise threshold (m/s)

# Linear actuator calibration results
# Boom actuator
float32 boom_min_position	# Minimum position (m)
float32 boom_max_position	# Maximum position (m)
float32 boom_scale		# Scale factor (m per command unit)

# Tilt actuator (already in TiltCalibrationCmd, but included for completeness)
float32 tilt_min_angle		# Minimum angle (rad)
float32 tilt_max_angle		# Maximum angle (rad)
float32 tilt_zero_angle		# Zero reference angle (rad)

# Articulation actuator
float32 articulation_min	# Minimum angle (rad)
float32 articulation_max	# Maximum angle (rad)
float32 articulation_center	# Center position (rad)

# Quality metrics
float32 calibration_quality	# Quality score 0-100
uint32 samples_collected	# Number of samples used
