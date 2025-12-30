# Wheel loader sensor status message
# Reports health and position of wheel loader specific sensors

uint64 timestamp		# time since system start (microseconds)

# Boom position sensor
bool boom_sensor_valid		# Boom position sensor is valid
float32 boom_position		# Current boom position/extension (m)
float32 boom_velocity		# Boom velocity (m/s)

# Tilt angle sensor
bool tilt_sensor_valid		# Tilt angle sensor is valid
float32 tilt_angle		# Current tilt angle (rad)
float32 tilt_rate		# Tilt angular rate (rad/s)

# Articulation sensor
bool articulation_sensor_valid	# Articulation sensor is valid
float32 articulation_angle	# Current articulation angle (rad)
float32 articulation_rate	# Articulation rate (rad/s)

# Limit switches
bool limit_switch_boom_min	# Boom minimum limit switch
bool limit_switch_boom_max	# Boom maximum limit switch
bool limit_switch_tilt_min	# Tilt minimum limit switch
bool limit_switch_tilt_max	# Tilt maximum limit switch

# Sensor quality indicators
uint8 boom_sensor_quality	# 0-100%
uint8 tilt_sensor_quality	# 0-100%
uint8 articulation_sensor_quality # 0-100%
