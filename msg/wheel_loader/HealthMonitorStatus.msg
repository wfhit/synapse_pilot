# Health monitor status message
# Reports overall system health with detailed per-subsystem status

uint64 timestamp		# time since system start (microseconds)

# Overall health summary (true = healthy)
bool system_healthy		# All systems healthy
bool sensors_healthy		# All sensors healthy
bool actuators_healthy		# All actuators healthy
bool power_healthy		# Power system healthy
bool vehicle_stable		# Vehicle within safe operating limits

# Sensor health bits (true = healthy)
bool sensor_gps			# GPS sensor
bool sensor_imu			# IMU sensor
bool sensor_mag			# Magnetometer
bool sensor_baro		# Barometer
bool sensor_boom		# Boom position sensor
bool sensor_tilt		# Tilt angle sensor
bool sensor_articulation	# Articulation sensor
bool sensor_limit_switches	# Limit switches

# Actuator health bits (true = healthy)
bool actuator_chassis_left	# Left chassis motor
bool actuator_chassis_right	# Right chassis motor
bool actuator_boom		# Boom hydraulic actuator
bool actuator_tilt		# Tilt actuator
bool actuator_articulation	# Articulation actuator

# Power system health bits (true = healthy)
bool battery_voltage_ok		# Battery voltage within limits
bool battery_current_ok		# Battery current within limits
bool battery_charge_ok		# Battery charge sufficient
bool battery_temp_ok		# Battery temperature OK

# Vehicle stability bits (true = safe)
bool tilt_angle_safe		# Vehicle tilt angle within limits
bool slip_ok			# No excessive slip detected
bool shock_ok			# No excessive shock detected
bool temperature_ok		# System temperature OK

# Communication health bits (true = connected)
bool mavlink_connected		# MAVLink connection active
bool rc_connected		# RC receiver connection active
uint64 mavlink_last_heartbeat	# Timestamp of last MAVLink activity
uint64 rc_last_update		# Timestamp of last RC signal

# Fault counts (total since boot)
uint32 sensor_fault_count
uint32 actuator_fault_count
uint32 power_fault_count
uint32 stability_fault_count

# Time since last fault (seconds, -1 = never)
float32 time_since_sensor_fault
float32 time_since_actuator_fault
float32 time_since_power_fault
float32 time_since_stability_fault

# Current measurements for monitoring
float32 vehicle_roll		# Vehicle roll angle (rad)
float32 vehicle_pitch		# Vehicle pitch angle (rad)
float32 slip_ratio		# Estimated slip ratio (0-1)
float32 max_accel_magnitude	# Peak acceleration since last report (m/s^2)
