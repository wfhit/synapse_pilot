# Operation control configuration message
# Similar to vehicle_control_mode but for the operation_mode system
# Published by active operation mode to configure which controllers should be active

uint64 timestamp		# time since system start (microseconds)

# Current active mode
uint8 current_mode		# Currently active operation mode (see OperationModeCmd)

# Control loop enable flags
bool flag_control_position_enabled
bool flag_control_velocity_enabled
bool flag_control_acceleration_enabled
bool flag_control_attitude_enabled
bool flag_control_rates_enabled

# Additional mode-specific flags
bool flag_control_auto_enabled		# Autonomous control active
bool flag_control_manual_enabled	# Manual control active
bool flag_trajectory_control_enabled	# Trajectory tracking active

# Mode status
bool mode_active		# Mode is actively running
bool mode_valid			# Mode has valid inputs and can operate
