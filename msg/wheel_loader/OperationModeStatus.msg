# Operation mode status message
# Published by the operation mode manager to report current system state

uint64 timestamp		# time since system start (microseconds)

uint8 current_mode		# Currently active mode (see OperationModeCmd)
uint8 previous_mode		# Previously active mode
uint64 mode_start_time		# Timestamp when current mode was activated

# Mode state
bool transition_in_progress	# Mode transition currently happening
bool mode_ready			# Current mode is ready to operate
bool fallback_active		# Emergency fallback mode active

# Diagnostics
uint32 mode_switch_count	# Total number of mode switches
uint32 mode_failure_count	# Number of mode activation failures
uint64 last_mode_switch_time	# Timestamp of last successful mode switch
