# Arm status message
# Reports current arming state and conditions

uint64 timestamp		# time since system start (microseconds)

# Arming state
bool armed			# System is armed
bool ready_to_arm		# All conditions met for arming

# Arm prevention flags (true = blocking arm)
bool health_check_failed	# System not healthy
bool mode_not_allowed		# Current mode doesn't allow arming
bool actuators_not_ready	# Actuators not in valid state
bool manual_control_lost	# No manual control signal

# Idle detection for auto-disarm
bool system_idle		# System detected as idle
float32 idle_time		# Time system has been idle (seconds)

# Disarm reason (when armed=false)
uint8 DISARM_REASON_NONE = 0
uint8 DISARM_REASON_MANUAL = 1		# Manual disarm request
uint8 DISARM_REASON_TIMEOUT = 2		# Idle timeout
uint8 DISARM_REASON_HEALTH = 3		# Health monitor failure
uint8 DISARM_REASON_EMERGENCY = 4	# Emergency condition
uint8 DISARM_REASON_INIT = 5		# Initial state

uint8 last_disarm_reason	# Reason for last disarm

# Timing
float32 time_since_armed	# Time since last arm (seconds, -1 if never)
float32 time_since_disarmed	# Time since last disarm (seconds, -1 if never)

# Command rejection
uint32 arm_reject_count		# Total arm commands rejected
