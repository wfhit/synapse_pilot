# Safety status message
# Reports current safety state and active faults

uint64 timestamp		# time since system start (microseconds)

# Overall safety state
uint8 LEVEL_NONE = 0		# Normal operation
uint8 LEVEL_WARNING = 1		# Warning only
uint8 LEVEL_CAUTION = 2		# Stop tasks
uint8 LEVEL_CRITICAL = 3	# Emergency stop
uint8 LEVEL_EMERGENCY = 4	# Immediate disarm

uint8 safety_level		# Current highest safety level
bool safety_ok			# All safety checks pass (level 0)
bool faults_latched		# Some faults are latched awaiting acknowledge

# Active fault flags (true = fault active)
bool fault_emergency_stop	# E-stop button pressed
bool fault_deadman		# Dead man switch released
bool fault_limit_switch		# Limit switch triggered
bool fault_health_system	# System health failure
bool fault_health_vehicle	# Vehicle stability failure
bool fault_health_sensors	# Sensor failure
bool fault_health_actuators	# Actuator failure
bool fault_health_power		# Power system failure
bool fault_external_estop	# External E-stop triggered
bool fault_remote_estop		# Remote E-stop triggered

# Fault latching status
bool latched_emergency_stop
bool latched_deadman
bool latched_limit_switch
bool latched_health_system
bool latched_health_vehicle
bool latched_health_sensors
bool latched_health_actuators
bool latched_health_power
bool latched_external_estop
bool latched_remote_estop

# Actions taken
bool task_abort_commanded	# Task abort command sent
bool emergency_stop_commanded	# Emergency stop task started
bool disarm_commanded		# Disarm command sent
bool safety_stop_commanded	# SAFETY_STOP mode commanded

# Statistics
uint32 total_faults		# Total faults since boot
uint32 acknowledged_faults	# Total acknowledged faults
float32 time_since_fault	# Time since last fault (seconds, -1 if never)
