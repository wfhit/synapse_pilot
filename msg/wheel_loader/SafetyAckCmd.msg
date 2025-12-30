# Safety acknowledge command
# Used to acknowledge and clear latched safety faults

uint64 timestamp		# time since system start (microseconds)

uint8 ACK_ALL = 0		# Acknowledge all clearable faults
uint8 ACK_EMERGENCY_STOP = 1
uint8 ACK_DEADMAN = 2
uint8 ACK_LIMIT_SWITCH = 3
uint8 ACK_HEALTH_SYSTEM = 4
uint8 ACK_HEALTH_VEHICLE = 5
uint8 ACK_HEALTH_SENSORS = 6
uint8 ACK_HEALTH_ACTUATORS = 7
uint8 ACK_HEALTH_POWER = 8
uint8 ACK_EXTERNAL_ESTOP = 9
uint8 ACK_REMOTE_ESTOP = 10

uint8 acknowledge_type		# What to acknowledge

bool force_clear		# Force clear even if condition still present (use carefully)
