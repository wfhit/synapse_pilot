# Command module status

uint64 timestamp  # time since system start (microseconds)

# Command sources
bool mavlink_connected
uint64 mavlink_last_heartbeat  # timestamp of last MAVLink heartbeat
bool rc_connected
uint64 rc_last_update  # timestamp of last RC update

# Current command authority
uint8 AUTHORITY_NONE = 0
uint8 AUTHORITY_RC = 1
uint8 AUTHORITY_MAVLINK = 2
uint8 AUTHORITY_BOTH = 3
uint8 command_authority

# RC inputs
bool rc_arm_switch
bool rc_emergency_stop
bool rc_deadman_switch
uint8 rc_mode_switch  # 0-7 mode selection
uint8 rc_task_trigger  # 0-7 task selection

# MAVLink command statistics
uint32 mavlink_commands_received
uint32 mavlink_commands_accepted
uint32 mavlink_commands_rejected

# RC command statistics
uint32 rc_commands_sent
uint32 rc_mode_changes
uint32 rc_task_triggers

# Safety status
bool deadman_required
bool deadman_active
bool emergency_stop_active
bool command_allowed  # overall command permission

# Last command info
uint8 LAST_CMD_NONE = 0
uint8 LAST_CMD_ARM = 1
uint8 LAST_CMD_DISARM = 2
uint8 LAST_CMD_MODE_CHANGE = 3
uint8 LAST_CMD_TASK_START = 4
uint8 LAST_CMD_TASK_STOP = 5
uint8 LAST_CMD_EMERGENCY_STOP = 6
uint8 LAST_CMD_SAFETY_ACK = 7
uint8 LAST_CMD_CALIBRATION = 8
uint8 last_command_type
uint64 last_command_timestamp
