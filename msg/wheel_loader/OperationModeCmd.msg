# Operation mode command message
# This message is used to command a mode switch in the operation_mode system

uint64 timestamp		# time since system start (microseconds)

uint8 target_mode		# Requested operation mode
uint8 mode_source		# Source of the mode command

# Operation mode definitions
uint8 MODE_WL_TRAJ_FOLLOWER = 0	# Wheel loader trajectory follower mode
uint8 MODE_WL_MANUAL_BUCKET = 1	# Wheel loader manual bucket control mode
uint8 MODE_WL_MANUAL_DIRECT = 2	# Wheel loader manual direct control mode
uint8 MODE_WL_HOLD = 3		# Wheel loader hold mode
uint8 MODE_WL_LOITER = 4	# Wheel loader loiter mode
uint8 MODE_WL_SAFETY_STOP = 5	# Wheel loader safety stop mode

# Mode source definitions
uint8 SOURCE_INTERNAL = 0	# Internal mode logic
uint8 SOURCE_OFFBOARD = 1	# Offboard computer
uint8 SOURCE_GCS = 2		# Ground control station
uint8 SOURCE_RC = 3		# RC transmitter

uint8 priority			# Command priority (0-255, higher = more important)
