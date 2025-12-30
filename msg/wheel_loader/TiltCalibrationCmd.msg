# Tilt calibration command message
# Published by task executor to calibrate/zero the tilt tracking module

uint64 timestamp		# time since system start (microseconds)

uint8 command			# Calibration command

# Calibration command definitions
uint8 CMD_ZERO = 0		# Zero the tilt angle at current position
uint8 CMD_SET_LIMITS = 1	# Set min/max limits
uint8 CMD_RESET = 2		# Reset calibration to defaults

float32 tilt_min		# Minimum tilt angle (radians)
float32 tilt_max		# Maximum tilt angle (radians)
float32 tilt_zero		# Zero reference angle (radians)
