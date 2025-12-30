# Arm/Disarm command message
# Multiple sources can request arming/disarming

uint64 timestamp		# time since system start (microseconds)

# Command types
uint8 ARM = 0
uint8 DISARM = 1

uint8 command			# ARM or DISARM

# Command source
uint8 SOURCE_UNKNOWN = 0
uint8 SOURCE_GCS = 1		# Ground control station (MAVLink)
uint8 SOURCE_RC = 2		# RC transmitter switch
uint8 SOURCE_ONBOARD = 3	# Onboard logic/script
uint8 SOURCE_BUTTON = 4		# Physical button
uint8 SOURCE_AUTO = 5		# Automatic (timeout, safety)

uint8 source			# Command source

bool force			# Force operation (bypass some safety checks)
