# Navigation lamp command message

uint64 timestamp    # time since system start (microseconds)

# Command types
uint8 CMD_SET_STATE = 0         # Set state indicator lamp
uint8 CMD_SET_ILLUMINATION = 1  # Control illumination lamp

# State constants (used with CMD_SET_STATE)
uint8 STATE_OFF = 0
uint8 STATE_BLINK_ROTATE = 1
uint8 STATE_BLINK_FAST = 2
uint8 STATE_BLINK_SLOW = 3
uint8 STATE_ON = 4

# Illumination constants (used with CMD_SET_ILLUMINATION)
uint8 OPTION_OFF = 0
uint8 OPTION_ON = 1

uint8 command       # Command type (CMD_*)
uint8 option        # Command option (STATE_* for CMD_SET_STATE, OPTION_* for CMD_SET_ILLUMINATION)
