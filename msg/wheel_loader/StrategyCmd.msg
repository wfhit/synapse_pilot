# Strategy Command Message
# Command to start/stop/abort strategies

uint64 timestamp  # time since system start (microseconds)

# Strategy IDs
uint8 STRATEGY_IDLE = 0
uint8 STRATEGY_MANUAL_BUCKET = 1
uint8 STRATEGY_TRAJECTORY_FOLLOWER = 2
uint8 STRATEGY_HOLD = 3
uint8 STRATEGY_CALIBRATION_ENCODER = 10
uint8 STRATEGY_CALIBRATION_ACTUATOR = 11
uint8 STRATEGY_CALIBRATION_TILT = 12
uint8 STRATEGY_EMERGENCY_STOP = 255

# Command source
uint8 SOURCE_INTERNAL = 0
uint8 SOURCE_MAVLINK = 1
uint8 SOURCE_RC = 2
uint8 SOURCE_FAILSAFE = 3

uint8 strategy_id   # Target strategy ID
uint8 source        # Command source
uint8 priority      # Command priority (0-255, higher = more important)
