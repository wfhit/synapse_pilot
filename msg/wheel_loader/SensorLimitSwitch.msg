uint64 timestamp                    # time since system start (microseconds)

uint8 instance                      # Instance ID of this limit sensor
uint8 function                      # Function of this limit sensor (see LimitFunction enum)
bool state                         # Combined state (true = limit reached)
bool switch_1_state                # Individual switch 1 state
bool switch_2_state                # Individual switch 2 state
bool redundancy_enabled            # Using redundant switches
bool redundancy_fault              # Mismatch between switches
uint32 activation_count            # Total number of activations
uint64 last_activation_time        # Last time limit was activated

# Limit sensor functions
uint8 FUNCTION_BUCKET_LOAD = 0
uint8 FUNCTION_BUCKET_DUMP = 1
uint8 FUNCTION_BOOM_UP = 2
uint8 FUNCTION_BOOM_DOWN = 3
uint8 FUNCTION_STEERING_LEFT = 4
uint8 FUNCTION_STEERING_RIGHT = 5
uint8 FUNCTION_DISABLED = 255

uint8 ORB_QUEUE_LENGTH = 4
