# Strategy Status Message
# Published by strategy_executor to indicate current strategy state

uint64 timestamp  # time since system start (microseconds)

# Strategy states
uint8 STATE_IDLE = 0
uint8 STATE_ACTIVE = 1
uint8 STATE_PRECHECK = 2
uint8 STATE_INIT = 3
uint8 STATE_RUNNING = 4

uint8 current_strategy_id    # Currently active strategy (0 = none)
uint8 state                  # Current strategy state
uint8 current_step           # Current step number (0-based)
uint8 total_steps            # Total number of steps in strategy

char[50] strategy_name       # Human-readable strategy name
char[50] current_step_name   # Current step name
char[100] status_text        # Status or error description
