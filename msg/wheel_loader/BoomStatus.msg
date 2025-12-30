uint64 timestamp        # time since system start (microseconds)
float32 angle           # boom angle in radians (relative to horizontal)
float32 velocity        # boom angular velocity (rad/s)
float32 load            # current load on boom (normalized 0-1)
float32 motor_current   # electric motor current (Amperes)
float32 motor_voltage   # electric motor voltage (Volts)
float32 motor_temperature_c  # electric motor temperature (Celsius)
bool motor_fault        # electric motor fault flag
bool encoder_fault      # position encoder fault flag
bool position_valid     # position sensor valid flag
uint8 state             # boom state (0: uninitialized, 1: zeroing, 2: ready, 3: moving, 4: error)
