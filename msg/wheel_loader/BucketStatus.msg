uint64 timestamp               # time since system start (microseconds)

# Basic bucket status
uint8 state                    # 0: uninitialized, 1: zeroing, 2: ready, 3: moving, 4: error
float32 actuator_length        # current actuator length in mm
float32 target_actuator_length # target actuator length in mm
float32 bucket_angle           # current bucket angle relative to boom (radians)
float32 current_angle          # alias for bucket_angle (for compatibility)
float32 ground_angle           # current bucket angle relative to ground (radians)
float32 velocity               # current actuator velocity (mm/s)
float32 control_output         # motor control output (-1 to 1)

# Motor information
float32 motor_current          # electric motor current (Amperes)
float32 motor_voltage          # electric motor voltage (Volts)
float32 motor_temperature_c    # electric motor temperature (Celsius)
bool motor_fault               # electric motor fault flag
bool encoder_fault             # position encoder fault flag

# Limit switches and zeroing
bool limit_switch_coarse       # coarse limit switch state (down position)
bool limit_switch_fine         # fine limit switch state (up position)
bool limit_switch_load         # load limit switch state (bucket down)
bool limit_switch_dump         # dump limit switch state (bucket up)
bool zeroing_complete          # zeroing procedure completion status

# Boom and machine angles
float32 boom_angle             # boom angle (radians)
float32 machine_pitch          # machine pitch angle (radians)
float32 machine_roll           # machine roll angle (radians)

# State management and calibration
uint8 calibration_phase        # current calibration phase
uint32 error_flags             # error flags bitmask
uint8 calibration_progress     # calibration progress (0-100%)

# Control and hardware status
bool boom_compensation_enabled # boom angle compensation enabled
bool motor_enabled             # motor enabled status
bool encoder_valid             # encoder validity status
bool limits_valid              # limit switches validity status
float32 target_ground_angle    # target angle relative to ground (radians)

# Performance metrics
float32 position_error_rms     # position error RMS (mm)
float32 velocity_error_rms     # velocity error RMS (mm/s)
float32 control_effort         # control effort (0-1)

# AHRS integration fields
uint8 control_mode             # 0: Manual, 1: Auto-level, 2: Slope compensation, 3: Grading, 4: Transport
float32 stability_factor       # dynamic stability factor (0.0 = unstable, 1.0 = stable)
bool anti_spill_active         # anti-spill control active flag
bool stability_warning         # stability warning flag

# Load estimation
float32 estimated_load_kg      # estimated load in bucket (kg)
