# Wheel controller status for monitoring by wheel loader

uint64 timestamp                # Time since system start (microseconds)

# Wheel identification
uint8 wheel_id                  # 0=front_left, 1=front_right, 2=rear_left, 3=rear_right
bool is_front_wheel             # True for front wheels, false for rear wheels

# Speed control status
float32 current_speed_rpm       # Current wheel speed in RPM
float32 target_speed_rpm        # Target wheel speed in RPM
float32 speed_error_rpm         # Speed tracking error in RPM
float32 pwm_output              # PWM output to motor (-1000 to +1000)

# Encoder data
int32 encoder_count             # Raw encoder count
int32 encoder_delta             # Encoder count change since last update
float32 encoder_speed_rpm       # Speed calculated from encoder
bool encoder_healthy            # Encoder signal quality status

# Traction control status
bool slip_detected              # True if wheel slip is detected
float32 slip_ratio              # Current slip ratio (0-1)
float32 traction_limit_factor   # Current traction limiting factor (0-1)
uint32 slip_events_count        # Total slip events since startup

# Motor and electrical status
float32 motor_current_amps      # Motor current draw in amperes
float32 motor_temperature_c     # Motor temperature in Celsius (if available)
float32 supply_voltage_v        # Motor supply voltage
bool current_limit_active       # True if current limiting is active

# Safety and health status
bool controller_healthy         # Overall controller health
bool emergency_stop_active      # True if emergency stop is engaged
bool armed                      # True if controller is armed
float32 health_score            # Overall health score (0-100)
uint64 last_command_time        # Time of last valid command

# Performance metrics
float32 speed_error_rms         # RMS speed tracking error
float32 control_effort_avg      # Average control effort
uint32 missed_updates_count     # Count of missed control updates
uint32 safety_violations_count  # Count of safety violations
float32 max_speed_error_rpm     # Maximum speed error recorded

# Control mode and configuration
uint8 control_mode              # Current control mode
bool traction_control_enabled   # True if traction control is active
float32 speed_limit_rpm         # Current speed limit setting
float32 ramp_rate_rpm_s         # Current speed ramp rate

# Error flags
bool encoder_timeout            # True if encoder data is stale
bool command_timeout            # True if command data is stale
bool overcurrent_detected       # True if overcurrent condition
bool overspeed_detected         # True if overspeed condition
bool hardware_fault             # True if hardware fault detected

# Control mode constants
uint8 MODE_DISABLED = 0
uint8 MODE_SPEED_CONTROL = 1
uint8 MODE_TORQUE_CONTROL = 2
uint8 MODE_EMERGENCY_STOP = 3
