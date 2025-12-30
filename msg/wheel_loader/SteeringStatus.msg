# Steering system status

uint64 timestamp                # Time since system start (microseconds)

float32 steering_angle_deg      # Current steering angle in degrees
float32 steering_angle_setpoint_deg # Target steering angle in degrees
float32 steering_rate_deg_s     # Current steering rate in deg/s
float32 slip_compensation_deg   # Slip compensation in degrees

uint8 control_mode              # Current control mode (see CONTROL_MODE_*)

float32 actual_angle_rad        # Current steering angle in radians
float32 actual_rate_rad_s       # Current steering rate in rad/s
float32 servo_position_rad      # Servo position in radians
float32 servo_torque_nm         # Applied servo torque in Nm
float32 power_steering_pressure # Power steering pressure in Pa
float32 steering_temperature_c  # Servo controller temperature in Celsius

bool is_healthy                 # Overall system health status
bool servo_healthy              # Servo controller health status
bool power_steering_active      # Power steering system active
bool position_valid             # Position sensor valid
bool emergency_stop             # Emergency stop engaged
bool emergency_stop_active      # Emergency stop engaged (legacy)

# Limit sensor status
bool limit_left_active          # Left limit sensor activated
bool limit_right_active         # Right limit sensor activated
bool limit_sensors_healthy      # All limit sensors healthy

# Safety manager status
bool safety_violation           # Safety violation detected

uint8 error_flags               # Error flags bitfield
float32 max_torque_available    # Maximum available torque in Nm

# Control modes
uint8 CONTROL_MODE_POSITION = 0
uint8 CONTROL_MODE_VELOCITY = 1
uint8 CONTROL_MODE_TORQUE = 2
