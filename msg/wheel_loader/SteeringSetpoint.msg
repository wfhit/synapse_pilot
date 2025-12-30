# Steering setpoint for chassis control

uint64 timestamp               # Time since system start (microseconds)

float32 steering_angle_rad     # Desired steering angle in radians
float32 steering_rate_rad_s    # Maximum steering rate in rad/s
float32 steering_torque_nm     # Steering assist torque in Nm (optional)

bool power_steering_enable     # Enable power steering assistance
bool auto_centering_enable     # Enable automatic centering when not steering
