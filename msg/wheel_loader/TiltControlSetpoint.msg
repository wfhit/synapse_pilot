# Tilt control setpoint
# Published by ModeVLA at 50Hz
# Subscribed by tilt control module

uint64 timestamp                    # time since system start (microseconds)

# Angle setpoint
float32 angle                       # Target tilt angle [rad]
float32 angular_velocity            # Target angular velocity [rad/s]
float32 angular_acceleration        # Target angular acceleration [rad/s^2]

# Trajectory info
uint32 sequence                     # Trajectory sequence ID
bool valid                          # Setpoint is valid
