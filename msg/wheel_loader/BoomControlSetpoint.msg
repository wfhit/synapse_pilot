# Boom control setpoint
# Published by ModeVLA at 50Hz
# Subscribed by boom control module

uint64 timestamp                    # time since system start (microseconds)

# Height setpoint (IK computed in boom_control)
float32 bucket_height               # Target bucket height from ground [m]
float32 bucket_height_velocity      # Target bucket height velocity [m/s]

# Trajectory info
uint32 sequence                     # Trajectory sequence ID
bool valid                          # Setpoint is valid
