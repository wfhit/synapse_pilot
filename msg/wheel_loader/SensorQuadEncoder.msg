# Quadrature encoder sensor data
# Generic topic for quadrature encoders (wheels, actuators, linear motors, etc.)

uint64 timestamp              # time since system start (microseconds)

# Instance identification
uint8 instance               # Encoder instance number

# Processed encoder data
float64 position             # Accumulated position (1/million rad for rotary, mm for linear)
float64 velocity             # Velocity (1/million rad/s for rotary, mm/s for linear)
