uint64 timestamp          # time since system start (microseconds)
uint8 instance            # H-bridge instance (0 or 1)
float32 duty_cycle        # Current duty cycle (-1.0 to 1.0)
bool forward_limit        # Forward limit sensor active (stops forward motion)
bool reverse_limit        # Reverse limit sensor active (stops reverse motion)
bool enabled              # H-bridge enable state

