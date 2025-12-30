# Load lamp command message for wheel loader
# Commands the load indication lamps based on motor load analysis

uint64 timestamp         # time since system start (microseconds)

# Load level constants
uint8 LOAD_VERY_LOW = 0  # 0-10% load: Very slow blink (0.5 Hz)
uint8 LOAD_LOW = 1       # 10-20% load: Slow blink (1 Hz)
uint8 LOAD_MED_LOW = 2   # 20-30% load: Medium-slow blink (2 Hz)
uint8 LOAD_MEDIUM = 3    # 30-60% load: Medium blink (5 Hz)
uint8 LOAD_HIGH = 4      # 60-80% load: Fast blink (10 Hz)
uint8 LOAD_VERY_HIGH = 5 # 80-100% load: Very fast blink (20 Hz)
uint8 LOAD_OFF = 255     # Turn off lamps

uint8 load_level         # Load level (LOAD_*)
float32 load_value       # Actual load value (0.0-1.0) for reference
uint32 blink_interval_us # Blink interval in microseconds
