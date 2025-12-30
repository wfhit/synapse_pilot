# Traction control status and commands

uint64 timestamp                   # microseconds since boot

# System status
bool traction_control_active
float32 overall_traction_quality   # 0.0 = no traction, 1.0 = perfect
bool any_wheel_slipping

# Per-wheel slip status
float32[4] wheel_slip_ratios       # [FL, FR, RL, RR]
bool[4] wheel_slip_flags           # True if wheel is slipping
float32[4] traction_coefficients   # Estimated traction coefficient per wheel

# Control adjustments
float32[4] speed_adjustments       # Speed reduction factors (0.0-1.0)
uint8 intervention_level           # 0: none, 1: light, 2: moderate, 3: aggressive
