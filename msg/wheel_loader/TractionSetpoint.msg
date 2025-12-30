# Traction control setpoint message for high-level traction and motion commands
# Used by chassis trajectory follower to communicate desired traction behavior
# to the traction control system

uint64 timestamp                    # Time since system start (microseconds)

# Motion commands
float32 desired_velocity_ms         # Desired forward/backward velocity [m/s]
float32 desired_acceleration_ms2    # Desired acceleration [m/s²]
float32 desired_steering_angle_rad  # Desired steering angle [rad]
float32 desired_yaw_rate_rad_s      # Desired yaw rate [rad/s]

# Traction control settings
bool enable_traction_control        # Enable/disable traction control system
bool enable_slip_control           # Enable/disable slip control
bool enable_stability_control      # Enable/disable stability control
uint8 traction_mode                # Traction mode selection

# Force/torque distribution preferences
float32 torque_bias                # Torque distribution bias [-1.0=rear bias, 1.0=front bias]
float32 max_slip_ratio             # Maximum allowed slip ratio [0.0-1.0]
float32 target_traction_coefficient # Target traction coefficient for surface [0.0-1.0]

# Surface and operating conditions
uint8 surface_type                 # Surface type hint for traction algorithms
float32 load_factor               # Load factor affecting traction [0.0-2.0, 1.0=nominal]
bool aggressive_mode              # Enable aggressive traction recovery

# Safety limits
float32 max_torque_nm             # Maximum total torque limit [Nm]
float32 max_velocity_ms           # Maximum velocity limit [m/s]
bool emergency_stop               # Emergency stop command

# Operating modes
uint8 MODE_ECO = 0                # Eco mode - prioritize efficiency
uint8 MODE_NORMAL = 1             # Normal mode - balanced performance
uint8 MODE_PERFORMANCE = 2        # Performance mode - maximum traction
uint8 MODE_PRECISION = 3          # Precision mode - fine control for work operations

# Surface types
uint8 SURFACE_UNKNOWN = 0         # Unknown surface conditions
uint8 SURFACE_CONCRETE = 1        # Hard concrete/asphalt surface
uint8 SURFACE_GRAVEL = 2          # Gravel or rough surface
uint8 SURFACE_SAND = 3            # Sandy or loose surface
uint8 SURFACE_MUD = 4             # Muddy or slippery surface
uint8 SURFACE_SNOW = 5            # Snow or ice surface
uint8 SURFACE_MIXED = 6           # Mixed surface conditions

# Coordination with other systems
bool coordination_active          # Coordination with bucket/boom active
float32 coordination_factor       # Coordination influence factor [0.0-1.0]
