# Wheel Loader Trajectory Message
# Contains up to 16 trajectory points for wheel loader control

uint64 timestamp		# time since system start (microseconds)

# Trajectory metadata
uint8 trajectory_type		# Type of trajectory representation
uint8 TRAJ_TYPE_BUCKET_6DOF = 0		# Bucket 6DOF pose
uint8 TRAJ_TYPE_CHASSIS_BUCKET_FULL = 1	# Chassis pos + bucket heading/altitude/tilt + articulation
uint8 TRAJ_TYPE_CHASSIS_BUCKET_POSE = 2	# Chassis pose + bucket altitude/tilt

uint8 frame_id			# Reference frame
uint8 FRAME_GLOBAL = 0		# Global/world frame
uint8 FRAME_LOCAL = 1		# Local NED frame
uint8 FRAME_BODY = 2		# Body frame

uint8 num_points		# Number of valid trajectory points (1-16)
uint8 current_point_index	# Index of current target point

# Trajectory points (max 16)
uint64[16] point_timestamps	# Timestamp for each point (microseconds)

# Type 0: Bucket 6DOF pose
float32[16] bucket_x		# Bucket position x [m]
float32[16] bucket_y		# Bucket position y [m]
float32[16] bucket_z		# Bucket position z [m]
float32[16] bucket_qw		# Bucket orientation quaternion w
float32[16] bucket_qx		# Bucket orientation quaternion x
float32[16] bucket_qy		# Bucket orientation quaternion y
float32[16] bucket_qz		# Bucket orientation quaternion z

# Type 1 & 2: Chassis position/pose
float32[16] chassis_x		# Chassis position x [m]
float32[16] chassis_y		# Chassis position y [m]
float32[16] chassis_heading	# Chassis heading [rad] (Type 2 only)

# Type 1 & 2: Bucket parameters
float32[16] bucket_heading	# Bucket heading [rad] (Type 1 only)
float32[16] bucket_altitude	# Bucket altitude/height [m]
float32[16] bucket_tilt		# Bucket tilt angle [rad]

# Type 1: Articulation
float32[16] articulation_angle	# Articulated steering angle [rad] (Type 1 only)

# Control flags
bool hold_last_point		# Hold position after reaching last point
bool smooth_transition		# Smooth blend from current state to first point
