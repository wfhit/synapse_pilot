# Wheel Loader Trajectory Follower Mode

## Overview

The Wheel Loader Trajectory Follower Mode is a specialized operation mode designed for autonomous wheel loader control. It provides sophisticated trajectory tracking with time-synchronized motion for chassis, boom, and bucket tilt.

## Key Features

- **Model Predictive Control (MPC)** for chassis motion
- **S-Curve trajectory planning** for smooth boom and tilt motion
- **Time-synchronized execution** across all actuators
- **Multi-frame support** (Global, Local, Body frames)
- **Three trajectory representation types**
- **Smooth trajectory blending**
- **50 Hz update rate** for responsive control

## Architecture

### Components

```
wheel_loader/
├── wheel_loader_traj_follower_mode.hpp/cpp   # Main mode controller
├── chassis_mpc_controller.hpp/cpp            # MPC for chassis
├── s_curve_planner.hpp/cpp                   # S-curve trajectory planner
└── vehicle_params.yaml                       # Vehicle configuration
```

### Data Flow

```
WheelLoaderTrajectory (input)
    ↓
[Trajectory Decoder]
    ↓
[MPC Controller] → ChassisSetpoint → Low-level Controller
[S-Curve Boom]   → BoomSetpoint    → Low-level Controller  
[S-Curve Tilt]   → TiltSetpoint    → Low-level Controller
    ↓
[Time Synchronization]
```

## Trajectory Message Format

### Input: `WheelLoaderTrajectory`

Contains up to 16 trajectory points with:

- **Trajectory metadata**:
  - `trajectory_type`: Type of trajectory (0, 1, or 2)
  - `frame_id`: Reference frame (Global/Local/Body)
  - `num_points`: Number of valid points
  - `current_point_index`: Current target point

- **Control flags**:
  - `hold_last_point`: Hold position at end
  - `smooth_transition`: Enable smooth blending

### Trajectory Types

#### Type 0: Bucket 6DOF Pose
Complete bucket pose in space:
- Position: `bucket_x`, `bucket_y`, `bucket_z`
- Orientation: `bucket_qw`, `bucket_qx`, `bucket_qy`, `bucket_qz`

**Use Case**: Precise bucket positioning tasks (e.g., loading, dumping)

#### Type 1: Chassis Position + Bucket Full Control
- Chassis: `chassis_x`, `chassis_y`
- Bucket: `bucket_heading`, `bucket_altitude`, `bucket_tilt`
- Articulation: `articulation_angle`

**Use Case**: Complex maneuvering with articulated steering

#### Type 2: Chassis Pose + Bucket Attitude
- Chassis: `chassis_x`, `chassis_y`, `chassis_heading`
- Bucket: `bucket_altitude`, `bucket_tilt`

**Use Case**: Standard navigation with bucket control

### Output Messages

#### `WheelLoaderChassisSetpoint`
- Velocity control: `velocity_x`, `yaw_rate`, `articulation_rate`
- Feedforward: `acceleration_x`, `acceleration_y`
- Target state: `target_x`, `target_y`, `target_heading`, `target_articulation`

#### `WheelLoaderBoomSetpoint`
- Position control: `position`, `velocity`, `acceleration`
- Safety limits: `max_velocity`, `max_acceleration`
- Status: `setpoint_valid`, `trajectory_complete`

#### `WheelLoaderTiltSetpoint`
- Position control: `angle`, `angular_velocity`, `angular_acceleration`
- Safety limits: `max_velocity`, `max_acceleration`
- Status: `setpoint_valid`, `trajectory_complete`

## Control Algorithms

### 1. Chassis MPC Controller

**Model**: Articulated vehicle kinematics
```
dx/dt = v * cos(θ)
dy/dt = v * sin(θ)
dθ/dt = ω + (v/L) * sin(α)
dα/dt = α_dot
```

Where:
- `v`: Forward velocity
- `θ`: Heading angle
- `ω`: Yaw rate command
- `α`: Articulation angle
- `L`: Vehicle length

**Cost Function**:
```
J = Σ [Q_pos * ||pos_error||² + Q_heading * heading_error² + 
      Q_art * articulation_error² + R * ||u||² + R_Δ * ||Δu||²]
```

**Optimization**: Simplified gradient descent (production: use QP solver like OSQP)

**Constraints**:
- Velocity: `[-max_vel, +max_vel]`
- Yaw rate: `[-max_yaw_rate, +max_yaw_rate]`
- Articulation: `[-max_art_angle, +max_art_angle]`
- Articulation rate: `[-max_art_rate, +max_art_rate]`

### 2. S-Curve Trajectory Planner

**Profiles Supported**:
- **Quintic Polynomial**: Smooth 5th-order polynomial with continuous jerk
- **7-Segment S-Curve**: Jerk-limited trapezoidal profile

**Quintic Polynomial**:
```
p(t) = a₀ + a₁t + a₂t² + a₃t³ + a₄t⁴ + a₅t⁵
```

Boundary conditions:
- `p(0) = p₀`, `p'(0) = v₀`, `p''(0) = a₀`
- `p(T) = pf`, `p'(T) = 0`, `p''(T) = 0`

**Advantages**:
- Smooth motion without vibration
- Continuous acceleration (no jerks)
- Respects velocity/acceleration limits

### 3. Time Synchronization

All three controllers (chassis, boom, tilt) are synchronized to complete their motions simultaneously:

```python
t_chassis = estimate_chassis_time()
t_boom = boom_planner.get_duration()
t_tilt = tilt_planner.get_duration()

t_sync = max(t_chassis, t_boom, t_tilt)

# Re-plan slower trajectories to match
if t_boom < t_sync:
    boom_planner.plan_trajectory(current, target, t_sync)
if t_tilt < t_sync:
    tilt_planner.plan_trajectory(current, target, t_sync)
```

**Benefits**:
- Coordinated motion
- Prevents mechanical stress
- Predictable arrival times

## Configuration

### Vehicle Parameters (`vehicle_params.yaml`)

```yaml
vehicle:
  front_section_length: 2.5    # m
  rear_section_length: 2.0     # m
  wheelbase: 3.0               # m
  boom_length: 3.5             # m

limits:
  max_forward_velocity: 2.0    # m/s
  max_yaw_rate: 0.5            # rad/s
  max_articulation_angle: 0.785 # rad (45°)
  max_boom_velocity: 0.5       # m/s
  max_tilt_velocity: 0.8       # rad/s
```

### MPC Parameters

```yaml
mpc:
  prediction_horizon: 10
  control_horizon: 5
  sample_time: 0.02  # 50 Hz
  
  weights:
    position_x: 10.0
    position_y: 10.0
    heading: 5.0
    articulation: 3.0
```

### PX4 Parameters

Configured via `module.yaml`:

- `WL_MAX_VEL`: Maximum chassis velocity
- `WL_MAX_YAW_RATE`: Maximum yaw rate
- `WL_MAX_ART_ANG`: Maximum articulation angle
- `WL_BOOM_MAX_VEL`: Maximum boom velocity
- `WL_TILT_MAX_VEL`: Maximum tilt velocity
- `WL_BLEND_DUR`: Trajectory blend duration

## Usage

### 1. Enable the Mode

Add to operation_mode_manager or create dedicated mode switcher:

```cpp
#include "modes/wheel_loader/wheel_loader_traj_follower_mode.hpp"

WheelLoaderTrajFollowerMode _wheel_loader_mode{this};
```

### 2. Publish Trajectory

```cpp
wheel_loader_trajectory_s traj{};
traj.timestamp = hrt_absolute_time();
traj.trajectory_type = wheel_loader_trajectory_s::TRAJ_TYPE_CHASSIS_BUCKET_POSE;
traj.frame_id = wheel_loader_trajectory_s::FRAME_LOCAL;
traj.num_points = 3;

// Point 0: Current position
traj.chassis_x[0] = 0.0f;
traj.chassis_y[0] = 0.0f;
traj.chassis_heading[0] = 0.0f;
traj.bucket_altitude[0] = 0.5f;
traj.bucket_tilt[0] = 0.0f;
traj.point_timestamps[0] = hrt_absolute_time();

// Point 1: Intermediate
traj.chassis_x[1] = 5.0f;
traj.chassis_y[1] = 0.0f;
traj.chassis_heading[1] = 0.0f;
traj.bucket_altitude[1] = 1.0f;
traj.bucket_tilt[1] = 0.3f;
traj.point_timestamps[1] = hrt_absolute_time() + 3_s;

// Point 2: Final
traj.chassis_x[2] = 10.0f;
traj.chassis_y[2] = 2.0f;
traj.chassis_heading[2] = 0.785f;  // 45°
traj.bucket_altitude[2] = 0.5f;
traj.bucket_tilt[2] = 0.0f;
traj.point_timestamps[2] = hrt_absolute_time() + 6_s;

traj.hold_last_point = true;
traj.smooth_transition = true;

// Publish
_wheel_loader_trajectory_pub.publish(traj);
```

### 3. Monitor Setpoints

Subscribe to output topics:
- `wheel_loader_chassis_setpoint`
- `wheel_loader_boom_setpoint`
- `wheel_loader_tilt_setpoint`

## Implementation Notes

### Current State Estimation

The mode requires:
- **Chassis pose**: From `vehicle_local_position` and `vehicle_attitude`
- **Boom position**: TODO - Add sensor subscription
- **Tilt angle**: TODO - Add sensor subscription
- **Articulation angle**: TODO - Add sensor subscription

### Inverse Kinematics (Type 0)

For Type 0 trajectories (bucket 6DOF), inverse kinematics is required to compute chassis/boom/tilt from bucket pose. Current implementation is simplified.

**TODO**: Implement full IK solver considering:
- Vehicle geometry
- Boom kinematics
- Joint limits
- Singularity avoidance

### Frame Transformations

- **FRAME_LOCAL**: No transformation
- **FRAME_BODY**: Transformed using current heading
- **FRAME_GLOBAL**: TODO - Needs proper GPS/map transform

### Trajectory Blending

When `smooth_transition = true`, the mode blends from current state to first trajectory point over `blend_duration` seconds. This prevents sudden motion changes.

## Testing

### SITL Testing

1. Build with wheel loader mode enabled
2. Start SITL with custom vehicle model
3. Publish test trajectories
4. Monitor setpoint outputs
5. Verify time synchronization

### Hardware Testing

1. Configure vehicle parameters
2. Calibrate sensors (position, boom, tilt, articulation)
3. Test individual controllers separately
4. Test full trajectory execution
5. Verify safety limits

## Safety Considerations

1. **Velocity Limits**: All velocities are constrained to configured maximums
2. **Position Limits**: Boom and tilt respect physical joint limits
3. **Trajectory Validation**: Invalid trajectories are rejected
4. **State Monitoring**: Mode checks sensor validity before executing
5. **Emergency Stop**: Mode can be deactivated immediately

## Future Enhancements

1. **Proper QP Solver**: Replace simplified MPC with OSQP or qpOASES
2. **Full IK Solver**: Complete inverse kinematics for Type 0 trajectories
3. **Obstacle Avoidance**: Integrate collision checking
4. **Dynamic Re-planning**: Update trajectory based on sensor feedback
5. **Multi-vehicle Coordination**: Coordinate multiple wheel loaders
6. **Learning-based Control**: Adaptive MPC with learned models
7. **YAML Parameter Loading**: Load parameters from YAML at runtime

## References

- PX4 Operation Mode System: `../README.md`
- MPC Theory: Model Predictive Control textbooks
- S-Curve Planning: "Trajectory Planning for Automatic Machines and Robots"
- Articulated Vehicle Control: Mobile robot kinematics literature

## Questions?

For implementation questions or issues, see the main operation_mode README or contact the development team.
