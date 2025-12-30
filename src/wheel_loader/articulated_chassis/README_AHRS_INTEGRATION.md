# AHRS (EKF2) Integration with Articulated Chassis Control

## Overview

This document outlines the design for integrating AHRS (Attitude and Heading Reference System) using EKF2 with articulated chassis control for wheel loader vehicles. The integration enhances state estimation, slip detection, and overall vehicle stability in challenging terrain conditions.

## Architecture

### Core Components

1. **Enhanced Slip Estimator** - Integrates EKF2 innovation data for improved slip detection
2. **Terrain Classification System** - Uses EKF2 metrics to classify surface conditions
3. **Predictive Stability Control** - Combines attitude prediction with articulation dynamics
4. **Multi-Mode Navigation Filter** - Adaptive EKF2 parameters for different operation modes
5. **Articulation-Aware Dead Reckoning** - Enhanced position estimation using vehicle kinematics
6. **Load-Adaptive Control** - Dynamic parameter adjustment based on bucket load
7. **Sensor Fusion Health Monitor** - Real-time assessment of fusion quality

## Integration Points

### 1. SlipEstimator Enhancement

The existing `SlipEstimator` class is enhanced to subscribe to EKF2 outputs:

```cpp
// Additional EKF2 subscriptions
uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
uORB::Subscription _estimator_innovations_sub{ORB_ID(estimator_innovations)};
uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
```

### 2. Innovation-Based Slip Detection

Uses EKF2 innovation statistics to detect wheel slip:

```cpp
void SlipEstimator::runEKFEstimation(slip_estimation_s &slip)
{
    // Get EKF2 innovations for cross-validation
    estimator_innovations_s innovations;
    if (_estimator_innovations_sub.update(&innovations)) {
        // Use velocity innovations to detect slip
        float vel_innovation_mag = sqrtf(innovations.vel_pos_innov[0] * innovations.vel_pos_innov[0] +
                                         innovations.vel_pos_innov[1] * innovations.vel_pos_innov[1]);

        // High velocity innovation indicates potential slip
        if (vel_innovation_mag > _param_slip_innovation_threshold.get()) {
            slip.slip_detected = true;
            slip.confidence *= 0.8f; // Reduce confidence during high innovation
        }
    }
}
```

### 3. EKF2 Configuration for Articulated Vehicles

Specific parameter optimization for articulated vehicle dynamics:

```cpp
void WheelLoaderRobot::configureEKF2ForArticulated()
{
    // Set EKF2 parameters optimized for articulated vehicles
    param_set(param_find("EKF2_OF_CTRL"), &(int32_t){0}); // Disable optical flow
    param_set(param_find("EKF2_EVP_NOISE"), &(float){0.1f}); // External vision position noise
    param_set(param_find("EKF2_EVV_NOISE"), &(float){0.1f}); // External vision velocity noise

    // Enable wheel odometry fusion (custom extension)
    param_set(param_find("EKF2_WHEEL_CTRL"), &(int32_t){1}); // Enable wheel odometry

    // Adjust process noise for ground vehicles
    param_set(param_find("EKF2_ACC_NOISE"), &(float){0.5f}); // Higher for rough terrain
    param_set(param_find("EKF2_GYR_NOISE"), &(float){0.01f}); // Lower for stable platform
}
```

## Advanced Features

### 1. Terrain Classification System

Automatically classifies terrain based on sensor fusion metrics:

```cpp
class TerrainClassifier : public ModuleBase<TerrainClassifier>
{
public:
    enum class TerrainType {
        PAVED,     // Low slip, stable estimation
        GRAVEL,    // Medium slip, moderate vibration
        SAND,      // High slip, low traction
        MUD,       // Variable slip, high damping
        ROCKY      // High vibration, impact loads
    };

    TerrainType classifyTerrain(const estimator_status_s &status,
                                const slip_estimation_s &slip);

private:
    // Classification features
    float _vibration_metrics[3];     // XYZ vibration levels
    float _slip_history[100];        // Recent slip values
    float _innovation_variance[6];   // EKF2 innovation statistics
    float _confidence_threshold;     // Classification confidence
};
```

**Classification Algorithm:**
- **Paved**: Low vibration (< 0.1 m/s²), minimal slip (< 5%), stable innovations
- **Gravel**: Medium vibration (0.1-0.5 m/s²), moderate slip (5-15%), consistent patterns
- **Sand**: High slip (> 20%), low friction coefficient, velocity innovation spikes
- **Mud**: Variable slip patterns, high lateral slip, damped vibrations
- **Rocky**: High-frequency vibrations (> 1 Hz), impact signatures, orientation disturbances

### 2. Predictive Stability Control

Combines EKF2 attitude prediction with articulation kinematics:

```cpp
class StabilityControl
{
public:
    struct StabilityMetrics {
        float rollover_risk;      // 0-1 scale (0 = stable, 1 = imminent rollover)
        float lateral_stability;  // lateral acceleration margin (m/s²)
        float load_distribution;  // front/rear axle load balance
        bool limit_speed;         // recommend speed reduction
        bool limit_steering;      // recommend steering limitation
        float max_safe_speed;     // maximum recommended speed (m/s)
        float max_safe_steering;  // maximum recommended steering angle (rad)
    };

    StabilityMetrics assessStability(const vehicle_attitude_s &attitude,
                                    const articulation_status_s &articulation,
                                    const boom_bucket_status_s &load_status);

private:
    // Stability assessment parameters
    float _cg_height;            // Center of gravity height
    float _track_width;          // Vehicle track width
    float _wheelbase_front;      // Front axle to articulation point
    float _wheelbase_rear;       // Rear axle to articulation point
    float _max_lateral_accel;    // Maximum safe lateral acceleration

    // Dynamic stability margins
    float calculateRolloverThreshold(float load_mass, float cg_offset);
    float calculateLateralStabilityMargin(const matrix::Vector3f &attitude_rates);
    void predictTrajectory(float steering_angle, float speed, float dt);
};
```

**Stability Assessment Logic:**
1. **Rollover Risk Calculation**: Uses articulation angle, load position, and lateral acceleration
2. **Dynamic Load Transfer**: Estimates front/rear axle load distribution during maneuvers
3. **Predictive Analysis**: Projects vehicle trajectory over next 2-3 seconds
4. **Safety Margins**: Implements configurable safety factors for different operation modes

### 3. Multi-Mode Navigation Filter

Adaptive EKF2 operation for different vehicle tasks:

```cpp
class NavigationModeManager
{
public:
    enum class NavMode {
        LOADING,      // High slip tolerance, reduced position accuracy requirements
        TRANSPORT,    // Standard navigation, balanced accuracy/robustness
        PRECISION,    // High accuracy for autonomous docking operations
        ROUGH_TERRAIN // Increased process noise, enhanced vibration rejection
    };

    void switchMode(NavMode mode);
    void adjustEKF2Parameters(NavMode mode);

private:
    struct ModeParameters {
        float acc_noise;          // Accelerometer process noise
        float gyro_noise;         // Gyroscope process noise
        float pos_obs_noise;      // Position observation noise
        float vel_obs_noise;      // Velocity observation noise
        float innovation_gate;    // Innovation consistency gate
        bool enable_wheel_fusion; // Wheel odometry fusion enable
        float wheel_noise;        // Wheel speed measurement noise
    };

    std::map<NavMode, ModeParameters> _mode_configs;
    NavMode _current_mode{NavMode::TRANSPORT};

    void configureModeParameters();
};
```

**Mode Configurations:**

| Parameter | Loading | Transport | Precision | Rough Terrain |
|-----------|---------|-----------|-----------|---------------|
| Acc Noise | 0.8 | 0.5 | 0.3 | 1.2 |
| Gyro Noise | 0.02 | 0.01 | 0.005 | 0.03 |
| Pos Obs Noise | 0.5 | 0.3 | 0.1 | 0.8 |
| Innovation Gate | 5.0 | 3.0 | 2.0 | 8.0 |
| Wheel Fusion | Yes | Yes | Yes | Limited |

### 4. Articulation-Aware Dead Reckoning

Enhanced position estimation using vehicle kinematics:

```cpp
class ArticulatedDeadReckoning
{
public:
    // Ackermann steering model with articulation joint
    matrix::Vector3f predictPosition(float front_wheel_speed,
                                   float rear_wheel_speed,
                                   float articulation_angle,
                                   float dt);

    // Fuse predictions with EKF2 when GPS is available
    void updateWithEKF2(const vehicle_local_position_s &ekf_pos);

    // Get current position estimate
    matrix::Vector3f getPosition() const { return _position; }
    matrix::Vector3f getVelocity() const { return _velocity; }
    float getHeading() const { return _heading; }

private:
    // Vehicle kinematic parameters
    float _wheelbase_front;      // Distance from front axle to articulation
    float _wheelbase_rear;       // Distance from rear axle to articulation
    float _wheel_radius;         // Wheel radius

    // State variables
    matrix::Vector3f _position{0.0f, 0.0f, 0.0f};  // X, Y, Z position
    matrix::Vector3f _velocity{0.0f, 0.0f, 0.0f};  // X, Y, Z velocity
    float _heading{0.0f};        // Vehicle heading
    float _front_heading{0.0f};  // Front section heading
    float _rear_heading{0.0f};   // Rear section heading

    // Kinematic model implementation
    void updateArticulatedKinematics(float front_speed, float rear_speed,
                                   float articulation_angle, float dt);
    matrix::Vector2f calculateInstantaneousCenter(float articulation_angle);
};
```

### 5. Load-Adaptive Control

Dynamic parameter adjustment based on bucket load and operation:

```cpp
class LoadAdaptiveControl
{
public:
    struct LoadState {
        float bucket_load_kg;     // Current bucket load mass
        float boom_angle;         // Boom elevation angle
        float bucket_angle;       // Bucket tilt angle
        matrix::Vector3f cg_offset; // Center of gravity offset from nominal
    };

    void adaptToLoad(const LoadState &load_state);
    void adaptToTerrain(TerrainClassifier::TerrainType terrain);

private:
    // Adaptive parameter ranges
    struct AdaptiveParams {
        float min_acc_noise{0.3f};
        float max_acc_noise{1.5f};
        float min_steering_response{0.5f};
        float max_steering_response{2.0f};
        float min_traction_gain{0.8f};
        float max_traction_gain{1.5f};
    } _params;

    // Load-based adaptations
    void adjustEKF2ForLoad(float load_kg);
    void adjustSteeringResponse(float load_kg);
    void adjustTractionControl(float load_kg);

    // Terrain-based adaptations
    void adjustForTerrain(TerrainClassifier::TerrainType terrain);
};
```

**Adaptation Logic:**
- **Light Load (0-500kg)**: Standard parameters, high responsiveness
- **Medium Load (500-1500kg)**: Increased stability margins, moderate responsiveness
- **Heavy Load (>1500kg)**: Maximum stability, reduced aggressiveness
- **High CG**: Enhanced rollover protection, limited lateral acceleration

### 6. Sensor Fusion Health Monitor

Real-time assessment and visualization of fusion quality:

```cpp
class FusionHealthMonitor
{
public:
    struct FusionHealth {
        float imu_health{1.0f};              // IMU data quality (0-1)
        float wheel_odometry_health{1.0f};   // Wheel encoder quality (0-1)
        float gps_health{1.0f};              // GPS signal quality (0-1)
        float overall_confidence{1.0f};       // Overall fusion confidence (0-1)
        bool degraded_mode{false};           // Operating in degraded mode
        uint8_t failed_sensors{0};           // Bitmask of failed sensors
        float time_since_gps{0.0f};          // Time since last GPS update
    };

    FusionHealth assessHealth(const estimator_status_s &ekf_status,
                            const slip_estimation_s &slip_status);

    // Health monitoring thresholds
    struct HealthThresholds {
        float min_imu_health{0.7f};
        float min_gps_health{0.6f};
        float max_innovation_variance{2.0f};
        float max_gps_timeout{5.0f};        // seconds
        float min_wheel_consistency{0.8f};
    } _thresholds;

private:
    // Health assessment methods
    float assessIMUHealth(const estimator_status_s &status);
    float assessGPSHealth(const estimator_status_s &status);
    float assessWheelOdometryHealth(const slip_estimation_s &slip);
    bool checkInnovationConsistency(const estimator_innovations_s &innovations);

    // Health history for trend analysis
    static constexpr size_t HEALTH_HISTORY_SIZE = 100;
    float _imu_health_history[HEALTH_HISTORY_SIZE];
    float _gps_health_history[HEALTH_HISTORY_SIZE];
    size_t _history_index{0};
};
```

## Implementation Priority

### Phase 1: Foundation (Weeks 1-4)
1. **Basic EKF2 Integration**
   - Add EKF2 subscriptions to SlipEstimator
   - Implement innovation-based slip detection
   - Configure EKF2 parameters for articulated vehicles

2. **Enhanced Slip Detection**
   - Integrate EKF2 innovation data
   - Implement cross-validation between wheel odometry and EKF2
   - Add slip confidence metrics

### Phase 2: Advanced Features (Weeks 5-8)
3. **Terrain Classification**
   - Implement basic terrain classification
   - Add vibration analysis
   - Integrate slip patterns for surface identification

4. **Stability Control**
   - Develop rollover prediction algorithm
   - Implement load-based stability assessment
   - Add predictive trajectory analysis

### Phase 3: Optimization (Weeks 9-12)
5. **Multi-Mode Navigation**
   - Implement adaptive EKF2 parameter switching
   - Add operation mode detection
   - Optimize parameters for each mode

6. **Load Adaptation**
   - Implement load-based parameter adjustment
   - Add center of gravity estimation
   - Integrate with boom/bucket control

### Phase 4: Integration & Testing (Weeks 13-16)
7. **Health Monitoring**
   - Implement fusion health assessment
   - Add degraded mode operations
   - Create diagnostic interfaces

8. **System Integration**
   - Integrate all components
   - Comprehensive testing and validation
   - Performance optimization

## Testing Strategy

### Simulation Testing
- **Gazebo Integration**: Test in various terrain conditions
- **SITL Testing**: Validate algorithms without hardware
- **Parameter Sweeps**: Optimize configuration parameters

### Hardware Testing
- **Controlled Environment**: Validate on known surfaces
- **Field Testing**: Real-world validation in various conditions
- **Edge Cases**: Test failure modes and recovery

### Performance Metrics
- **Position Accuracy**: Compare EKF2 position with ground truth
- **Slip Detection Rate**: Measure true/false positive rates
- **Stability Margins**: Validate rollover prediction accuracy
- **Computational Load**: Monitor CPU usage and real-time performance

## Configuration Parameters

### EKF2 Parameters
```cpp
// Articulated vehicle specific parameters
PARAM_DEFINE_FLOAT(EKF2_ART_ACC_NOISE, 0.5f);    // Articulated vehicle acceleration noise
PARAM_DEFINE_FLOAT(EKF2_ART_GYR_NOISE, 0.01f);   // Articulated vehicle gyro noise
PARAM_DEFINE_INT32(EKF2_ART_WHEEL_EN, 1);        // Enable wheel odometry fusion
PARAM_DEFINE_FLOAT(EKF2_ART_WHEEL_NOISE, 0.1f);  // Wheel speed measurement noise

// Slip detection parameters
PARAM_DEFINE_FLOAT(SE_INNOV_THRESH, 1.0f);       // Innovation threshold for slip detection
PARAM_DEFINE_FLOAT(SE_SLIP_CONF_MIN, 0.7f);      // Minimum confidence for slip detection
PARAM_DEFINE_INT32(SE_TERRAIN_EN, 1);            // Enable terrain classification

// Stability control parameters
PARAM_DEFINE_FLOAT(SC_ROLLOVER_THRESH, 0.8f);    // Rollover risk threshold
PARAM_DEFINE_FLOAT(SC_LATERAL_MARGIN, 2.0f);     // Lateral stability margin (m/s²)
PARAM_DEFINE_FLOAT(SC_LOAD_ADAPT_GAIN, 0.5f);    // Load adaptation gain
```

## Future Enhancements

### Machine Learning Integration
- **Terrain Classification ML**: Use neural networks for improved terrain recognition
- **Predictive Maintenance**: Monitor sensor health trends
- **Adaptive Control**: Learn optimal parameters for specific conditions

### Advanced Sensors
- **Lidar Integration**: Enhanced terrain mapping and obstacle detection
- **Camera Vision**: Visual odometry and object recognition
- **Radar**: All-weather velocity measurement

### Autonomous Features
- **Path Planning**: Integrate with mission planning systems
- **Autonomous Docking**: Precision control for loading operations
- **Fleet Coordination**: Multi-vehicle operation support

## Conclusion

This AHRS (EKF2) integration design provides a comprehensive framework for enhancing articulated chassis control through improved state estimation, terrain awareness, and predictive safety features. The modular architecture allows for incremental implementation and testing while maintaining system reliability and performance.

The integration leverages EKF2's robust sensor fusion capabilities while addressing the unique challenges of articulated vehicles operating in challenging terrain conditions. The result is a more intelligent, adaptive, and safe control system suitable for autonomous and semi-autonomous wheel loader operations.
