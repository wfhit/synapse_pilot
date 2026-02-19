/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "traction_controller.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

TractionController::TractionController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

TractionController::~TractionController()
{
	perf_free(_loop_perf);
	perf_free(_slip_detect_perf);
	perf_free(_stall_detect_perf);
}

bool TractionController::init()
{
	// Apply PID gains from parameters
	_slip_controller_front.setGains(_param_slip_p.get(), _param_slip_i.get(), _param_slip_d.get());
	_slip_controller_front.setOutputLimit(1.0f);
	_slip_controller_front.setIntegralLimit(1.0f);

	_slip_controller_rear.setGains(_param_slip_p.get(), _param_slip_i.get(), _param_slip_d.get());
	_slip_controller_rear.setOutputLimit(1.0f);
	_slip_controller_rear.setIntegralLimit(1.0f);

	_yaw_rate_controller.setGains(_param_yaw_p.get(), _param_yaw_i.get(), _param_yaw_d.get());
	_yaw_rate_controller.setOutputLimit(1.0f);
	_yaw_rate_controller.setIntegralLimit(1.0f);

	ScheduleOnInterval(1000000 / UPDATE_RATE_HZ); // 50Hz
	return true;
}

void TractionController::Run()
{
	if (should_exit()) {
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Update all state information
	update_vehicle_state();
	update_drivetrain_feedback();
	update_traction_setpoint();

	// Check if traction control should be active
	vehicle_control_mode_s control_mode;

	if (_vehicle_control_mode_sub.copy(&control_mode)) {
		_traction_control_active = control_mode.flag_control_velocity_enabled &&
					   _param_enable.get();
	}

	if (!_traction_control_active) {
		perf_end(_loop_perf);
		return;
	}

	// Core traction control pipeline
	estimate_drivetrain_slip();
	estimate_ground_conditions();

	// Compute velocity commands based on slip control
	compute_velocity_commands();

	// Compute force distribution between drivetrains
	compute_force_distribution();

	// Compute steering compensation for slip
	compute_steering_compensation();

	// Check for wheel stall conditions
	detect_wheel_stall();

	// Handle excessive slip events
	if (_front_drivetrain.is_slipping || _rear_drivetrain.is_slipping) {
		handle_excessive_slip();
	}

	// Publish commands to drivetrain and steering controllers
	publish_drivetrain_commands();
	publish_steering_command();

	_last_update = hrt_absolute_time();

	perf_end(_loop_perf);
}

void TractionController::update_vehicle_state()
{
	// Get velocity from EKF
	vehicle_local_position_s local_pos;

	if (_vehicle_local_position_sub.copy(&local_pos)) {
		_vehicle.velocity_body(0) = local_pos.vx;
		_vehicle.velocity_body(1) = local_pos.vy;
		_vehicle.velocity_body(2) = local_pos.vz;

		_vehicle.ground_speed = sqrtf(local_pos.vx * local_pos.vx +
					      local_pos.vy * local_pos.vy);
	}

	// Get acceleration
	vehicle_acceleration_s accel;

	if (_vehicle_acceleration_sub.copy(&accel)) {
		_vehicle.acceleration_body(0) = accel.xyz[0];
		_vehicle.acceleration_body(1) = accel.xyz[1];
		_vehicle.acceleration_body(2) = accel.xyz[2];
	}

	// Get angular velocity
	vehicle_angular_velocity_s angular_vel;

	if (_vehicle_angular_velocity_sub.copy(&angular_vel)) {
		_vehicle.yaw_rate = angular_vel.xyz[2];
	}

	// Calculate sideslip angle
	if (_vehicle.ground_speed > MIN_GROUND_SPEED) {
		_vehicle.sideslip_angle = atan2f(_vehicle.velocity_body(1),
						 _vehicle.velocity_body(0));
	}
}

void TractionController::update_drivetrain_feedback()
{
	// Get drivetrain status from drivetrain controllers
	// Front drivetrain (instance 0) and rear drivetrain (instance 1)

	drivetrain_status_s front_status{}, rear_status{};
	bool front_updated = false, rear_updated =
				     false;    // Check all drivetrain status instances to find front and rear drivetrains

	for (uint8_t i = 0; i < _drivetrain_status_sub.size(); i++) {
		drivetrain_status_s status;

		if (_drivetrain_status_sub[i].updated() && _drivetrain_status_sub[i].copy(&status)) {
			if (status.is_front_wheel && !front_updated) {
				front_status = status;
				front_updated = true;

			} else if (!status.is_front_wheel && !rear_updated) {
				rear_status = status;
				rear_updated = true;
			}
		}
	}

	// Update front drivetrain data
	if (front_updated) {
		// Convert RPM to rad/s
		_front_drivetrain.motor_speed = front_status.current_speed_rpm * (2.0f * M_PI_F / 60.0f);

		// For wheel speed, use encoder-based speed if available, otherwise current speed
		if (front_status.encoder_healthy) {
			_front_drivetrain.wheel_speed = front_status.encoder_speed_rpm * (2.0f * M_PI_F / 60.0f);

		} else {
			_front_drivetrain.wheel_speed = _front_drivetrain.motor_speed;
		}
	}

	// Update rear drivetrain data
	if (rear_updated) {
		// Convert RPM to rad/s
		_rear_drivetrain.motor_speed = rear_status.current_speed_rpm * (2.0f * M_PI_F / 60.0f);

		// For wheel speed, use encoder-based speed if available, otherwise current speed
		if (rear_status.encoder_healthy) {
			_rear_drivetrain.wheel_speed = rear_status.encoder_speed_rpm * (2.0f * M_PI_F / 60.0f);

		} else {
			_rear_drivetrain.wheel_speed = _rear_drivetrain.motor_speed;
		}
	}
}

void TractionController::update_traction_setpoint()
{
	// Get traction setpoint from chassis trajectory follower
	if (_traction_setpoint_sub.copy(&_traction_setpoint)) {
		// Extract desired motion commands
		_desired_velocity = _traction_setpoint.desired_velocity_ms;
		_desired_yaw_rate = _traction_setpoint.desired_yaw_rate_rad_s;

		// Update control parameters based on traction setpoint
		_control_params.target_slip_ratio = _traction_setpoint.max_slip_ratio * 0.5f; // Use 50% of max as target
		_control_params.max_slip_ratio = _traction_setpoint.max_slip_ratio;

		// Update force distribution based on torque bias
		// torque_bias: -1.0=rear bias, 0.0=balanced, 1.0=front bias
		_control_params.force_distribution = 0.5f + (_traction_setpoint.torque_bias * 0.3f); // 0.2-0.8 range

		// Update surface-specific parameters
		if (_traction_setpoint.surface_type != traction_setpoint_s::SURFACE_UNKNOWN) {
			// Adjust friction estimate based on surface type
			switch (_traction_setpoint.surface_type) {
			case traction_setpoint_s::SURFACE_CONCRETE:
				_estimated_friction = 0.9f * _traction_setpoint.target_traction_coefficient;
				break;

			case traction_setpoint_s::SURFACE_GRAVEL:
				_estimated_friction = 0.7f * _traction_setpoint.target_traction_coefficient;
				break;

			case traction_setpoint_s::SURFACE_SAND:
				_estimated_friction = 0.5f * _traction_setpoint.target_traction_coefficient;
				break;

			case traction_setpoint_s::SURFACE_MUD:
				_estimated_friction = 0.3f * _traction_setpoint.target_traction_coefficient;
				break;

			case traction_setpoint_s::SURFACE_SNOW:
				_estimated_friction = 0.25f * _traction_setpoint.target_traction_coefficient;
				break;

			case traction_setpoint_s::SURFACE_MIXED:
				_estimated_friction = 0.6f * _traction_setpoint.target_traction_coefficient;
				break;

			default: // SURFACE_UNKNOWN
				_estimated_friction = 0.7f; // Default
				break;
			}
		}

		// Update emergency stop state
		if (_traction_setpoint.emergency_stop) {
			_desired_velocity = 0.0f;
			_desired_yaw_rate = 0.0f;
			_desired_force = 0.0f;
		}
	}
}

void TractionController::estimate_drivetrain_slip()
{
	// Estimate slip for each drivetrain
	_front_drivetrain.slip = calculate_drivetrain_slip(
					 _front_drivetrain.wheel_speed,
					 _vehicle.ground_speed,
					 _vehicle.articulation_angle,
					 true
				 );

	_rear_drivetrain.slip = calculate_drivetrain_slip(
					_rear_drivetrain.wheel_speed,
					_vehicle.ground_speed,
					_vehicle.articulation_angle,
					false
				);

	// Determine if slipping
	float slip_threshold = _param_max_slip.get();

	if (fabsf(_front_drivetrain.slip.longitudinal) > slip_threshold) {
		_front_drivetrain.is_slipping = true;
		perf_count(_slip_detect_perf);
		_last_slip_event = hrt_absolute_time();

	} else {
		_front_drivetrain.is_slipping = false;
	}

	if (fabsf(_rear_drivetrain.slip.longitudinal) > slip_threshold) {
		_rear_drivetrain.is_slipping = true;
		perf_count(_slip_detect_perf);
		_last_slip_event = hrt_absolute_time();

	} else {
		_rear_drivetrain.is_slipping = false;
	}
}

TractionController::SlipEstimate TractionController::calculate_drivetrain_slip(
	float wheel_speed, float vehicle_speed, float articulation_angle, bool is_front_drivetrain)
{
	SlipEstimate slip;
	slip.timestamp = hrt_absolute_time();

	// Calculate wheel linear velocity
	float wheel_linear_speed = wheel_speed * _param_wheel_radius.get();

	// Adjust for articulated steering kinematics
	float effective_speed = vehicle_speed;

	if (fabsf(articulation_angle) > 0.01f && vehicle_speed > MIN_GROUND_SPEED) {
		float wheelbase = _param_wheelbase.get();
		float turn_radius = wheelbase / tanf(articulation_angle);

		if (is_front_drivetrain) {
			// Front drivetrain outer radius adjustment
			effective_speed *= (1.0f + wheelbase / (2.0f * fabsf(turn_radius)));

		} else {
			// Rear drivetrain inner radius adjustment
			effective_speed *= (1.0f - wheelbase / (2.0f * fabsf(turn_radius)));
		}
	}

	// Calculate longitudinal slip
	if (effective_speed > MIN_GROUND_SPEED) {
		slip.longitudinal = (wheel_linear_speed - effective_speed) / effective_speed;

	} else if (wheel_linear_speed > MIN_GROUND_SPEED) {
		slip.longitudinal = 1.0f; // Pure spinning

	} else {
		slip.longitudinal = 0.0f;
	}

	// Constrain slip
	slip.longitudinal = math::constrain(slip.longitudinal, -1.0f, 1.0f);

	// Estimate lateral slip
	if (is_front_drivetrain) {
		slip.lateral = _vehicle.sideslip_angle + articulation_angle / 2.0f;

	} else {
		slip.lateral = _vehicle.sideslip_angle - articulation_angle / 2.0f;
	}

	// Confidence based on speed
	slip.confidence = math::constrain(vehicle_speed / 1.0f, 0.1f, 1.0f);

	return slip;
}

void TractionController::estimate_ground_conditions()
{
	// Estimate surface type from slip behavior
	float avg_slip = (fabsf(_front_drivetrain.slip.longitudinal) +
			  fabsf(_rear_drivetrain.slip.longitudinal)) / 2.0f;

	// Surface estimation based on slip characteristics
	if (avg_slip < 0.05f) {
		_surface_type = SurfaceType::HARD_ROCK;
		_estimated_friction = 0.9f;

	} else if (avg_slip < 0.15f) {
		_surface_type = SurfaceType::GRAVEL;
		_estimated_friction = 0.7f;

	} else if (avg_slip < 0.3f) {
		_surface_type = SurfaceType::LOOSE_SOIL;
		_estimated_friction = 0.5f;

	} else {
		_surface_type = SurfaceType::MUD;
		_estimated_friction = 0.3f;
	}
}

void TractionController::compute_velocity_commands()
{
	// Base velocity from desired speed
	float base_velocity = _desired_velocity;

	// Target slip from parameters
	float target_slip = _param_target_slip.get();

	// Compute slip errors
	float front_slip_error = target_slip - _front_drivetrain.slip.longitudinal;
	float rear_slip_error = target_slip - _rear_drivetrain.slip.longitudinal;

	// PID control for slip regulation
	float dt = (_last_update > 0) ? (hrt_absolute_time() - _last_update) * 1e-6f : 0.02f;

	float front_correction = _slip_controller_front.update(front_slip_error, dt);
	float rear_correction = _slip_controller_rear.update(rear_slip_error, dt);

	// Apply corrections to velocity commands
	_front_velocity_cmd = base_velocity * (1.0f + front_correction);
	_rear_velocity_cmd = base_velocity * (1.0f + rear_correction);

	// Limit velocity difference between drivetrains for stability
	float max_diff = base_velocity * 0.2f; // 20% maximum difference
	float diff = _front_velocity_cmd - _rear_velocity_cmd;

	if (fabsf(diff) > max_diff) {
		float avg = (_front_velocity_cmd + _rear_velocity_cmd) / 2.0f;
		_front_velocity_cmd = avg + max_diff * (diff > 0 ? 0.5f : -0.5f);
		_rear_velocity_cmd = avg - max_diff * (diff > 0 ? 0.5f : -0.5f);
	}
}

void TractionController::compute_force_distribution()
{
	// Start with default distribution
	float front_ratio = _param_force_distribution.get();

	// Adjust for slip conditions
	if (_front_drivetrain.is_slipping && !_rear_drivetrain.is_slipping) {
		// Reduce front force
		front_ratio = fmaxf(0.2f, front_ratio - 0.2f);

	} else if (_rear_drivetrain.is_slipping && !_front_drivetrain.is_slipping) {
		// Reduce rear force
		front_ratio = fminf(0.8f, front_ratio + 0.2f);
	}

	// Apply distribution
	_front_force_cmd = _desired_force * front_ratio;
	_rear_force_cmd = _desired_force * (1.0f - front_ratio);

	// Apply maximum force limits
	limit_wheel_forces();
}

void TractionController::limit_wheel_forces()
{
	// Apply maximum force limits from parameters
	float max_force = _param_max_force.get();
	_front_force_cmd = math::constrain(_front_force_cmd, -max_force, max_force);
	_rear_force_cmd = math::constrain(_rear_force_cmd, -max_force, max_force);
}

void TractionController::compute_steering_compensation()
{
	// Calculate desired articulation for trajectory following
	float desired_articulation = calculate_kinematic_steering_rate(_desired_yaw_rate);

	// Use yaw rate controller to maintain desired yaw rate
	float dt = (_last_update > 0) ? (hrt_absolute_time() - _last_update) * 1e-6f : 0.02f;
	float yaw_rate_error = _desired_yaw_rate - _vehicle.yaw_rate;
	float yaw_compensation = _yaw_rate_controller.update(yaw_rate_error, dt);

	// Combine compensations
	_articulation_cmd = desired_articulation + yaw_compensation;

	// Apply limits
	_articulation_cmd = math::constrain(_articulation_cmd,
					    -_param_max_articulation.get(),
					    _param_max_articulation.get());

	// Rate limiting for smooth operation
	float rate = (_articulation_cmd - _last_articulation_cmd) / dt;

	if (fabsf(rate) > ARTICULATION_RATE_LIMIT) {
		_articulation_cmd = _last_articulation_cmd + ARTICULATION_RATE_LIMIT * dt * (rate > 0 ? 1.0f : -1.0f);
	}

	_last_articulation_cmd = _articulation_cmd;
}

float TractionController::calculate_kinematic_steering_rate(float desired_yaw_rate)
{
	float wheelbase = _param_wheelbase.get();

	if (_vehicle.ground_speed > MIN_GROUND_SPEED) {
		return atanf(wheelbase * desired_yaw_rate / _vehicle.ground_speed);

	} else {
		return 0.0f;
	}
}

void TractionController::detect_wheel_stall()
{
	// Detect if wheels are stalled (not rotating despite commands)
	bool front_stalled = (_front_velocity_cmd > MIN_GROUND_SPEED &&
			      _front_drivetrain.wheel_speed < _param_stall_threshold.get());
	bool rear_stalled = (_rear_velocity_cmd > MIN_GROUND_SPEED &&
			     _rear_drivetrain.wheel_speed < _param_stall_threshold.get());

	if (front_stalled || rear_stalled) {
		perf_count(_stall_detect_perf);
		_last_stall_event = hrt_absolute_time();

		// Reduce force commands if stalled
		if (front_stalled) {
			_front_force_cmd *= 0.5f;
			PX4_WARN("Front wheel stall detected");
		}

		if (rear_stalled) {
			_rear_force_cmd *= 0.5f;
			PX4_WARN("Rear wheel stall detected");
		}
	}
}

void TractionController::handle_excessive_slip()
{
	// Handle excessive slip events

	if (_front_drivetrain.is_slipping) {
		// Reduce front velocity and force
		_front_velocity_cmd *= 0.8f;
		_front_force_cmd *= 0.7f;
	}

	if (_rear_drivetrain.is_slipping) {
		// Reduce rear velocity and force
		_rear_velocity_cmd *= 0.8f;
		_rear_force_cmd *= 0.7f;
	}

	// If both slipping, reduce overall commands
	if (_front_drivetrain.is_slipping && _rear_drivetrain.is_slipping) {
		_front_velocity_cmd *= 0.7f;
		_rear_velocity_cmd *= 0.7f;
		_front_force_cmd *= 0.6f;
		_rear_force_cmd *= 0.6f;
	}
}

void TractionController::publish_drivetrain_commands()
{
	// Publish drivetrain setpoints for front and rear drivetrains

	// Front drivetrain setpoint
	drivetrain_setpoint_s front_setpoint{};
	front_setpoint.timestamp = hrt_absolute_time();
	front_setpoint.wheel_speed_rad_s = _front_velocity_cmd / _param_wheel_radius.get(); // Convert m/s to rad/s
	front_setpoint.wheel_acceleration_rad_s2 = 2.0f; // Default acceleration limit
	front_setpoint.wheel_torque_nm = _front_force_cmd * _param_wheel_radius.get(); // Convert force to torque
	front_setpoint.control_mode = drivetrain_setpoint_s::MODE_SPEED_CONTROL;
	front_setpoint.enable_traction_control = _traction_control_active;
	front_setpoint.emergency_stop = false;
	front_setpoint.torque_limit_nm = _param_max_force.get() * _param_wheel_radius.get();

	// Rear drivetrain setpoint
	drivetrain_setpoint_s rear_setpoint{};
	rear_setpoint.timestamp = hrt_absolute_time();
	rear_setpoint.wheel_speed_rad_s = _rear_velocity_cmd / _param_wheel_radius.get(); // Convert m/s to rad/s
	rear_setpoint.wheel_acceleration_rad_s2 = 2.0f; // Default acceleration limit
	rear_setpoint.wheel_torque_nm = _rear_force_cmd * _param_wheel_radius.get(); // Convert force to torque
	rear_setpoint.control_mode = drivetrain_setpoint_s::MODE_SPEED_CONTROL;
	rear_setpoint.enable_traction_control = _traction_control_active;
	rear_setpoint.emergency_stop = false;
	rear_setpoint.torque_limit_nm = _param_max_force.get() * _param_wheel_radius.get();

	// Publish setpoints
	_drivetrain_setpoint_front_pub.publish(front_setpoint);
	_drivetrain_setpoint_rear_pub.publish(rear_setpoint);
}

void TractionController::publish_steering_command()
{
	// Publish steering command
	steering_setpoint_s steering_sp{};
	steering_sp.timestamp = hrt_absolute_time();
	steering_sp.steering_angle_rad = _articulation_cmd;
	steering_sp.steering_rate_rad_s = _desired_yaw_rate; // Feed-forward

	_steering_setpoint_pub.publish(steering_sp);
}

float TractionController::estimate_friction_coefficient()
{
	return _estimated_friction;
}

int TractionController::task_spawn(int argc, char *argv[])
{
	TractionController *instance = new TractionController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int TractionController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int TractionController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Traction Control Module for Articulated Wheel Loader

This module provides slip-based traction control optimized for mining operations:
- Estimates slip for front/rear drivetrains using motor encoders and EKF velocity
- Provides steering compensation for articulated vehicle
- Optimizes traction and stability for maximum performance

The module acts as an intermediary between control inputs and actuators,
optimizing traction based on slip estimation and ground conditions.

### Implementation
Currently configured to work with manual control or autonomous planner inputs.
Publishes steering commands to steering controller.
Note: Direct wheel control implementation needs to be added based on specific actuator setup.

### Examples
Start with default parameters:
$ traction_control start

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("traction_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int traction_control_main(int argc, char *argv[])
{
    return TractionController::main(argc, argv);
}
