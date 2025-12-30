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

#include "boom_motion_controller.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

BoomMotionController::BoomMotionController(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool BoomMotionController::initialize()
{
	updateParams();

	// Initialize PID controllers
	_position_controller.setGains(_param_pos_p.get(), _param_pos_i.get(), _param_pos_d.get());
	_velocity_controller.setGains(_param_vel_p.get(), _param_vel_i.get(), _param_vel_d.get());

	// Initialize trajectory generator
	_trajectory_generator.reset({0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f});

	PX4_INFO("Boom motion controller initialized");
	return true;
}

void BoomMotionController::set_mode(ControlMode mode)
{
	_mode = mode;
	reset(); // Reset controller state on mode change
}

void BoomMotionController::set_target_position(float angle, float velocity_limit)
{
	_target_position = angle;
	_velocity_limit_override = velocity_limit;

	// Update trajectory generator target
	Vector3f target_pos(angle, 0.0f, 0.0f);
	Vector3f current_vel(0.0f, 0.0f, 0.0f);
	Vector3f current_accel(0.0f, 0.0f, 0.0f);
	_trajectory_generator.reset(current_accel, current_vel, target_pos);
}

void BoomMotionController::set_target_velocity(float velocity)
{
	_target_velocity = velocity;
}

BoomMotionController::MotionSetpoint
BoomMotionController::update_trajectory(float current_position, float dt)
{
	MotionSetpoint setpoint{};

	if (_emergency_stop_active) {
		// Emergency stop - zero everything
		setpoint.position = current_position;
		setpoint.velocity = 0.0f;
		setpoint.acceleration = 0.0f;
		setpoint.feedforward = 0.0f;
		return setpoint;
	}

	switch (_mode) {
	case ControlMode::POSITION: {
			// Update trajectory generator
			Vector3f current_pos(current_position, 0.0f, 0.0f);
			Vector3f current_vel(_current_traj_vel(0), 0.0f, 0.0f);
			Vector3f current_acc(_current_traj_acc(0), 0.0f, 0.0f);

			// Set motion constraints
			Vector3f max_vel(_param_max_velocity.get(), 0.0f, 0.0f);
			Vector3f max_acc(_param_max_acceleration.get(), 0.0f, 0.0f);
			Vector3f max_jerk(_param_max_jerk.get(), 0.0f, 0.0f);

			if (_velocity_limit_override > 0.0f) {
				max_vel(0) = _velocity_limit_override;
			}

			_trajectory_generator.setMaxVelocity(max_vel);
			_trajectory_generator.setMaxAcceleration(max_acc);
			_trajectory_generator.setMaxJerk(max_jerk(0)); // Use first component only

			// Update trajectory using PositionSmoothing
			current_pos = Vector3f(current_position, 0.0f, 0.0f);
			Vector3f waypoints[3] = {
				current_pos,                                    // past waypoint
				Vector3f(_target_position, 0.0f, 0.0f),       // target
				Vector3f(_target_position, 0.0f, 0.0f)        // next target (same as current for simplicity)
			};
			Vector3f ff_velocity(0.0f, 0.0f, 0.0f);
			PositionSmoothing::PositionSmoothingSetpoints setpoints;

			_trajectory_generator.generateSetpoints(current_pos, waypoints, ff_velocity, dt, false, setpoints);

			// Extract trajectory output (use only x-axis since boom is 1D)
			_current_traj_pos = Vector3f(setpoints.position(0), 0.0f, 0.0f);
			_current_traj_vel = Vector3f(setpoints.velocity(0), 0.0f, 0.0f);
			_current_traj_acc = Vector3f(setpoints.acceleration(0), 0.0f, 0.0f);

			setpoint.position = _current_traj_pos(0);
			setpoint.velocity = _current_traj_vel(0);
			setpoint.acceleration = _current_traj_acc(0);
			setpoint.feedforward = calculate_feedforward(setpoint);
			break;
		}

	case ControlMode::VELOCITY:
		setpoint.position = current_position; // Don't care about position
		setpoint.velocity = _target_velocity;
		setpoint.acceleration = 0.0f;
		setpoint.feedforward = 0.0f;
		break;

	case ControlMode::MANUAL:
		// Manual mode handled elsewhere
		setpoint.position = current_position;
		setpoint.velocity = 0.0f;
		setpoint.acceleration = 0.0f;
		setpoint.feedforward = 0.0f;
		break;

	default:
		setpoint.position = current_position;
		setpoint.velocity = 0.0f;
		setpoint.acceleration = 0.0f;
		setpoint.feedforward = 0.0f;
		break;
	}

	return setpoint;
}

BoomMotionController::ControlOutput BoomMotionController::compute_control(
	const MotionSetpoint &setpoint,
	float current_position,
	float current_velocity,
	float dt)
{
	ControlOutput output{};

	if (_emergency_stop_active) {
		output.duty_cycle = 0.0f;
		output.at_target = false;
		output.limited = true;
		output.velocity_limit = 0.0f;
		return output;
	}

	float control_output = 0.0f;

	switch (_mode) {
	case ControlMode::POSITION: {
			// Position control with velocity feedforward
			float position_error = setpoint.position - current_position;

			// Position controller output becomes velocity setpoint
			_position_controller.setSetpoint(0.0f); // Set target error to 0
			float velocity_setpoint = _position_controller.update(current_position - setpoint.position, dt);
			velocity_setpoint += setpoint.velocity; // Add feedforward

			// Velocity controller
			_velocity_controller.setSetpoint(velocity_setpoint);
			control_output = _velocity_controller.update(current_velocity, dt);			// Add feedforward and load compensation
			control_output += setpoint.feedforward + _load_compensation;

			// Check if at target
			static constexpr float POSITION_TOLERANCE = 0.01f; // ~0.5 degrees
			static constexpr float VELOCITY_TOLERANCE = 0.001f;
			output.at_target = (fabsf(position_error) < POSITION_TOLERANCE &&
					    fabsf(current_velocity) < VELOCITY_TOLERANCE);
			break;
		}

	case ControlMode::VELOCITY: {
			_velocity_controller.setSetpoint(setpoint.velocity);
			control_output = _velocity_controller.update(current_velocity, dt);
			control_output += _load_compensation;
			float velocity_error = setpoint.velocity - current_velocity;
			output.at_target = (fabsf(velocity_error) < 0.01f);
			break;
		}

	case ControlMode::MANUAL: {
			// Manual mode - pass through (would be handled by external command)
			control_output = 0.0f;
			output.at_target = true;
			break;
		}

	default:
		control_output = 0.0f;
		output.at_target = true;
		break;
	}

	// Apply deadzone compensation
	control_output = apply_deadzone_compensation(control_output);

	// Apply limits
	control_output = apply_limits(control_output);

	output.duty_cycle = control_output;
	output.limited = (fabsf(control_output) >= 0.95f);
	output.velocity_limit = _param_max_velocity.get();

	_last_output = control_output;
	return output;
}

void BoomMotionController::set_load_compensation(float load_estimate)
{
	_load_compensation = load_estimate * _param_load_comp_gain.get();
}

void BoomMotionController::reset()
{
	_position_controller.resetIntegral();
	_velocity_controller.resetIntegral();
	_last_output = 0.0f;
}

void BoomMotionController::emergency_stop()
{
	_emergency_stop_active = true;
	reset();
}

float BoomMotionController::apply_deadzone_compensation(float output) const
{
	float deadzone = _param_deadzone.get();

	if (fabsf(output) < deadzone) {
		return 0.0f;
	}

	// Compensate for deadzone by shifting output
	if (output > 0.0f) {
		return output + deadzone;

	} else {
		return output - deadzone;
	}
}

float BoomMotionController::apply_limits(float output) const
{
	return math::constrain(output, -1.0f, 1.0f);
}

float BoomMotionController::calculate_feedforward(const MotionSetpoint &setpoint) const
{
	// Simple feedforward based on acceleration
	return setpoint.acceleration * _param_feedforward_gain.get();
}

void BoomMotionController::update_parameters()
{
	// Update all parameters from the parameter system
	updateParams();

	// Update PID controller parameters
	_position_controller.setGains(_param_pos_p.get(), _param_pos_i.get(), _param_pos_d.get());

	_velocity_controller.setGains(_param_vel_p.get(), _param_vel_i.get(), _param_vel_d.get());

	// Update trajectory planner limits
	Vector3f max_vel(_param_max_velocity.get(), 0.0f, 0.0f);
	Vector3f max_acc(_param_max_acceleration.get(), 0.0f, 0.0f);

	_trajectory_generator.setMaxVelocity(max_vel);
	_trajectory_generator.setMaxAcceleration(max_acc);
	_trajectory_generator.setMaxJerk(_param_max_jerk.get());

	PX4_DEBUG("Motion controller parameters updated: pos_p=%.3f, vel_p=%.3f, max_vel=%.2f",
		  (double)_param_pos_p.get(),
		  (double)_param_vel_p.get(),
		  (double)_param_max_velocity.get());
}
