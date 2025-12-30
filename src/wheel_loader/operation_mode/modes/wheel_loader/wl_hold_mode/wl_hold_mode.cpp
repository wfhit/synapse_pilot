/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "wheel_loader_hold_mode.hpp"
#include <px4_platform_common/log.h>

WheelLoaderHoldMode::WheelLoaderHoldMode(ModuleParams *parent) :
	OperationModeBase(parent, "WheelLoaderHold")
{
	loadParameters();
}

bool WheelLoaderHoldMode::activate()
{
	PX4_INFO("Activating Wheel Loader Hold Mode");

	// Check if required sensor data is available
	if (!_vehicle_local_position_sub.copy(&_vehicle_local_position)) {
		PX4_ERR("Hold mode activation failed: Vehicle local position not available");
		return false;
	}

	if (!_vehicle_attitude_sub.copy(&_vehicle_attitude)) {
		PX4_ERR("Hold mode activation failed: Vehicle attitude not available");
		return false;
	}

	// Verify position data is valid and recent
	bool position_valid = _vehicle_local_position.xy_valid && _vehicle_local_position.z_valid &&
			      (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 500_ms);

	bool attitude_valid = (hrt_elapsed_time(&_vehicle_attitude.timestamp) < 500_ms);

	if (!position_valid || !attitude_valid) {
		PX4_ERR("Hold mode activation failed: Sensor data invalid or stale");
		return false;
	}

	// Capture current state
	if (!captureCurrentState()) {
		PX4_ERR("Hold mode activation failed: Could not capture current state");
		return false;
	}

	set_active(true);

	// Publish initial control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = true;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = true;
	config.flag_control_rates_enabled = false;
	publish_control_config(config);

	PX4_INFO("Hold mode activated - holding at chassis(%.2f, %.2f, %.2f rad) boom=%.2f tilt=%.2f art=%.2f",
		 (double)_hold_state.chassis_x, (double)_hold_state.chassis_y,
		 (double)_hold_state.chassis_heading, (double)_hold_state.boom_position,
		 (double)_hold_state.tilt_angle, (double)_hold_state.articulation_angle);

	return true;
}

void WheelLoaderHoldMode::deactivate()
{
	PX4_INFO("Deactivating Wheel Loader Hold Mode");
	set_active(false);
}

void WheelLoaderHoldMode::update(float dt)
{
	// Update current state from sensors
	updateCurrentState();

	// Compute and publish control setpoints to maintain hold position
	computeAndPublishSetpoints();

	// Publish control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = true;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = true;
	config.flag_control_rates_enabled = false;
	publish_control_config(config);
}

bool WheelLoaderHoldMode::is_valid() const
{
	bool position_valid = _vehicle_local_position.xy_valid && _vehicle_local_position.z_valid &&
			      (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 500_ms);

	bool attitude_valid = (hrt_elapsed_time(&_vehicle_attitude.timestamp) < 500_ms);

	return position_valid && attitude_valid && _hold_state.valid;
}

bool WheelLoaderHoldMode::captureCurrentState()
{
	// Update subscriptions
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_attitude_sub.update(&_vehicle_attitude);

	// Capture chassis position
	_hold_state.chassis_x = _vehicle_local_position.x;
	_hold_state.chassis_y = _vehicle_local_position.y;

	// Get heading from attitude
	matrix::Quatf q(_vehicle_attitude.q);
	matrix::Eulerf euler(q);
	_hold_state.chassis_heading = euler.psi();

	// TODO: Get actual boom, tilt, and articulation positions from sensors
	// For now, use default safe values
	_hold_state.boom_position = 1.5f;       // Mid-range boom position
	_hold_state.tilt_angle = 0.0f;          // Level tilt
	_hold_state.articulation_angle = 0.0f;  // Straight articulation

	_hold_state.valid = true;

	PX4_INFO("Captured hold state: chassis(%.2f, %.2f, %.2f) boom=%.2f tilt=%.2f art=%.2f",
		 (double)_hold_state.chassis_x, (double)_hold_state.chassis_y,
		 (double)_hold_state.chassis_heading, (double)_hold_state.boom_position,
		 (double)_hold_state.tilt_angle, (double)_hold_state.articulation_angle);

	return true;
}

void WheelLoaderHoldMode::updateCurrentState()
{
	// Update subscriptions
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_attitude_sub.update(&_vehicle_attitude);

	// Update chassis state
	_current_state.chassis_x = _vehicle_local_position.x;
	_current_state.chassis_y = _vehicle_local_position.y;

	// Get heading from attitude
	matrix::Quatf q(_vehicle_attitude.q);
	matrix::Eulerf euler(q);
	_current_state.chassis_heading = euler.psi();

	// TODO: Get actual boom, tilt, and articulation positions from sensors
	_current_state.boom_position = _hold_state.boom_position;
	_current_state.tilt_angle = _hold_state.tilt_angle;
	_current_state.articulation_angle = _hold_state.articulation_angle;

	_current_state.valid = true;
}

void WheelLoaderHoldMode::computeAndPublishSetpoints()
{
	hrt_abstime now = hrt_absolute_time();

	// Compute position errors
	float chassis_error_x = _hold_state.chassis_x - _current_state.chassis_x;
	float chassis_error_y = _hold_state.chassis_y - _current_state.chassis_y;
	float heading_error = matrix::wrap_pi(_hold_state.chassis_heading - _current_state.chassis_heading);
	float articulation_error = matrix::wrap_pi(_hold_state.articulation_angle - _current_state.articulation_angle);
	float boom_error = _hold_state.boom_position - _current_state.boom_position;
	float tilt_error = matrix::wrap_pi(_hold_state.tilt_angle - _current_state.tilt_angle);

	// Apply deadbands to prevent chatter
	if (fabsf(chassis_error_x) < _control_params.chassis_position_tol) { chassis_error_x = 0.0f; }

	if (fabsf(chassis_error_y) < _control_params.chassis_position_tol) { chassis_error_y = 0.0f; }

	if (fabsf(heading_error) < _control_params.chassis_heading_tol) { heading_error = 0.0f; }

	if (fabsf(articulation_error) < _control_params.articulation_tol) { articulation_error = 0.0f; }

	if (fabsf(boom_error) < _control_params.boom_position_tol) { boom_error = 0.0f; }

	if (fabsf(tilt_error) < _control_params.tilt_angle_tol) { tilt_error = 0.0f; }

	// Compute control velocities (proportional control)
	float chassis_vel_x = _control_params.chassis_position_gain * chassis_error_x;
	float chassis_vel_y = _control_params.chassis_position_gain * chassis_error_y;
	float chassis_yaw_rate = _control_params.chassis_heading_gain * heading_error;
	float articulation_rate = _control_params.articulation_gain * articulation_error;
	float boom_velocity = _control_params.boom_position_gain * boom_error;
	float tilt_rate = _control_params.tilt_angle_gain * tilt_error;

	// Apply velocity limits
	chassis_vel_x = math::constrain(chassis_vel_x, -_control_params.max_chassis_velocity,
					_control_params.max_chassis_velocity);
	chassis_vel_y = math::constrain(chassis_vel_y, -_control_params.max_chassis_velocity,
					_control_params.max_chassis_velocity);
	chassis_yaw_rate = math::constrain(chassis_yaw_rate, -_control_params.max_chassis_yaw_rate,
					   _control_params.max_chassis_yaw_rate);
	articulation_rate = math::constrain(articulation_rate, -_control_params.max_articulation_rate,
					    _control_params.max_articulation_rate);
	boom_velocity = math::constrain(boom_velocity, -_control_params.max_boom_velocity,
					_control_params.max_boom_velocity);
	tilt_rate = math::constrain(tilt_rate, -_control_params.max_tilt_rate, _control_params.max_tilt_rate);

	// Publish chassis setpoint
	wheel_loader_chassis_setpoint_s chassis_sp{};
	chassis_sp.timestamp = now;
	chassis_sp.velocity_x = chassis_vel_x;
	chassis_sp.velocity_y = chassis_vel_y;
	chassis_sp.yaw_rate = chassis_yaw_rate;
	chassis_sp.articulation_rate = articulation_rate;
	chassis_sp.position_valid = true;
	chassis_sp.velocity_valid = true;
	chassis_sp.articulation_valid = true;
	_chassis_setpoint_pub.publish(chassis_sp);

	// Publish boom setpoint
	wheel_loader_boom_setpoint_s boom_sp{};
	boom_sp.timestamp = now;
	boom_sp.position = _hold_state.boom_position;
	boom_sp.velocity = boom_velocity;
	boom_sp.acceleration = 0.f;
	boom_sp.control_mode = wheel_loader_boom_setpoint_s::MODE_POSITION;
	boom_sp.max_velocity = _control_params.max_boom_velocity;
	boom_sp.max_acceleration = 0.5f;
	boom_sp.setpoint_valid = true;
	boom_sp.trajectory_complete = false;
	_boom_setpoint_pub.publish(boom_sp);

	// Publish tilt setpoint
	wheel_loader_tilt_setpoint_s tilt_sp{};
	tilt_sp.timestamp = now;
	tilt_sp.angle = _hold_state.tilt_angle;
	tilt_sp.angular_velocity = tilt_rate;
	tilt_sp.angular_acceleration = 0.f;
	tilt_sp.control_mode = wheel_loader_tilt_setpoint_s::MODE_POSITION;
	tilt_sp.max_velocity = _control_params.max_tilt_rate;
	tilt_sp.max_acceleration = 1.0f;
	tilt_sp.setpoint_valid = true;
	tilt_sp.trajectory_complete = false;
	_tilt_setpoint_pub.publish(tilt_sp);

	PX4_DEBUG("Hold errors: chassis(%.3f,%.3f,%.3f) boom=%.3f tilt=%.3f art=%.3f",
		  (double)chassis_error_x, (double)chassis_error_y, (double)heading_error,
		  (double)boom_error, (double)tilt_error, (double)articulation_error);
}

void WheelLoaderHoldMode::loadParameters()
{
	// TODO: Load from YAML or PX4 parameters
	// Using defaults for now

	_control_params.chassis_position_gain = 1.0f;
	_control_params.chassis_heading_gain = 1.0f;
	_control_params.articulation_gain = 1.0f;
	_control_params.boom_position_gain = 2.0f;
	_control_params.tilt_angle_gain = 2.0f;

	_control_params.max_chassis_velocity = 0.3f;
	_control_params.max_chassis_yaw_rate = 0.3f;
	_control_params.max_articulation_rate = 0.2f;
	_control_params.max_boom_velocity = 0.3f;
	_control_params.max_tilt_rate = 0.3f;

	_control_params.chassis_position_tol = 0.02f;
	_control_params.chassis_heading_tol = 0.05f;
	_control_params.articulation_tol = 0.05f;
	_control_params.boom_position_tol = 0.02f;
	_control_params.tilt_angle_tol = 0.05f;
}
