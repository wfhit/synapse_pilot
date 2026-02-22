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

#include "manual_direct_mode.hpp"
#include <px4_platform_common/log.h>

ManualDirectMode::ManualDirectMode(ModuleParams *parent) :
	OperationModeBase(parent, "WheelLoaderManualDirect")
{
	// Load vehicle parameters
	loadVehicleParameters();
}

bool ManualDirectMode::activate()
{
	PX4_INFO("Activating Wheel Loader Manual Direct Mode");

	// Check if required data is available
	if (!_vehicle_local_position_sub.copy(&_vehicle_local_position)) {
		PX4_WARN("Vehicle local position not available");
		return false;
	}

	if (!_vehicle_attitude_sub.copy(&_vehicle_attitude)) {
		PX4_WARN("Vehicle attitude not available");
		return false;
	}

	if (!_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {
		PX4_WARN("Manual control setpoint not available");
		return false;
	}

	// Update current state
	updateCurrentState();

	// Initialize target states from current state
	initializeTargetStates();

	set_active(true);

	// Publish initial control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = false;
	config.flag_control_rates_enabled = true;
	publish_control_config(config);

	PX4_INFO("Manual direct mode activated - boom:%.2f art:%.2f tilt:%.2f",
		 (double)_target_boom_position, (double)_target_articulation_angle, (double)_target_tilt_angle);

	return true;
}

void ManualDirectMode::deactivate()
{
	PX4_INFO("Deactivating Wheel Loader Manual Direct Mode");
	set_active(false);
}

void ManualDirectMode::update(float dt)
{
	// Update sensor data
	updateCurrentState();

	// Process RC inputs and compute velocities/rates
	processRCInputs(dt);

	// Integrate rates to update target states
	updateTargetStates(dt);

	// Apply limits
	applyLimits();

	// Publish control setpoints
	publishSetpoints();

	// Publish control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = false;
	config.flag_control_rates_enabled = true;
	publish_control_config(config);
}

bool ManualDirectMode::is_valid() const
{
	// Activation validity is checked in activate() via subscription copies.
	// Returning true here allows the manager to attempt activation.
	return true;
}

void ManualDirectMode::processRCInputs(float dt)
{
	// Update RC data
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);

	// RC Channel mapping (normalized -1 to +1):
	// x (roll):     Ch 1 - Chassis forward/backward velocity
	// y (pitch):    Ch 2 - Boom height velocity
	// z (throttle): Ch 3 - Articulation angle rate
	// r (yaw):      Ch 4 - Bucket tilt rate

	float rc_x = applyDeadband(_manual_control_setpoint.roll, _control_params.rc_deadband);
	float rc_y = applyDeadband(_manual_control_setpoint.pitch, _control_params.rc_deadband);
	float rc_z = applyDeadband(_manual_control_setpoint.throttle, _control_params.rc_deadband);
	float rc_r = applyDeadband(_manual_control_setpoint.yaw, _control_params.rc_deadband);

	// Scale to velocities/rates
	_chassis_velocity = rc_x * _control_params.max_chassis_velocity;
	_boom_velocity = rc_y * _control_params.max_boom_velocity;
	_articulation_rate = rc_z * _control_params.max_articulation_rate;
	_tilt_rate = rc_r * _control_params.max_tilt_rate;

	PX4_DEBUG("RC: x=%.2f y=%.2f z=%.2f r=%.2f -> chas_v=%.2f boom_v=%.2f art_r=%.2f tilt_r=%.2f",
		  (double)rc_x, (double)rc_y, (double)rc_z, (double)rc_r,
		  (double)_chassis_velocity, (double)_boom_velocity,
		  (double)_articulation_rate, (double)_tilt_rate);
}

float ManualDirectMode::applyDeadband(float input, float deadband)
{
	if (fabsf(input) < deadband) {
		return 0.0f;
	}

	// Scale from [deadband, 1.0] to [0, 1.0]
	float sign = (input > 0.0f) ? 1.0f : -1.0f;
	return sign * (fabsf(input) - deadband) / (1.0f - deadband);
}

void ManualDirectMode::updateTargetStates(float dt)
{
	// Integrate rates to get target states
	_target_boom_position += _boom_velocity * dt;
	_target_articulation_angle += _articulation_rate * dt;
	_target_tilt_angle += _tilt_rate * dt;
}

void ManualDirectMode::applyLimits()
{
	// Clamp boom position
	_target_boom_position = math::constrain(_target_boom_position,
						_vehicle_params.boom_min,
						_vehicle_params.boom_max);

	// Clamp articulation angle
	_target_articulation_angle = math::constrain(_target_articulation_angle,
				     _vehicle_params.articulation_min,
				     _vehicle_params.articulation_max);

	// Clamp tilt angle
	_target_tilt_angle = math::constrain(_target_tilt_angle,
					     _vehicle_params.tilt_min,
					     _vehicle_params.tilt_max);
}

void ManualDirectMode::publishSetpoints()
{
	hrt_abstime now = hrt_absolute_time();

	// Publish traction setpoint - direct velocity control
	traction_setpoint_s traction_sp{};
	traction_sp.timestamp = now;
	traction_sp.desired_velocity_ms = _chassis_velocity;
	traction_sp.desired_steering_angle_rad = _target_articulation_angle;
	traction_sp.desired_yaw_rate_rad_s = _articulation_rate;
	traction_sp.enable_traction_control = false;
	traction_sp.enable_slip_control = false;
	traction_sp.enable_stability_control = false;
	traction_sp.traction_mode = traction_setpoint_s::MODE_NORMAL;
	traction_sp.max_velocity_ms = _control_params.max_chassis_velocity;
	_traction_setpoint_pub.publish(traction_sp);

	// Publish boom control setpoint
	boom_control_setpoint_s boom_sp{};
	boom_sp.timestamp = now;
	boom_sp.bucket_height = _target_boom_position;
	boom_sp.bucket_height_velocity = _boom_velocity;
	boom_sp.valid = true;
	_boom_control_setpoint_pub.publish(boom_sp);

	// Publish tilt control setpoint
	tilt_control_setpoint_s tilt_sp{};
	tilt_sp.timestamp = now;
	tilt_sp.angle = _target_tilt_angle;
	tilt_sp.angular_velocity = _tilt_rate;
	tilt_sp.valid = true;
	_tilt_control_setpoint_pub.publish(tilt_sp);

	PX4_DEBUG("Setpoints: chas_v=%.2f boom=%.2f(v=%.2f) art=%.2f(r=%.2f) tilt=%.2f(r=%.2f)",
		  (double)_chassis_velocity,
		  (double)_target_boom_position, (double)_boom_velocity,
		  (double)_target_articulation_angle, (double)_articulation_rate,
		  (double)_target_tilt_angle, (double)_tilt_rate);
}

void ManualDirectMode::updateCurrentState()
{
	// Update subscriptions
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_attitude_sub.update(&_vehicle_attitude);

	// TODO: Get actual boom, articulation, and tilt positions from sensors
	// For now, use last commanded values
	_current_boom_position = _target_boom_position;
	_current_articulation_angle = _target_articulation_angle;
	_current_tilt_angle = _target_tilt_angle;
}

void ManualDirectMode::loadVehicleParameters()
{
	// TODO: Load from YAML or PX4 parameters
	// Using defaults for now

	_vehicle_params.boom_min = 0.5f;
	_vehicle_params.boom_max = 3.0f;
	_vehicle_params.tilt_min = -1.57f;
	_vehicle_params.tilt_max = 1.57f;
	_vehicle_params.articulation_min = -0.785f;  // -45 deg
	_vehicle_params.articulation_max = 0.785f;   // +45 deg

	_control_params.rc_deadband = 0.05f;
	_control_params.max_chassis_velocity = 1.0f;
	_control_params.max_boom_velocity = 0.5f;
	_control_params.max_articulation_rate = 0.3f;
	_control_params.max_tilt_rate = 0.5f;
}

void ManualDirectMode::initializeTargetStates()
{
	// Initialize from current state (or defaults if sensors not available)
	// TODO: Get actual positions from sensors

	// Default initial states
	_target_boom_position = 1.5f;  // Mid-range
	_target_articulation_angle = 0.0f;  // Straight
	_target_tilt_angle = 0.0f;  // Level

	_current_boom_position = _target_boom_position;
	_current_articulation_angle = _target_articulation_angle;
	_current_tilt_angle = _target_tilt_angle;

	PX4_INFO("Initialized states: boom=%.2f art=%.2f tilt=%.2f",
		 (double)_target_boom_position, (double)_target_articulation_angle, (double)_target_tilt_angle);
}
