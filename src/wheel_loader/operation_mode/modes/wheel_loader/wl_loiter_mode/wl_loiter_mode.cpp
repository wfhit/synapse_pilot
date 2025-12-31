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

#include "wl_loiter_mode.hpp"
#include <px4_platform_common/log.h>

WheelLoaderLoiterMode::WheelLoaderLoiterMode(ModuleParams *parent) :
	OperationModeBase(parent, "WheelLoaderLoiter")
{
	loadParameters();
}

bool WheelLoaderLoiterMode::activate()
{
	PX4_INFO("Activating Wheel Loader Loiter Mode");

	// Check if required sensor data is available for safety
	if (!_vehicle_local_position_sub.copy(&_vehicle_local_position)) {
		PX4_WARN("Loiter mode: Vehicle local position not available");
		// Continue activation - loiter doesn't strictly need position feedback
	}

	if (!_vehicle_attitude_sub.copy(&_vehicle_attitude)) {
		PX4_WARN("Loiter mode: Vehicle attitude not available");
		// Continue activation - loiter doesn't strictly need attitude feedback
	}

	set_active(true);

	// Publish initial control config - velocity control enabled
	operation_control_config_s config{};
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = false;
	config.flag_control_rates_enabled = true;
	publish_control_config(config);

	PX4_INFO("Loiter mode activated - commanding zero velocities (passive hold)");

	return true;
}

void WheelLoaderLoiterMode::deactivate()
{
	PX4_INFO("Deactivating Wheel Loader Loiter Mode");
	set_active(false);
}

void WheelLoaderLoiterMode::update(float dt)
{
	// Update sensor data (for validity checking only)
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_attitude_sub.update(&_vehicle_attitude);

	// Publish zero velocity setpoints to all actuators
	publishZeroSetpoints();

	// Publish control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = false;
	config.flag_control_rates_enabled = true;
	publish_control_config(config);
}

bool WheelLoaderLoiterMode::is_valid() const
{
	// Loiter mode is always valid once activated
	// It doesn't require active sensor feedback
	return true;
}

void WheelLoaderLoiterMode::publishZeroSetpoints()
{
	hrt_abstime now = hrt_absolute_time();

	// Publish chassis zero velocity setpoint
	if (_loiter_params.enable_chassis) {
		chassis_setpoint_s chassis_sp{};
		chassis_sp.timestamp = now;
		chassis_sp.velocity_x = 0.f;
		chassis_sp.velocity_y = 0.f;
		chassis_sp.yaw_rate = 0.f;
		chassis_sp.articulation_rate = 0.f;
		chassis_sp.position_valid = false;
		chassis_sp.velocity_valid = true;
		chassis_sp.articulation_valid = _loiter_params.enable_articulation;
		_chassis_setpoint_pub.publish(chassis_sp);
	}

	// Publish boom zero velocity setpoint
	if (_loiter_params.enable_boom) {
		boom_setpoint_s boom_sp{};
		boom_sp.timestamp = now;
		boom_sp.position = 0.f;
		boom_sp.velocity = 0.f;
		boom_sp.acceleration = 0.f;
		boom_sp.control_mode = boom_setpoint_s::MODE_VELOCITY;
		boom_sp.max_velocity = 0.f;
		boom_sp.max_acceleration = 0.f;
		boom_sp.setpoint_valid = true;
		boom_sp.trajectory_complete = false;
		_boom_setpoint_pub.publish(boom_sp);
	}

	// Publish tilt zero rate setpoint
	if (_loiter_params.enable_tilt) {
		tilt_setpoint_s tilt_sp{};
		tilt_sp.timestamp = now;
		tilt_sp.angle = 0.f;
		tilt_sp.angular_velocity = 0.f;
		tilt_sp.angular_acceleration = 0.f;
		tilt_sp.control_mode = tilt_setpoint_s::MODE_VELOCITY;
		tilt_sp.max_velocity = 0.f;
		tilt_sp.max_acceleration = 0.f;
		tilt_sp.setpoint_valid = true;
		tilt_sp.trajectory_complete = false;
		_tilt_setpoint_pub.publish(tilt_sp);
	}

	PX4_DEBUG("Loiter: publishing zero velocities to all actuators");
}

void WheelLoaderLoiterMode::loadParameters()
{
	// TODO: Load from YAML or PX4 parameters
	// Using defaults for now

	_loiter_params.enable_chassis = true;
	_loiter_params.enable_boom = true;
	_loiter_params.enable_tilt = true;
	_loiter_params.enable_articulation = true;
}
