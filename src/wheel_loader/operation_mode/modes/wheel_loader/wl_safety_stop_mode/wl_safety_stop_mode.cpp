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

#include "wheel_loader_safety_stop_mode.hpp"
#include <px4_platform_common/log.h>

WheelLoaderSafetyStopMode::WheelLoaderSafetyStopMode(ModuleParams *parent) :
	OperationModeBase(parent, "WheelLoaderSafetyStop")
{
}

bool WheelLoaderSafetyStopMode::activate()
{
	PX4_WARN("Activating Wheel Loader Safety Stop Mode - IMMEDIATE STOP");

	// Safety stop mode does not require sensor feedback
	// It can activate even during sensor failures for safety

	set_active(true);

	// Immediately publish stop setpoints
	publishStopSetpoints();

	// Publish control config - velocity control enabled
	operation_control_config_s config{};
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = false;
	config.flag_control_rates_enabled = false;
	publish_control_config(config);

	PX4_WARN("Safety stop activated - all actuators commanded to zero velocity");

	return true;
}

void WheelLoaderSafetyStopMode::deactivate()
{
	PX4_INFO("Deactivating Wheel Loader Safety Stop Mode");
	set_active(false);
}

void WheelLoaderSafetyStopMode::update(float dt)
{
	// Continuously publish stop setpoints
	publishStopSetpoints();

	// Publish control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = false;
	config.flag_control_rates_enabled = false;
	publish_control_config(config);
}

bool WheelLoaderSafetyStopMode::is_valid() const
{
	// Safety stop mode is always valid
	// It does not depend on any sensor feedback
	return true;
}

void WheelLoaderSafetyStopMode::publishStopSetpoints()
{
	hrt_abstime now = hrt_absolute_time();

	// Publish chassis zero velocity setpoint
	chassis_setpoint_s chassis_sp{};
	chassis_sp.timestamp = now;
	chassis_sp.velocity_x = 0.f;
	chassis_sp.velocity_y = 0.f;
	chassis_sp.yaw_rate = 0.f;
	chassis_sp.articulation_rate = 0.f;
	chassis_sp.position_valid = false;
	chassis_sp.velocity_valid = true;
	chassis_sp.articulation_valid = true;
	_chassis_setpoint_pub.publish(chassis_sp);

	// Publish boom zero velocity setpoint
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

	// Publish tilt zero rate setpoint
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

	PX4_DEBUG("Safety stop: publishing zero velocities to all actuators");
}
