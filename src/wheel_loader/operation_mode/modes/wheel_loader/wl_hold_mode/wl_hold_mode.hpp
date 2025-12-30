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

/**
 * @file wheel_loader_hold_mode.hpp
 *
 * Wheel loader hold mode - maintains current position of all actuators.
 *
 * This mode captures the current state when activated and actively maintains
 * position control on all actuators (chassis, boom, tilt, articulation).
 * - Normal operational pause mode (not emergency)
 * - Requires sensor feedback to activate
 * - Actively controls position against disturbances
 * - Uses deadbands to prevent control chatter
 * - Requires explicit mode switch to exit
 * - Runs at 20Hz
 *
 * @author PX4 Development Team
 */

#pragma once

#include "../../operation_mode_base.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/wheel_loader/chassis_setpoint.h>
#include <uORB/topics/wheel_loader/boom_setpoint.h>
#include <uORB/topics/wheel_loader/tilt_setpoint.h>

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

class WheelLoaderHoldMode : public OperationModeBase
{
public:
	WheelLoaderHoldMode(ModuleParams *parent);
	~WheelLoaderHoldMode() override = default;

	bool activate() override;
	void deactivate() override;
	void update(float dt) override;
	bool is_valid() const override;

private:
	// Vehicle state
	struct HoldState {
		// Chassis state
		float chassis_x{0.f};
		float chassis_y{0.f};
		float chassis_heading{0.f};
		float articulation_angle{0.f};

		// Boom and tilt state
		float boom_position{0.f};
		float tilt_angle{0.f};

		bool valid{false};
	};

	// Control parameters
	struct ControlParams {
		// Position control gains
		float chassis_position_gain{1.0f};
		float chassis_heading_gain{1.0f};
		float articulation_gain{1.0f};
		float boom_position_gain{2.0f};
		float tilt_angle_gain{2.0f};

		// Velocity limits for position control
		float max_chassis_velocity{0.3f};
		float max_chassis_yaw_rate{0.3f};
		float max_articulation_rate{0.2f};
		float max_boom_velocity{0.3f};
		float max_tilt_rate{0.3f};

		// Position tolerance deadbands
		float chassis_position_tol{0.02f};    // 2cm
		float chassis_heading_tol{0.05f};     // ~3 degrees
		float articulation_tol{0.05f};        // ~3 degrees
		float boom_position_tol{0.02f};       // 2cm
		float tilt_angle_tol{0.05f};          // ~3 degrees
	};

	// Subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	// Publications
	uORB::Publication<wheel_loader_chassis_setpoint_s> _chassis_setpoint_pub{ORB_ID(wheel_loader_chassis_setpoint)};
	uORB::Publication<wheel_loader_boom_setpoint_s> _boom_setpoint_pub{ORB_ID(wheel_loader_boom_setpoint)};
	uORB::Publication<wheel_loader_tilt_setpoint_s> _tilt_setpoint_pub{ORB_ID(wheel_loader_tilt_setpoint)};

	// State
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_attitude_s _vehicle_attitude{};

	HoldState _hold_state{};
	HoldState _current_state{};
	ControlParams _control_params{};

	// Methods
	bool captureCurrentState();
	void updateCurrentState();
	void computeAndPublishSetpoints();
	void loadParameters();

	static constexpr float UPDATE_RATE = 20.0f;  // 20 Hz
	static constexpr float UPDATE_DT = 1.0f / UPDATE_RATE;
};
