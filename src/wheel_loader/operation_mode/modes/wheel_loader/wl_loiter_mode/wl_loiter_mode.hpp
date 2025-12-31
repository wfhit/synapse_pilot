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
 * @file wheel_loader_loiter_mode.hpp
 *
 * Wheel loader loiter mode - passive hold with zero velocities.
 *
 * This mode provides a passive hold by commanding zero velocities/rates
 * to all actuators. It does not actively correct for position drift.
 * - Passive operational pause (no position correction)
 * - Simply commands zero velocities to all actuators
 * - Relies on lower-level controllers and mechanical friction
 * - Allows natural drift without correction
 * - Requires sensor validity for safe operation
 * - Runs at 20Hz
 *
 * Use cases:
 * - Temporary pause where drift is acceptable
 * - Situations where active control is not needed
 * - Lower energy consumption than active hold mode
 *
 * @author PX4 Development Team
 */

#pragma once

#include "../../../operation_mode_base.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/chassis_setpoint.h>
#include <uORB/topics/boom_setpoint.h>
#include <uORB/topics/tilt_setpoint.h>

class WheelLoaderLoiterMode : public OperationModeBase
{
public:
	WheelLoaderLoiterMode(ModuleParams *parent);
	~WheelLoaderLoiterMode() override = default;

	bool activate() override;
	void deactivate() override;
	void update(float dt) override;
	bool is_valid() const override;

private:
	// Control parameters
	struct LoiterParams {
		bool enable_chassis{true};
		bool enable_boom{true};
		bool enable_tilt{true};
		bool enable_articulation{true};
	};

	// Subscriptions
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	// Publications
	uORB::Publication<chassis_setpoint_s> _chassis_setpoint_pub{ORB_ID(chassis_setpoint)};
	uORB::Publication<boom_setpoint_s> _boom_setpoint_pub{ORB_ID(boom_setpoint)};
	uORB::Publication<tilt_setpoint_s> _tilt_setpoint_pub{ORB_ID(tilt_setpoint)};

	// State
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_attitude_s _vehicle_attitude{};

	LoiterParams _loiter_params{};

	// Methods
	void publishZeroSetpoints();
	void loadParameters();

	static constexpr float UPDATE_RATE = 20.0f;  // 20 Hz
	static constexpr float UPDATE_DT = 1.0f / UPDATE_RATE;
};
