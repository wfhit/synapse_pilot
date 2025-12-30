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
 * @file wheel_loader_safety_stop_mode.hpp
 *
 * Wheel loader safety stop mode - immediate stop of all actuators.
 *
 * This mode provides an immediate hard stop by commanding zero velocities
 * to all actuators. It does not require sensor feedback and can activate
 * even during sensor failures.
 * - Immediate stop: zero velocities to all actuators
 * - No sensor requirements: can activate blindly
 * - Normal mode priority: activated via mode command
 * - Exit handling: managed by mode manager
 * - Runs at 20Hz
 *
 * Use cases:
 * - Emergency stop command from operator
 * - Safe state during system transitions
 * - Default stop mode before switching operations
 *
 * @author PX4 Development Team
 */

#pragma once

#include "../../operation_mode_base.hpp"

#include <uORB/Publication.hpp>
#include <uORB/topics/chassis_setpoint.h>
#include <uORB/topics/boom_setpoint.h>
#include <uORB/topics/tilt_setpoint.h>

class WheelLoaderSafetyStopMode : public OperationModeBase
{
public:
	WheelLoaderSafetyStopMode(ModuleParams *parent);
	~WheelLoaderSafetyStopMode() override = default;

	bool activate() override;
	void deactivate() override;
	void update(float dt) override;
	bool is_valid() const override;

private:
	// Publications
	uORB::Publication<chassis_setpoint_s> _chassis_setpoint_pub{ORB_ID(chassis_setpoint)};
	uORB::Publication<boom_setpoint_s> _boom_setpoint_pub{ORB_ID(boom_setpoint)};
	uORB::Publication<tilt_setpoint_s> _tilt_setpoint_pub{ORB_ID(tilt_setpoint)};

	// Methods
	void publishStopSetpoints();

	static constexpr float UPDATE_RATE = 20.0f;  // 20 Hz
	static constexpr float UPDATE_DT = 1.0f / UPDATE_RATE;
};
