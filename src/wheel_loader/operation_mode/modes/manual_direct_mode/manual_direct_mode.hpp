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
 * @file wheel_loader_manual_direct_mode.hpp
 *
 * Wheel Loader Manual Direct Control Mode
 * - RC channels directly control all actuators independently
 * - Ch 1: Chassis forward/backward velocity
 * - Ch 2: Boom height velocity
 * - Ch 3: Articulation angle rate
 * - Ch 4: Bucket tilt rate
 * - No inverse kinematics, pure velocity/rate control
 * - Runs at 50Hz
 *
 * @author PX4 Development Team
 */

#pragma once

#include "../../operation_mode_base.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/traction_setpoint.h>
#include <uORB/topics/boom_control_setpoint.h>
#include <uORB/topics/tilt_control_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

class ManualDirectMode : public OperationModeBase
{
public:
	ManualDirectMode(ModuleParams *parent);
	~ManualDirectMode() override = default;

	bool activate() override;
	void deactivate() override;
	void update(float dt) override;
	bool is_valid() const override;

private:
	/**
	 * Read RC inputs and convert to velocities/rates
	 */
	void processRCInputs(float dt);

	/**
	 * Apply deadband to RC input
	 */
	float applyDeadband(float input, float deadband);

	/**
	 * Integrate rates to update target states
	 */
	void updateTargetStates(float dt);

	/**
	 * Apply limits to target states
	 */
	void applyLimits();

	/**
	 * Publish control setpoints
	 */
	void publishSetpoints();

	/**
	 * Update current sensor state
	 */
	void updateCurrentState();

	/**
	 * Load vehicle parameters
	 */
	void loadVehicleParameters();

	/**
	 * Initialize target states from current state
	 */
	void initializeTargetStates();

	// Subscriptions
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	// Publications
	uORB::Publication<traction_setpoint_s> _traction_setpoint_pub{ORB_ID(traction_setpoint)};
	uORB::Publication<boom_control_setpoint_s> _boom_control_setpoint_pub{ORB_ID(boom_control_setpoint)};
	uORB::Publication<tilt_control_setpoint_s> _tilt_control_setpoint_pub{ORB_ID(tilt_control_setpoint)};

	// RC input data
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_attitude_s _vehicle_attitude{};

	// RC velocities/rates
	float _chassis_velocity{0.f};        // Forward/backward velocity [m/s]
	float _boom_velocity{0.f};           // Boom height velocity [m/s]
	float _articulation_rate{0.f};       // Articulation angle rate [rad/s]
	float _tilt_rate{0.f};              // Tilt angle rate [rad/s]

	// Target states (integrated from rates)
	float _target_boom_position{0.f};    // Target boom position [m]
	float _target_articulation_angle{0.f}; // Target articulation angle [rad]
	float _target_tilt_angle{0.f};      // Target tilt angle [rad]

	// Current states
	float _current_boom_position{0.f};
	float _current_articulation_angle{0.f};
	float _current_tilt_angle{0.f};

	// Vehicle parameters
	struct VehicleParams {
		float boom_min{0.5f};              // Minimum boom position [m]
		float boom_max{3.0f};              // Maximum boom position [m]
		float tilt_min{-1.57f};            // Minimum tilt angle [rad]
		float tilt_max{1.57f};             // Maximum tilt angle [rad]
		float articulation_min{-0.785f};   // Minimum articulation angle [rad] (-45 deg)
		float articulation_max{0.785f};    // Maximum articulation angle [rad] (+45 deg)
	} _vehicle_params;

	// Control parameters
	struct ControlParams {
		float rc_deadband{0.05f};          // RC deadband (0-1)
		float max_chassis_velocity{1.0f};  // Max chassis velocity [m/s]
		float max_boom_velocity{0.5f};     // Max boom velocity [m/s]
		float max_articulation_rate{0.3f}; // Max articulation rate [rad/s]
		float max_tilt_rate{0.5f};        // Max tilt rate [rad/s]
	} _control_params;

	// Update rate
	static constexpr float UPDATE_RATE = 50.0f;  // 50 Hz
	static constexpr float UPDATE_DT = 1.0f / UPDATE_RATE;
};
