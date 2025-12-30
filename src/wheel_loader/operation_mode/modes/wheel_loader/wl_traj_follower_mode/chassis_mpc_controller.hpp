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
 * @file chassis_mpc_controller.hpp
 *
 * Model Predictive Controller for articulated wheel loader chassis
 *
 * @author Your Name
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

struct ChassisState {
	float x{0.f};              // Position x [m]
	float y{0.f};              // Position y [m]
	float heading{0.f};        // Heading angle [rad]
	float articulation{0.f};   // Articulation angle [rad]
	float velocity{0.f};       // Forward velocity [m/s]
	float yaw_rate{0.f};       // Yaw rate [rad/s]
};

struct ChassisControl {
	float velocity{0.f};           // Forward velocity command [m/s]
	float yaw_rate{0.f};           // Yaw rate command [rad/s]
	float articulation_rate{0.f};  // Articulation rate command [rad/s]
};

struct MPCWeights {
	float position_x{10.f};
	float position_y{10.f};
	float heading{5.f};
	float articulation{3.f};
	float velocity{1.f};
	float yaw_rate{1.f};
	float articulation_rate{1.5f};
	float velocity_change{0.5f};
	float yaw_rate_change{0.5f};
	float terminal_position{20.f};
	float terminal_heading{10.f};
	float terminal_articulation{5.f};
};

struct VehicleParams {
	float front_length{2.5f};      // Front section length [m]
	float rear_length{2.0f};       // Rear section length [m]
	float max_velocity{2.0f};      // Max velocity [m/s]
	float max_yaw_rate{0.5f};      // Max yaw rate [rad/s]
	float max_articulation{0.785f}; // Max articulation angle [rad]
	float max_articulation_rate{0.3f}; // Max articulation rate [rad/s]
};

class ChassisMPCController
{
public:
	ChassisMPCController();
	~ChassisMPCController() = default;

	/**
	 * Set MPC parameters
	 */
	void setParameters(int prediction_horizon, int control_horizon, float dt);

	/**
	 * Set cost function weights
	 */
	void setWeights(const MPCWeights &weights);

	/**
	 * Set vehicle parameters
	 */
	void setVehicleParams(const VehicleParams &params);

	/**
	 * Compute optimal control using MPC
	 * @param current_state Current chassis state
	 * @param target_state Target chassis state
	 * @param control_output Output control command
	 * @return true if successful
	 */
	bool computeControl(const ChassisState &current_state, const ChassisState &target_state,
			    ChassisControl &control_output);

	/**
	 * Predict next state given current state and control
	 */
	ChassisState predictNextState(const ChassisState &state, const ChassisControl &control, float dt);

private:
	/**
	 * Articulated vehicle kinematics model
	 */
	void articulatedKinematics(const ChassisState &state, const ChassisControl &control,
				   float &dx, float &dy, float &dheading, float &darticulation);

	/**
	 * Compute cost for a given state and control
	 */
	float computeCost(const ChassisState &state, const ChassisState &target,
			  const ChassisControl &control, const ChassisControl &prev_control, bool is_terminal);

	/**
	 * Simple optimization using gradient descent
	 * (In production, use proper QP solver)
	 */
	bool optimizeTrajectory(const ChassisState &current_state, const ChassisState &target_state,
				ChassisControl &optimal_control);

	int _prediction_horizon{10};
	int _control_horizon{5};
	float _dt{0.02f};

	MPCWeights _weights;
	VehicleParams _vehicle_params;

	ChassisControl _prev_control;
};
