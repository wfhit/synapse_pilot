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

#include "chassis_mpc_controller.hpp"
#include <mathlib/mathlib.h>

ChassisMPCController::ChassisMPCController()
{
	// Initialize with default weights
	_weights = MPCWeights();
	_vehicle_params = VehicleParams();
}

void ChassisMPCController::setParameters(int prediction_horizon, int control_horizon, float dt)
{
	_prediction_horizon = prediction_horizon;
	_control_horizon = control_horizon;
	_dt = dt;
}

void ChassisMPCController::setWeights(const MPCWeights &weights)
{
	_weights = weights;
}

void ChassisMPCController::setVehicleParams(const VehicleParams &params)
{
	_vehicle_params = params;
}

bool ChassisMPCController::computeControl(const ChassisState &current_state, const ChassisState &target_state,
		ChassisControl &control_output)
{
	// Optimize control sequence
	if (!optimizeTrajectory(current_state, target_state, control_output)) {
		return false;
	}

	// Apply constraints
	control_output.velocity = math::constrain(control_output.velocity,
				  -_vehicle_params.max_velocity, _vehicle_params.max_velocity);
	control_output.yaw_rate = math::constrain(control_output.yaw_rate,
				  -_vehicle_params.max_yaw_rate, _vehicle_params.max_yaw_rate);
	control_output.articulation_rate = math::constrain(control_output.articulation_rate,
					   -_vehicle_params.max_articulation_rate, _vehicle_params.max_articulation_rate);

	_prev_control = control_output;
	return true;
}

ChassisState ChassisMPCController::predictNextState(const ChassisState &state, const ChassisControl &control,
		float dt)
{
	ChassisState next_state = state;

	float dx, dy, dheading, darticulation;
	articulatedKinematics(state, control, dx, dy, dheading, darticulation);

	// Integrate using Euler method
	next_state.x += dx * dt;
	next_state.y += dy * dt;
	next_state.heading += dheading * dt;
	next_state.articulation += darticulation * dt;
	next_state.velocity = control.velocity;
	next_state.yaw_rate = control.yaw_rate;

	// Wrap angles
	next_state.heading = matrix::wrap_pi(next_state.heading);
	next_state.articulation = math::constrain(next_state.articulation,
				  -_vehicle_params.max_articulation, _vehicle_params.max_articulation);

	return next_state;
}

void ChassisMPCController::articulatedKinematics(const ChassisState &state, const ChassisControl &control,
		float &dx, float &dy, float &dheading, float &darticulation)
{
	// Articulated vehicle kinematics
	// Using simplified model: front section steers, rear follows
	
	float v = control.velocity;
	float omega = control.yaw_rate;
	float alpha = state.articulation; // Current articulation angle
	float alpha_dot = control.articulation_rate;

	// Vehicle heading derivatives considering articulation
	// Simplified model: effective turning is combination of yaw rate and articulation
	float L_front = _vehicle_params.front_length;
	float L_rear = _vehicle_params.rear_length;
	float L_total = L_front + L_rear;

	// Velocity components in global frame
	dx = v * cosf(state.heading);
	dy = v * sinf(state.heading);

	// Heading rate: combination of commanded yaw rate and articulation effect
	// Articulation causes additional yaw rate proportional to velocity and articulation angle
	float articulation_yaw_contribution = (v / L_total) * sinf(alpha);
	dheading = omega + articulation_yaw_contribution;

	// Articulation angle rate
	darticulation = alpha_dot;
}

float ChassisMPCController::computeCost(const ChassisState &state, const ChassisState &target,
					const ChassisControl &control, const ChassisControl &prev_control, bool is_terminal)
{
	float cost = 0.f;

	// Position error
	float dx = state.x - target.x;
	float dy = state.y - target.y;
	float position_weight = is_terminal ? _weights.terminal_position : (_weights.position_x + _weights.position_y) / 2.f;
	cost += position_weight * (dx * dx + dy * dy);

	// Heading error
	float heading_error = matrix::wrap_pi(state.heading - target.heading);
	float heading_weight = is_terminal ? _weights.terminal_heading : _weights.heading;
	cost += heading_weight * heading_error * heading_error;

	// Articulation error
	float articulation_error = state.articulation - target.articulation;
	float articulation_weight = is_terminal ? _weights.terminal_articulation : _weights.articulation;
	cost += articulation_weight * articulation_error * articulation_error;

	if (!is_terminal) {
		// Control effort
		cost += _weights.velocity * control.velocity * control.velocity;
		cost += _weights.yaw_rate * control.yaw_rate * control.yaw_rate;
		cost += _weights.articulation_rate * control.articulation_rate * control.articulation_rate;

		// Control smoothness (rate of change)
		float dv = control.velocity - prev_control.velocity;
		float domega = control.yaw_rate - prev_control.yaw_rate;
		cost += _weights.velocity_change * dv * dv;
		cost += _weights.yaw_rate_change * domega * domega;
	}

	return cost;
}

bool ChassisMPCController::optimizeTrajectory(const ChassisState &current_state, const ChassisState &target_state,
		ChassisControl &optimal_control)
{
	// Simplified optimization: use gradient-based approach
	// In production, use proper QP solver (e.g., OSQP, qpOASES)

	// Initialize with simple proportional control as starting point
	float dx = target_state.x - current_state.x;
	float dy = target_state.y - current_state.y;
	float distance = sqrtf(dx * dx + dy * dy);
	float desired_heading = atan2f(dy, dx);
	float heading_error = matrix::wrap_pi(desired_heading - current_state.heading);

	// Initial guess
	ChassisControl best_control;
	best_control.velocity = math::constrain(distance * 2.0f, -_vehicle_params.max_velocity,
						_vehicle_params.max_velocity);
	best_control.yaw_rate = math::constrain(heading_error * 3.0f, -_vehicle_params.max_yaw_rate,
						_vehicle_params.max_yaw_rate);
	best_control.articulation_rate = math::constrain((target_state.articulation - current_state.articulation) * 2.0f,
					 -_vehicle_params.max_articulation_rate, _vehicle_params.max_articulation_rate);

	// Evaluate cost for initial control
	ChassisState predicted_state = predictNextState(current_state, best_control, _dt);
	float best_cost = computeCost(predicted_state, target_state, best_control, _prev_control, false);

	// Simple grid search for better control
	const int search_samples = 5;
	const float vel_range = _vehicle_params.max_velocity * 0.5f;
	const float yaw_range = _vehicle_params.max_yaw_rate * 0.5f;
	const float art_range = _vehicle_params.max_articulation_rate * 0.5f;

	for (int v_idx = 0; v_idx < search_samples; ++v_idx) {
		for (int y_idx = 0; y_idx < search_samples; ++y_idx) {
			for (int a_idx = 0; a_idx < search_samples; ++a_idx) {
				ChassisControl test_control;
				test_control.velocity = best_control.velocity + (v_idx - search_samples / 2) * (vel_range / search_samples);
				test_control.yaw_rate = best_control.yaw_rate + (y_idx - search_samples / 2) * (yaw_range / search_samples);
				test_control.articulation_rate = best_control.articulation_rate + (a_idx - search_samples / 2) *
								 (art_range / search_samples);

				// Apply constraints
				test_control.velocity = math::constrain(test_control.velocity,
									-_vehicle_params.max_velocity, _vehicle_params.max_velocity);
				test_control.yaw_rate = math::constrain(test_control.yaw_rate,
									-_vehicle_params.max_yaw_rate, _vehicle_params.max_yaw_rate);
				test_control.articulation_rate = math::constrain(test_control.articulation_rate,
								 -_vehicle_params.max_articulation_rate, _vehicle_params.max_articulation_rate);

				// Predict and evaluate
				ChassisState test_state = predictNextState(current_state, test_control, _dt);
				float test_cost = computeCost(test_state, target_state, test_control, _prev_control, false);

				if (test_cost < best_cost) {
					best_cost = test_cost;
					best_control = test_control;
				}
			}
		}
	}

	optimal_control = best_control;
	return true;
}
