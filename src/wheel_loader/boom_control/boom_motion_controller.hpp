/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module_params.h>
#include <lib/pid/PID.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <matrix/matrix/math.hpp>

using namespace matrix;

/**
 * @brief Motion controller for boom movement
 *
 * Handles trajectory planning and control:
 * - Smooth trajectory generation
 * - Position/velocity control
 * - Load compensation
 * - Safety limiting
 */
class BoomMotionController : public ModuleParams
{
public:
	enum class ControlMode {
		POSITION,      // Position control with trajectory planning
		VELOCITY,      // Direct velocity control
		FORCE,         // Force/torque control (future)
		MANUAL        // Direct duty cycle control
	};

	struct MotionSetpoint {
		float position;        // rad - Target boom angle
		float velocity;        // rad/s - Target velocity
		float acceleration;    // rad/s^2 - Target acceleration
		float feedforward;     // Feedforward term
	};

	struct ControlOutput {
		float duty_cycle;      // -1 to 1 - Motor duty cycle
		float velocity_limit;  // rad/s - Applied velocity limit
		bool at_target;       // True if at target position
		bool limited;         // True if output was limited
	};

	explicit BoomMotionController(ModuleParams *parent);

	/**
	 * @brief Initialize controller
	 * @return True if initialization successful
	 */
	bool initialize();

	/**
	 * @brief Set control mode
	 * @param mode Desired control mode
	 */
	void set_mode(ControlMode mode);

	/**
	 * @brief Set target position
	 * @param angle Target boom angle in radians
	 * @param velocity_limit Maximum velocity in rad/s (0 for default)
	 */
	void set_target_position(float angle, float velocity_limit = 0.0f);

	/**
	 * @brief Set target velocity
	 * @param velocity Target velocity in rad/s
	 */
	void set_target_velocity(float velocity);

	/**
	 * @brief Update trajectory planning
	 * @param current_position Current boom angle in radians
	 * @param dt Time step in seconds
	 * @return Motion setpoint for current timestep
	 */
	MotionSetpoint update_trajectory(float current_position, float dt);

	/**
	 * @brief Compute control output
	 * @param setpoint Motion setpoint
	 * @param current_position Current position in radians
	 * @param current_velocity Current velocity in rad/s
	 * @param dt Time step in seconds
	 * @return Control output
	 */
	ControlOutput compute_control(const MotionSetpoint &setpoint,
				      float current_position,
				      float current_velocity,
				      float dt);

	/**
	 * @brief Set load compensation
	 * @param load_estimate Estimated load (0-1)
	 */
	void set_load_compensation(float load_estimate);

	/**
	 * @brief Reset controller state
	 */
	void reset();

	/**
	 * @brief Emergency stop
	 */
	void emergency_stop();

	/**
	 * @brief Update motion controller parameters from parameter system
	 * This should be called when parameters change to reconfigure the motion controller
	 */
	void update_parameters();

private:
	// Control mode and state
	ControlMode _mode{ControlMode::POSITION};
	float _target_position{0.0f};
	float _target_velocity{0.0f};
	float _velocity_limit_override{0.0f};
	float _load_compensation{0.0f};

	// Controllers
	PID _position_controller;
	PID _velocity_controller;
	PositionSmoothing _trajectory_generator;

	// Trajectory state
	Vector3f _current_traj_pos;
	Vector3f _current_traj_vel;
	Vector3f _current_traj_acc;

	// Safety and limits
	bool _emergency_stop_active{false};
	float _last_output{0.0f};

	// Control parameters
	DEFINE_PARAMETERS(
		// PID gains
		(ParamFloat<px4::params::BOOM_POS_P>) _param_pos_p,
		(ParamFloat<px4::params::BOOM_POS_I>) _param_pos_i,
		(ParamFloat<px4::params::BOOM_POS_D>) _param_pos_d,
		(ParamFloat<px4::params::BOOM_VEL_P>) _param_vel_p,
		(ParamFloat<px4::params::BOOM_VEL_I>) _param_vel_i,
		(ParamFloat<px4::params::BOOM_VEL_D>) _param_vel_d,
		// Motion limits
		(ParamFloat<px4::params::BOOM_MAX_VEL>) _param_max_velocity,
		(ParamFloat<px4::params::BOOM_MAX_ACC>) _param_max_acceleration,
		(ParamFloat<px4::params::BOOM_MAX_JERK>) _param_max_jerk,
		// Control parameters
		(ParamFloat<px4::params::BOOM_DEADZONE>) _param_deadzone,
		(ParamFloat<px4::params::BOOM_FF_GAIN>) _param_feedforward_gain,
		(ParamFloat<px4::params::BOOM_LOAD_COM>) _param_load_comp_gain
	)

	/**
	 * @brief Apply deadzone compensation
	 * @param output Raw control output
	 * @return Compensated output
	 */
	float apply_deadzone_compensation(float output) const;

	/**
	 * @brief Apply output limiting
	 * @param output Raw output
	 * @return Limited output
	 */
	float apply_limits(float output) const;

	/**
	 * @brief Calculate feedforward term
	 * @param setpoint Motion setpoint
	 * @return Feedforward control term
	 */
	float calculate_feedforward(const MotionSetpoint &setpoint) const;
};
