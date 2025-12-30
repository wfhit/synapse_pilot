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

#pragma once

// System includes
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>

// PX4 control libraries
#include <lib/pid/PID.hpp>
#include <lib/motion_planning/VelocitySmoothing.hpp>
#include <lib/motion_planning/PositionSmoothing.hpp>
#include <mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

// Matrix library
#include <matrix/matrix/Matrix.hpp>

using namespace matrix;

/**
 * @brief Handles trajectory planning and motion control for tilt actuator
 *
 * This class provides:
 * - Smooth trajectory generation with jerk limiting
 * - Cascade PID control (position + velocity)

 * - Safety monitoring and constraint enforcement
 */
class TiltMotionController : public ModuleParams
{
public:
	/**
	 * @brief Motion setpoint structure for trajectory generation
	 */
	struct MotionSetpoint {
		float position;       // mm
		float velocity;       // mm/s
		float acceleration;   // mm/s²
		float jerk;          // mm/s³
	};

	/**
	 * @brief Control output structure
	 */
	struct ControlOutput {
		float duty_cycle;             // Motor duty cycle (-1.0 to 1.0)
		float position_error;         // Position tracking error (mm)
		float velocity_error;         // Velocity tracking error (mm/s)
		bool safety_stop;            // Safety stop flag
	};

	/**
	 * @brief Controller configuration structure
	 */
	struct ControllerConfig {
		// PID gains
		float position_p, position_i, position_d;
		float velocity_p, velocity_i, velocity_d;

		// Motion constraints
		float max_velocity;           // mm/s
		float max_acceleration;       // mm/s²
		float max_jerk;              // mm/s³

		// Safety limits
		float position_min, position_max;  // mm
		float duty_cycle_limit;            // Maximum duty cycle
	};


	explicit TiltMotionController(ModuleParams *parent);

	/**
	 * @brief Initialize controller with configuration
	 * @param config Controller configuration
	 */
	void initialize(const ControllerConfig& config);

	/**
	 * @brief Update controller configuration
	 * @param config New configuration
	 */
	void update_config(const ControllerConfig& config);

	/**
	 * @brief Generate smooth trajectory to target position
	 * @param current_position Current actuator position (mm)
	 * @param target_position Target actuator position (mm)
	 * @param dt Control loop time step (s)
	 * @return Motion setpoint for current time step
	 */
	MotionSetpoint plan_trajectory(float current_position, float target_position, float dt);

	/**
	 * @brief Compute control output
	 * @param setpoint Motion setpoint from trajectory planner
	 * @param current_position Current actuator position (mm)
	 * @param current_velocity Current actuator velocity (mm/s)
	 * @param dt Control loop time step (s)
	 * @return Control output
	 */
	ControlOutput compute_control(
		const MotionSetpoint& setpoint,
		float current_position,
		float current_velocity,
		float dt
	);



	/**
	 * @brief Reset controller state (clear integrators, etc.)
	 */
	void reset();

	/**
	 * @brief Set safety limits for position
	 * @param min_position Minimum safe position (mm)
	 * @param max_position Maximum safe position (mm)
	 */
	void set_safety_limits(float min_position, float max_position);

	/**
	 * @brief Update controller parameters from parameter system
	 * This should be called when parameters change to reconfigure the controller
	 */
	void update_parameters();

	/**
	 * @brief Get controller performance metrics
	 * @param position_rms_error Output: RMS position error
	 * @param velocity_rms_error Output: RMS velocity error
	 * @param control_effort Output: Average control effort
	 */
	void get_performance_metrics(
		float& position_rms_error,
		float& velocity_rms_error,
		float& control_effort) const;

private:
	// Control components
	PID _position_controller;
	PID _velocity_controller;
	VelocitySmoothing _velocity_smoother;
	PositionSmoothing _position_smoother;

	// Configuration
	ControllerConfig _config;
	bool _initialized{false};

	// Performance tracking
	struct PerformanceMetrics {
		float position_error_sum_squared{0.0f};
		float velocity_error_sum_squared{0.0f};
		float control_effort_sum{0.0f};
		uint32_t sample_count{0};
	} _performance_metrics;

	// Safety state
	bool _safety_stop_active{false};
	float _position_min_safe{0.0f};
	float _position_max_safe{1000.0f};



	/**
	 * @brief Check safety constraints
	 * @param position Current position (mm)
	 * @param velocity Current velocity (mm/s)
	 * @param setpoint Motion setpoint
	 * @return True if safe to continue
	 */
	bool check_safety_constraints(float position, float velocity, const MotionSetpoint& setpoint);

	/**
	 * @brief Update performance metrics
	 * @param position_error Position tracking error (mm)
	 * @param velocity_error Velocity tracking error (mm/s)
	 * @param control_output Control effort (duty cycle)
	 */
	void update_performance_metrics(float position_error, float velocity_error, float control_output);

	/**
	 * @brief Compute feedforward term for better tracking
	 * @param setpoint Motion setpoint
	 * @return Feedforward control term
	 */
	float compute_feedforward(const MotionSetpoint& setpoint);

	// Control parameters
	DEFINE_PARAMETERS(
		// Position PID gains
		(ParamFloat<px4::params::BCT_POS_P>) _param_position_p,
		(ParamFloat<px4::params::BCT_POS_I>) _param_position_i,
		(ParamFloat<px4::params::BCT_POS_D>) _param_position_d,

		// Velocity PID gains
		(ParamFloat<px4::params::BCT_VEL_P>) _param_velocity_p,
		(ParamFloat<px4::params::BCT_VEL_I>) _param_velocity_i,
		(ParamFloat<px4::params::BCT_VEL_D>) _param_velocity_d,

		// Motion constraints
		(ParamFloat<px4::params::BCT_MAX_VEL>) _param_max_velocity,
		(ParamFloat<px4::params::BCT_MAX_ACC>) _param_max_acceleration,
		(ParamFloat<px4::params::BCT_MAX_JERK>) _param_max_jerk,

		// Safety limits
		(ParamFloat<px4::params::BCT_DUTY_LIM>) _param_duty_cycle_limit
	)

	// Safety monitoring constants
	static constexpr float VELOCITY_LIMIT_TOLERANCE = 1.1f;  // 10% tolerance for velocity limits
};
