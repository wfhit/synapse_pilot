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

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/pid/PID.hpp>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/hbridge_setpoint.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/drivetrain_setpoint.h>
#include <uORB/topics/drivetrain_status.h>

using namespace time_literals;

/**
 * @brief Individual drivetrain controller for articulated wheel loader
 *
 * Implements closed-loop speed control using quadrature encoder feedback
 * and DRV8701 H-bridge motor driver interface.
 *
 * Features:
 * - PID speed control with configurable gains
 * - Low-pass filtering for noise reduction
 * - Safety monitoring and emergency stop
 * - Instance-based multi-axle support
 * - Performance monitoring
 */
class DrivetrainController : public ModuleBase<DrivetrainController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DrivetrainController();
	~DrivetrainController() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	static constexpr uint32_t SCHEDULE_INTERVAL{10_ms};	// 100Hz control loop
	static constexpr float CONTROL_DT{0.01f};		// 100Hz = 0.01s
	static constexpr float MAX_PWM_VALUE{1.0f};
	static constexpr float MIN_PWM_VALUE{-1.0f};
	static constexpr uint64_t SETPOINT_TIMEOUT_US{500_ms};
	static constexpr uint64_t ENCODER_TIMEOUT_US{100_ms};
	static constexpr float DEFAULT_FILTER_FREQ{10.0f};	// Hz

	void Run() override;

	// Core control functions
	bool update_speed_setpoint();
	void update_encoder_feedback();
	void update_hbridge_status();
	void run_speed_controller();
	void publish_motor_command();
	void check_safety_conditions();

	// Parameter and utility functions
	void parameters_update();
	float constrain_pwm(float value) const;
	bool is_setpoint_valid() const;
	void set_speed_setpoint(float speed_rad_s);

	// uORB subscriptions
	uORB::Subscription _param_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _drivetrain_setpoint_sub{ORB_ID(drivetrain_setpoint), 0}; // Will be updated in init()

	// Instance-specific subscriptions - initialized in init()
	uORB::SubscriptionMultiArray<sensor_quad_encoder_s> _encoder_sub{ORB_ID::sensor_quad_encoder};
	uORB::SubscriptionMultiArray<hbridge_status_s> _hbridge_status_sub{ORB_ID::hbridge_status};

	// uORB publications
	uORB::PublicationMulti<hbridge_setpoint_s> _motor_cmd_pub{ORB_ID(hbridge_setpoint)};
	uORB::PublicationMulti<drivetrain_status_s> _drivetrain_status_pub{ORB_ID(drivetrain_status)};

	// Control system
	PID _speed_controller;
	math::LowPassFilter2p<float> _speed_filter;

	// State tracking
	struct {
		float speed_rad_s{0.0f};		// Current measured speed
		float setpoint_rad_s{0.0f};		// Target speed
		float pwm_output{0.0f};			// Motor PWM command
		uint64_t last_setpoint_us{0};		// Last valid setpoint timestamp
		uint64_t last_encoder_us{0};		// Last encoder update
		bool motor_enabled{false};		// Motor enable state
		bool emergency_stop{false};		// Emergency stop active
		bool initialized{false};		// Initialization complete
	} _state;

	// Performance monitoring
	perf_counter_t _loop_perf{nullptr};
	perf_counter_t _control_perf{nullptr};

	// Module parameters (PX4 naming: ≤16 chars)
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::DTC_ENC_ID>) _param_encoder_id,
		(ParamInt<px4::params::DTC_MOTOR_CH>) _param_motor_channel,
		(ParamFloat<px4::params::DTC_P>) _param_speed_p,
		(ParamFloat<px4::params::DTC_I>) _param_speed_i,
		(ParamFloat<px4::params::DTC_D>) _param_speed_d,
		(ParamFloat<px4::params::DTC_I_MAX>) _param_integrator_max,
		(ParamFloat<px4::params::DTC_MAX_SPD>) _param_max_speed,
		(ParamFloat<px4::params::DTC_FILT_HZ>) _param_filter_freq,
		(ParamInt<px4::params::DTC_FRONT>) _param_is_front_wheel,
		(ParamFloat<px4::params::DTC_TIMEOUT>) _param_setpoint_timeout
	)
};
