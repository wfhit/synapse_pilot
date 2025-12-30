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
 * @file steering_controller.hpp
 * Steering controller for articulated wheel loader
 *
 * @author PX4 Development Team
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/steering_setpoint.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/robotic_servo_setpoint.h>
#include <uORB/topics/robotic_servo_status.h>
#include <uORB/topics/sensor_limit_switch.h>
#include <uORB/topics/parameter_update.h>

/**
 * @brief Steering Controller for articulated wheel loader
 *
 * Controls ST3125 servo-based steering with safety features including:
 * - Position-based command processing
 * - Limit sensor integration for safety
 * - Servo health monitoring
 * - Emergency stop handling
 */
class SteeringController : public ModuleBase<SteeringController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	SteeringController();
	~SteeringController() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	static constexpr float DEFAULT_CONTROL_RATE_HZ = 50.0f;
	static constexpr uint64_t CONTROL_INTERVAL_US = 20000; ///< 20ms = 50Hz

	void Run() override;

	/**
	 * Core control functions
	 */
	void process_steering_command();
	void send_servo_command(float position_rad);
	void process_servo_feedback();
	void process_limit_sensors();
	void handle_abnormal_events();
	void publish_steering_status();

	// uORB subscriptions
	uORB::Subscription _steering_setpoint_sub{ORB_ID(steering_setpoint)};
	uORB::Subscription _servo_feedback_sub{ORB_ID(robotic_servo_status)};
	uORB::SubscriptionMultiArray<sensor_limit_switch_s> _limit_sensor_sub{ORB_ID::sensor_limit_switch};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// uORB publications
	uORB::Publication<robotic_servo_setpoint_s> _servo_command_pub{ORB_ID(robotic_servo_setpoint)};
	uORB::Publication<steering_status_s> _steering_status_pub{ORB_ID(steering_status)};

	// State variables
	float _target_angle_rad{0.0f};      ///< Target steering angle from setpoint
	float _current_angle_rad{0.0f};     ///< Current servo position from feedback
	uint64_t _last_command_time{0};     ///< Last steering command timestamp
	uint64_t _last_feedback_time{0};    ///< Last servo feedback timestamp

	// Limit sensor state
	bool _left_limit_active{false};
	bool _right_limit_active{false};
	bool _limit_sensors_healthy{true};

	// Servo state
	bool _servo_healthy{true};
	uint16_t _servo_error_flags{0};
	float _servo_current_a{0.0f};
	float _servo_temperature_c{0.0f};

	// Emergency state
	bool _emergency_stop{false};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::STEER_MAX_ANG>) _max_steering_angle,
		(ParamInt<px4::params::STEER_ST3125_ID>) _st3125_servo_id,
		(ParamFloat<px4::params::STEER_ST3125_CR>) _st3125_current_limit,
		(ParamInt<px4::params::STEER_LT_LF_ID>) _limit_left_instance,
		(ParamInt<px4::params::STEER_LT_RT_ID>) _limit_right_instance,
		(ParamFloat<px4::params::STEER_CMD_TO>) _command_timeout_ms,
		(ParamFloat<px4::params::STEER_FB_TO>) _feedback_timeout_ms
	)

	// Helper methods
	void update_parameters();
	float saturate_angle(float angle_rad);
	bool is_command_timeout();
	uint8_t get_left_limit() const { return _limit_left_instance.get(); }
	uint8_t get_right_limit() const { return _limit_right_instance.get(); }
	bool is_feedback_timeout();
};
