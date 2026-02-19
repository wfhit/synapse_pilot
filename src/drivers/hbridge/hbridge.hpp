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
 * @file hbridge.hpp
 *
 * Multi-instance H-Bridge motor driver
 * Each instance controls one H-bridge channel with shared enable control
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/atomic.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_motor_pwm.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/hbridge_setpoint.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/sensor_limit_switch.h>
#include <board_config.h>
#include "hbridge_config.h"

class HBridge : public ModuleBase<HBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	// Module configuration
	static constexpr uint8_t MAX_INSTANCES = 2;  // Support 2 H-bridge channels
	static constexpr unsigned SCHEDULE_INTERVAL = 10000;  // 10ms = 100Hz update rate
	static constexpr uint8_t MANAGER_INSTANCE = 255;        // Instance 255 manages shared enable

	HBridge(uint8_t instance);
	~HBridge() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void updateParams() override;

	// Multi-instance management
	static HBridge *_instances[MAX_INSTANCES];
	static px4::atomic<uint8_t> _num_instances;
	static HBridge *_manager_instance; // Special manager instance
	static px4::atomic<uint32_t> _initialized_pwm_channels; // Track initialized PWM channels

	// Instance details
	const uint8_t _instance;
	const hbridge_config_t *_board_config{nullptr};

	// Current state
	float _current_duty_cycle{0.0f};
	bool _forward_limit_active{false};
	bool _reverse_limit_active{false};
	bool _initialized{false};
	bool _manual_mode{false};

	// Publications
	orb_advert_t _pub_handle{nullptr};

	// Subscriptions (instance-specific command subscription)
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::SubscriptionMultiArray<hbridge_setpoint_s> _command_sub{ORB_ID::hbridge_setpoint};
	uORB::SubscriptionMultiArray<sensor_limit_switch_s> _limit_sensor_sub{ORB_ID::sensor_limit_switch};

	// Performance counters
	perf_counter_t _loop_perf{nullptr};
	perf_counter_t _command_perf{nullptr};

	// Hardware control
	bool configure_hardware();
	void output_pwm();
	void set_direction(bool forward);
	void output_enable(bool enable);

	// Command processing
	void process_commands();
	void process_limit_sensors();

	// Safety and limits
	void apply_safety_limits();

	// Utilities
	bool is_manager_instance() const
	{
		return _instance == MANAGER_INSTANCE;
	}
	int get_fwd_limit() const
	{
		return _instance == 0 ? _param_0_fwd_limit.get() : _param_1_fwd_limit.get();
	}
	int get_rev_limit() const
	{
		return _instance == 0 ? _param_0_rev_limit.get() : _param_1_rev_limit.get();
	}
	bool get_dir_reverse() const
	{
		return _instance == 0 ? (_param_dir_reverse_0.get() != 0) : (_param_dir_reverse_1.get() != 0);
	}
	void publish_status();

	// Board configuration
	static const hbridge_config_t *get_board_config(uint8_t instance);
	static const hbridge_manager_config_t *get_manager_config();
	static bool start_instance(int instance);
	static void stop_all_instances();
	static void print_instance_status(uint8_t instance);  // Debug helper

	// Parameters (instance-specific)
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::HBRIDGE_DIR_REV0>) _param_dir_reverse_0,
		(ParamInt<px4::params::HBRIDGE_DIR_REV1>) _param_dir_reverse_1,
		(ParamInt<px4::params::HBRIDGE_0_FLIM>) _param_0_fwd_limit,
		(ParamInt<px4::params::HBRIDGE_0_RLIM>) _param_0_rev_limit,
		(ParamInt<px4::params::HBRIDGE_1_FLIM>) _param_1_fwd_limit,
		(ParamInt<px4::params::HBRIDGE_1_RLIM>) _param_1_rev_limit
	)
};
