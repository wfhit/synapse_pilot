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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/load_lamp_command.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class LoadLampController : public ModuleBase<LoadLampController>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	LoadLampController();
	~LoadLampController() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;
	void set_test_blink_rate(uint32_t interval_us);
	void set_test_load(float load);

	// Test mode methods
	void test_mode_enable();
	void test_mode_disable();

private:
	void Run() override;
	// Subscriptions
	uORB::Subscription _load_lamp_command_sub{ORB_ID(load_lamp_command)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _load_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": load_update")};

	// State variables
	bool _lamps_on{false};
	float _current_load{0.0f};
	uint64_t _last_toggle{0};
	uint32_t _blink_interval_us{500000}; // Default 2Hz

	// Test mode variables
	bool _test_mode_active{false};

	// Methods
	void parameters_update();
	void update_load();
	void update_blink_rate(float load);
	void toggle_lamps();
	void set_lamps(bool on);

	// GPIO initialization state
	bool _gpio_initialized{false};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::LOAD_LAMP_LOW>) _param_threshold_low,
		(ParamFloat<px4::params::LOAD_LAMP_MED>) _param_threshold_med,
		(ParamFloat<px4::params::LOAD_LAMP_HIGH>) _param_threshold_high
	)
};
