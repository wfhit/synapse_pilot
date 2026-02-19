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
#include <px4_platform_common/tasks.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>

class DriverLampController : public ModuleBase<DriverLampController>, public ModuleParams
{
public:
	DriverLampController();
	~DriverLampController() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run();
	int print_status();

private:
	// Lamp modes
	enum class LampMode {
		OFF,
		REVERSE,      // Both solid ON
		LEFT_TURN,    // Left blink
		RIGHT_TURN,   // Right blink
		HAZARD        // Both blink short-short-long pattern
	};

	// Hazard pattern states
	enum class HazardState {
		FIRST_SHORT,
		FIRST_SHORT_OFF,
		SECOND_SHORT,
		SECOND_SHORT_OFF,
		LONG,
		LONG_OFF
	};

	// Subscriptions
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _mode_update_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": mode_update")};

	// State variables
	LampMode _current_mode{LampMode::OFF};
	bool _left_lamp_on{false};
	bool _right_lamp_on{false};
	uint64_t _last_toggle{0};
	HazardState _hazard_state{HazardState::FIRST_SHORT};
	uint64_t _hazard_timer{0};

	// Test mode variables
	bool _test_mode_active{false};

	// Constants for timing
	static constexpr uint32_t HAZARD_SHORT_ON_US = 100000;   // 100ms on
	static constexpr uint32_t HAZARD_SHORT_OFF_US = 100000;  // 100ms off
	static constexpr uint32_t HAZARD_LONG_ON_US = 400000;    // 400ms on
	static constexpr uint32_t HAZARD_LONG_OFF_US = 100000;   // 100ms off

	// Methods
	void parameters_update();
	void update_lamp_mode();
	void process_lamp_state();
	void set_lamps(bool left, bool right);
	void process_hazard_pattern();
	uint32_t get_turn_signal_interval_us() const;

	// Test mode methods
	int test_mode(int argc, char *argv[]);

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::DRV_LAMP_BLINK>) _param_blink_rate
	)

	static int run_trampoline(int argc, char *argv[]);
};
