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

#include "load_lamp_controller.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <board_config.h>

#ifndef BOARD_HAS_LOAD_LAMP
#warning "BOARD_HAS_LOAD_LAMP not defined - load lamp controller module will be disabled"
#endif

LoadLampController::LoadLampController() : ModuleParams(nullptr)
{
}

LoadLampController::~LoadLampController()
{
	perf_free(_loop_perf);
	perf_free(_load_update_perf);
}

void LoadLampController::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void LoadLampController::update_load()
{
	perf_begin(_load_update_perf);

	// Skip normal load updates if in test mode
	if (_test_mode_active) {
		perf_end(_load_update_perf);
		return;
	}

	// Check for load lamp commands from the main board (X7+)
	load_lamp_command_s cmd;
	if (_load_lamp_command_sub.update(&cmd)) {
		// Update current load value and blink interval from command
		_current_load = cmd.load_value;
		_blink_interval_us = cmd.blink_interval_us;

		// Handle special case for lamp off
		if (cmd.load_level == load_lamp_command_s::LOAD_OFF) {
			_lamps_on = false;
			set_lamps(false);
		}
		// Note: Blinking is handled in the main run loop
	}

	perf_end(_load_update_perf);
}

void LoadLampController::update_blink_rate(float load)
{
	// Map load to blink frequency based on thresholds
	if (load < 0.1f) {
		_blink_interval_us = 2000000; // 0.5 Hz (very slow)

	} else if (load < 0.2f) {
		_blink_interval_us = 1000000; // 1 Hz (slow)

	} else if (load < _param_threshold_low.get()) {
		_blink_interval_us = 500000; // 2 Hz (medium-slow)

	} else if (load < _param_threshold_med.get()) {
		_blink_interval_us = 200000; // 5 Hz (medium)

	} else if (load < _param_threshold_high.get()) {
		_blink_interval_us = 100000; // 10 Hz (fast)

	} else {
		_blink_interval_us = 50000;  // 20 Hz (very fast)
	}
}

void LoadLampController::toggle_lamps()
{
	_lamps_on = !_lamps_on;
	set_lamps(_lamps_on);
}

void LoadLampController::set_lamps(bool on)
{
#ifdef BOARD_HAS_LOAD_LAMP
	LOAD_LAMP_LEFT(on);
	LOAD_LAMP_RIGHT(on);
#endif
}

void LoadLampController::set_test_blink_rate(uint32_t interval_us)
{
	_blink_interval_us = interval_us;
}

void LoadLampController::set_test_load(float load)
{
	_current_load = load;
}

void LoadLampController::test_mode_enable()
{
	_test_mode_active = true;
}

void LoadLampController::test_mode_disable()
{
	_test_mode_active = false;
}

void LoadLampController::run()
{
#ifdef BOARD_HAS_LOAD_LAMP
	// Initialize load lamp GPIOs
	px4_arch_configgpio(GPIO_LOAD_LAMP_LEFT);
	px4_arch_configgpio(GPIO_LOAD_LAMP_RIGHT);
	px4_arch_configgpio(GPIO_LOAD_LAMP_GND);

	// Set ground pin to 0
	px4_arch_gpiowrite(GPIO_LOAD_LAMP_GND, 0);

	// Start with lamps off
	set_lamps(false);

	PX4_INFO("Load lamp controller started:");
	PX4_INFO("  Lamp 1 GPIO (PA4): configured");
	PX4_INFO("  Lamp 2 GPIO (PC1): configured");
	PX4_INFO("  Ground GPIO (PC0): configured");
#else
	PX4_WARN("Load lamp GPIO not available on this board - module disabled");
#endif

	while (!should_exit()) {
		perf_begin(_loop_perf);

		parameters_update();

		update_load();  // Check for load lamp commands

		// Handle blinking (skip if in test mode with manual blink rate)
		uint64_t now = hrt_absolute_time();

		if (now - _last_toggle >= _blink_interval_us) {
			toggle_lamps();
			_last_toggle = now;
		}

		perf_end(_loop_perf);
		px4_usleep(20000); // 50Hz update rate
	}

#ifdef BOARD_HAS_LOAD_LAMP
	// Shutdown sequence - turn off all lamps
	set_lamps(false);
	PX4_INFO("Load lamp controller shutdown complete");
#endif
}

int LoadLampController::print_status()
{
	PX4_INFO("Load Lamp Controller");
	PX4_INFO("  Current load value: %.2f", (double)_current_load);
	PX4_INFO("  Test mode: %s", _test_mode_active ? "active" : "inactive");
	PX4_INFO("  Blink interval: %lu us", _blink_interval_us);
	PX4_INFO("  Load thresholds: %.2f / %.2f / %.2f",
		 (double)_param_threshold_low.get(),
		 (double)_param_threshold_med.get(),
		 (double)_param_threshold_high.get());
	PX4_INFO("  Lamp state: %s", _lamps_on ? "ON" : "OFF");

#ifdef BOARD_HAS_LOAD_LAMP
	PX4_INFO("  GPIO Configuration:");
	PX4_INFO("    Left Lamp (PA4): configured");
	PX4_INFO("    Right Lamp (PC1): configured");
	PX4_INFO("    Ground (PC0): configured");
#else
	PX4_INFO("  GPIO available: no (board not supported)");
#endif
	perf_print_counter(_loop_perf);
	perf_print_counter(_load_update_perf);
	return 0;
}

int LoadLampController::task_spawn(int argc, char *argv[])
{
	LoadLampController *instance = new LoadLampController();

	if (instance) {
		_object.store(instance);
		_task_id = px4_task_spawn_cmd("load_lamp_controller",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT - 10,
					      1200,
					      (px4_main_t)&run_trampoline,
					      (char *const *)argv);

		if (_task_id < 0) {
			PX4_ERR("task start failed");
			delete instance;
			_object.store(nullptr);
			_task_id = -1;
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	PX4_ERR("alloc failed");
	return PX4_ERROR;
}

int LoadLampController::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("module not running");
		return 1;
	}

	if (!strcmp(argv[0], "test")) {
		if (argc < 2) {
			PX4_WARN("Usage: load_lamp_controller test <load_value>");
			PX4_INFO("Test with motor load value (0.0 - 1.0):");
			PX4_INFO("  0.0 - 0.1: Very slow blink (0.5 Hz)");
			PX4_INFO("  0.1 - 0.2: Slow blink (1 Hz)");
			PX4_INFO("  0.2 - 0.3: Medium-slow blink (2 Hz)");
			PX4_INFO("  0.3 - 0.6: Medium blink (5 Hz)");
			PX4_INFO("  0.6 - 0.8: Fast blink (10 Hz)");
			PX4_INFO("  0.8 - 1.0: Very fast blink (20 Hz)");
			PX4_INFO("Example: load_lamp_controller test 0.5");
			return 1;
		}

		float test_load = atof(argv[1]);

		if (test_load < 0.0f || test_load > 1.0f) {
			PX4_WARN("Invalid load value: %.2f. Valid range: 0.0-1.0", (double)test_load);
			return 1;
		}

		LoadLampController *instance = get_instance();

		if (instance) {
			// Enable test mode and set the current load to the test value
			instance->test_mode_enable();
			instance->set_test_load(test_load);

			// Calculate blink rate based on load thresholds (same logic as update_blink_rate)
			uint32_t blink_interval;
			const char* rate_description;

			if (test_load < 0.1f) {
				blink_interval = 2000000; // 0.5 Hz
				rate_description = "Very slow blink (0.5 Hz)";
			} else if (test_load < 0.2f) {
				blink_interval = 1000000; // 1 Hz
				rate_description = "Slow blink (1 Hz)";
			} else if (test_load < instance->_param_threshold_low.get()) {
				blink_interval = 500000; // 2 Hz
				rate_description = "Medium-slow blink (2 Hz)";
			} else if (test_load < instance->_param_threshold_med.get()) {
				blink_interval = 200000; // 5 Hz
				rate_description = "Medium blink (5 Hz)";
			} else if (test_load < instance->_param_threshold_high.get()) {
				blink_interval = 100000; // 10 Hz
				rate_description = "Fast blink (10 Hz)";
			} else {
				blink_interval = 50000; // 20 Hz
				rate_description = "Very fast blink (20 Hz)";
			}

			instance->set_test_blink_rate(blink_interval);
			PX4_INFO("Test mode enabled - Load set: %.2f -> %s", (double)test_load, rate_description);
			return 0;
		}

		return 1;
	}

	if (!strcmp(argv[0], "normal")) {
		LoadLampController *instance = get_instance();

		if (instance) {
			instance->test_mode_disable();
			PX4_INFO("Test mode disabled - returning to normal operation");
			return 0;
		}

		return 1;
	}

	return print_usage("unknown command");
}

int LoadLampController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Load lamp controller for the wheel loader rear board that responds to load lamp commands.

The module subscribes to load_lamp_command topic sent from the main board (X7+)
which aggregates motor load data from both front and rear boards via uORB proxy.
The main board processes all hbridge_status data and sends appropriate lamp commands.

Controls two lamps (PA4, PC1) with PC0 as ground based on received commands.

Load levels and blink rates:
- LOAD_VERY_LOW (0-10%): Very slow blink (0.5 Hz)
- LOAD_LOW (10-20%): Slow blink (1 Hz)
- LOAD_MED_LOW (20-30%): Medium-slow blink (2 Hz)
- LOAD_MEDIUM (30-60%): Medium blink (5 Hz)
- LOAD_HIGH (60-80%): Fast blink (10 Hz)
- LOAD_VERY_HIGH (80-100%): Very fast blink (20 Hz)
- LOAD_OFF: Turn off lamps

### Architecture
Main Board (X7+) → uORB Proxy → Front/Rear Boards
  ↓ (aggregates hbridge_status from both boards)
Load Analysis & Command Generation
  ↓ (sends load_lamp_command to rear board)
Rear Board LoadLampController → Physical Lamps

### Examples
To start the module:
$ load_lamp_controller start

To test with different load levels:
$ load_lamp_controller test 0.1    # Test low load (slow blink)
$ load_lamp_controller test 0.5    # Test medium load (medium blink)
$ load_lamp_controller test 0.9    # Test high load (very fast blink)

To return to normal operation:
$ load_lamp_controller normal      # Disable test mode

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("load_lamp_controller", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test <load>", "Test with motor load value (0.0-1.0)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("normal", "Return to normal operation");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int LoadLampController::run_trampoline(int argc, char *argv[])
{
	LoadLampController *instance = get_instance();

	if (instance) {
		instance->run();
	}

	return 0;
}

extern "C" __EXPORT int load_lamp_controller_main(int argc, char *argv[])
{
	return LoadLampController::main(argc, argv);
}
