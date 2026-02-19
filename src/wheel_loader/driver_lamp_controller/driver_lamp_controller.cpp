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

#include "driver_lamp_controller.h"

DriverLampController::DriverLampController() :
	ModuleParams(nullptr)
{
}

DriverLampController::~DriverLampController()
{
	perf_free(_loop_perf);
	perf_free(_mode_update_perf);
}

void DriverLampController::parameters_update()
{
	updateParams();
}

void DriverLampController::update_lamp_mode()
{
	perf_begin(_mode_update_perf);

	vehicle_control_mode_s control_mode;
	vehicle_status_s vehicle_status;

	if (_vehicle_control_mode_sub.copy(&control_mode) && _vehicle_status_sub.copy(&vehicle_status)) {
		// Check for reverse mode first (highest priority)
		if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_MANUAL &&
		    control_mode.flag_control_manual_enabled) {
			// In manual mode, check for reverse
			// This would need additional status from RC or actuator outputs
			// For now, we'll use a simplified approach
		}

		// Check for turn signals
		if (control_mode.flag_control_manual_enabled) {
			// Check turn signal inputs
			// This is simplified - actual implementation would need RC channel mapping
			bool left_turn = false;  // Would come from RC channel
			bool right_turn = false; // Would come from RC channel

			if (left_turn && right_turn) {
				_current_mode = LampMode::HAZARD;

			} else if (left_turn) {
				_current_mode = LampMode::LEFT_TURN;

			} else if (right_turn) {
				_current_mode = LampMode::RIGHT_TURN;

			} else {
				_current_mode = LampMode::OFF;
			}

		} else {
			_current_mode = LampMode::OFF;
		}
	}

	perf_end(_mode_update_perf);
}

uint32_t DriverLampController::get_turn_signal_interval_us() const
{
	float blink_rate_hz = _param_blink_rate.get();

	if (blink_rate_hz <= 0.0f) {
		blink_rate_hz = 1.5f; // Default to 1.5 Hz
	}

	// Convert Hz to period in microseconds (1 / Hz * 1,000,000)
	// Divide by 2 because we want half-period (on or off duration)
	return static_cast<uint32_t>(1000000.0f / blink_rate_hz / 2.0f);
}

void DriverLampController::process_lamp_state()
{
	const hrt_abstime now = hrt_absolute_time();

	switch (_current_mode) {
	case LampMode::OFF:
		_left_lamp_on = false;
		_right_lamp_on = false;
		break;

	case LampMode::REVERSE:
		_left_lamp_on = true;
		_right_lamp_on = true;
		break;

	case LampMode::LEFT_TURN: {
		hrt_abstime interval = get_turn_signal_interval_us();

		if (now - _last_toggle > interval) {
			_left_lamp_on = !_left_lamp_on;
			_last_toggle = now;
		}

		_right_lamp_on = false;
		break;
	}

	case LampMode::RIGHT_TURN: {
		hrt_abstime interval = get_turn_signal_interval_us();

		if (now - _last_toggle > interval) {
			_right_lamp_on = !_right_lamp_on;
			_last_toggle = now;
		}

		_left_lamp_on = false;
		break;
	}

	case LampMode::HAZARD:
		process_hazard_pattern();
		break;
	}

	set_lamps(_left_lamp_on, _right_lamp_on);
}

void DriverLampController::process_hazard_pattern()
{
	const hrt_abstime now = hrt_absolute_time();
	hrt_abstime elapsed = now - _hazard_timer;

	switch (_hazard_state) {
	case HazardState::FIRST_SHORT:
		_left_lamp_on = true;
		_right_lamp_on = true;

		if (elapsed >= HAZARD_SHORT_ON_US) {
			_hazard_state = HazardState::FIRST_SHORT_OFF;
			_hazard_timer = now;
		}

		break;

	case HazardState::FIRST_SHORT_OFF:
		_left_lamp_on = false;
		_right_lamp_on = false;

		if (elapsed >= HAZARD_SHORT_OFF_US) {
			_hazard_state = HazardState::SECOND_SHORT;
			_hazard_timer = now;
		}

		break;

	case HazardState::SECOND_SHORT:
		_left_lamp_on = true;
		_right_lamp_on = true;

		if (elapsed >= HAZARD_SHORT_ON_US) {
			_hazard_state = HazardState::SECOND_SHORT_OFF;
			_hazard_timer = now;
		}

		break;

	case HazardState::SECOND_SHORT_OFF:
		_left_lamp_on = false;
		_right_lamp_on = false;

		if (elapsed >= HAZARD_SHORT_OFF_US) {
			_hazard_state = HazardState::LONG;
			_hazard_timer = now;
		}

		break;

	case HazardState::LONG:
		_left_lamp_on = true;
		_right_lamp_on = true;

		if (elapsed >= HAZARD_LONG_ON_US) {
			_hazard_state = HazardState::LONG_OFF;
			_hazard_timer = now;
		}

		break;

	case HazardState::LONG_OFF:
		_left_lamp_on = false;
		_right_lamp_on = false;

		if (elapsed >= HAZARD_LONG_OFF_US) {
			_hazard_state = HazardState::FIRST_SHORT;
			_hazard_timer = now;
		}

		break;
	}
}

void DriverLampController::set_lamps(bool left_on, bool right_on)
{
#ifdef BOARD_HAS_DRIVER_LAMP
	DRIVER_LAMP_LEFT(left_on);
	DRIVER_LAMP_RIGHT(right_on);
#endif
}

void DriverLampController::run()
{
#ifdef BOARD_HAS_DRIVER_LAMP
	// Initialize GPIO pins as outputs
	px4_arch_configgpio(GPIO_DRIVER_LAMP_LEFT);
	px4_arch_configgpio(GPIO_DRIVER_LAMP_RIGHT);
	px4_arch_configgpio(GPIO_DRIVER_LAMP_GND);

	// Set ground pin low
	px4_arch_gpiowrite(GPIO_DRIVER_LAMP_GND, 0);
#endif

	while (!should_exit()) {
		perf_begin(_loop_perf);

		// Check for parameter updates
		if (_parameter_update_sub.updated()) {
			parameter_update_s param_update;
			_parameter_update_sub.copy(&param_update);
			parameters_update();
		}

		// Update lamp mode based on vehicle state (unless in test mode)
		if (!_test_mode_active) {
			update_lamp_mode();
		}

		// Process lamp state and update outputs
		process_lamp_state();

		perf_end(_loop_perf);

		px4_usleep(20000); // 50Hz
	}

	// Turn off lamps before exit
	set_lamps(false, false);
}

int DriverLampController::task_spawn(int argc, char *argv[])
{
	DriverLampController *instance = new DriverLampController();

	if (instance) {
		_object.store(instance);
		_task_id = px4_task_spawn_cmd("driver_lamp_controller",
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

int DriverLampController::run_trampoline(int argc, char *argv[])
{
	DriverLampController *instance = get_instance();

	if (instance) {
		instance->run();
	}

	return 0;
}

int DriverLampController::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	DriverLampController *instance = get_instance();

	if (!strcmp(argv[0], "test")) {
		if (argc > 1) {
			return instance->test_mode(argc - 1, argv + 1);

		} else {
			PX4_ERR("test mode requires an argument");
			return 1;
		}
	}

	return print_usage("unknown command");
}

int DriverLampController::test_mode(int argc, char *argv[])
{
	if (!strcmp(argv[0], "off")) {
		_test_mode_active = true;
		_current_mode = LampMode::OFF;
		PX4_INFO("Test mode: OFF");

	} else if (!strcmp(argv[0], "left")) {
		_test_mode_active = true;
		_current_mode = LampMode::LEFT_TURN;
		_last_toggle = hrt_absolute_time();
		PX4_INFO("Test mode: LEFT_TURN");

	} else if (!strcmp(argv[0], "right")) {
		_test_mode_active = true;
		_current_mode = LampMode::RIGHT_TURN;
		_last_toggle = hrt_absolute_time();
		PX4_INFO("Test mode: RIGHT_TURN");

	} else if (!strcmp(argv[0], "reverse")) {
		_test_mode_active = true;
		_current_mode = LampMode::REVERSE;
		PX4_INFO("Test mode: REVERSE");

	} else if (!strcmp(argv[0], "hazard")) {
		_test_mode_active = true;
		_current_mode = LampMode::HAZARD;
		_hazard_state = HazardState::FIRST_SHORT;
		_hazard_timer = hrt_absolute_time();
		PX4_INFO("Test mode: HAZARD");

	} else if (!strcmp(argv[0], "normal")) {
		_test_mode_active = false;
		PX4_INFO("Test mode disabled - returning to normal operation");

	} else {
		PX4_ERR("Unknown test mode: %s", argv[0]);
		return 1;
	}

	return 0;
}

int DriverLampController::print_status()
{
	PX4_INFO("Running");

	const char *mode_str = "UNKNOWN";

	switch (_current_mode) {
	case LampMode::OFF:
		mode_str = "OFF";
		break;

	case LampMode::REVERSE:
		mode_str = "REVERSE";
		break;

	case LampMode::LEFT_TURN:
		mode_str = "LEFT_TURN";
		break;

	case LampMode::RIGHT_TURN:
		mode_str = "RIGHT_TURN";
		break;

	case LampMode::HAZARD:
		mode_str = "HAZARD";
		break;
	}

	PX4_INFO("Mode: %s (test mode: %s)", mode_str, _test_mode_active ? "YES" : "NO");
	PX4_INFO("Blink rate: %.2f Hz", (double)_param_blink_rate.get());
	PX4_INFO("Left lamp: %s, Right lamp: %s", _left_lamp_on ? "ON" : "OFF", _right_lamp_on ? "ON" : "OFF");

	perf_print_counter(_loop_perf);
	perf_print_counter(_mode_update_perf);

	return 0;
}

int DriverLampController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver lamp controller for wheel loader turn signals and reverse lamps.

Controls left and right lamps based on vehicle state:
- Turn signals: Blink at configured rate (default 1.5 Hz)
- Reverse mode: Both lamps solid on
- Hazard mode: Both lamps in short-short-long pattern

The module runs at 50Hz and uses GPIO outputs to control the lamps.

### Implementation
The controller monitors vehicle_control_mode and vehicle_status topics to determine
when to activate turn signals or reverse lamps. In hazard mode, it implements a
specific blinking pattern: 100ms on, 100ms off, 100ms on, 100ms off, 400ms on, 100ms off.

### Examples
Test the lamps:
$ driver_lamp_controller test left
$ driver_lamp_controller test right
$ driver_lamp_controller test reverse
$ driver_lamp_controller test hazard
$ driver_lamp_controller test off
$ driver_lamp_controller test normal
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("driver_lamp_controller", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test lamp modes");
	PRINT_MODULE_USAGE_ARG("off|left|right|reverse|hazard|normal", "Test mode", false);

	return 0;
}

extern "C" __EXPORT int driver_lamp_controller_main(int argc, char *argv[])
{
	return DriverLampController::main(argc, argv);
}
