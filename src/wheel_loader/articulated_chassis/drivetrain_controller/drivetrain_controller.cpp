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

#include "drivetrain_controller.hpp"

#include <cinttypes>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

DrivetrainController::DrivetrainController() :
	ModuleBase(),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_speed_controller(),
	_speed_filter(CONTROL_DT, DEFAULT_FILTER_FREQ)
{
}

DrivetrainController::~DrivetrainController()
{
	perf_free(_loop_perf);
	perf_free(_control_perf);
}

bool DrivetrainController::init()
{
	// Update parameters first to get instance information
	updateParams();

	// Validate parameter ranges
	uint8_t encoder_instance = static_cast<uint8_t>(_param_encoder_id.get());
	uint8_t motor_channel = static_cast<uint8_t>(_param_motor_channel.get());

	if (encoder_instance >= ORB_MULTI_MAX_INSTANCES) {
		PX4_ERR("Invalid encoder instance %d (max: %d)", encoder_instance, ORB_MULTI_MAX_INSTANCES - 1);
		return false;
	}

	if (motor_channel >= ORB_MULTI_MAX_INSTANCES) {
		PX4_ERR("Invalid motor channel %d (max: %d)", motor_channel, ORB_MULTI_MAX_INSTANCES - 1);
		return false;
	}

	// Subscribe to required topics
	// Determine which wheel setpoint instance to subscribe to
	uint8_t wheel_instance = (_param_is_front_wheel.get() == 1) ? 0 : 1; // Front=0, Rear=1
	_drivetrain_setpoint_sub = uORB::Subscription{ORB_ID(drivetrain_setpoint), wheel_instance};

	if (!_drivetrain_setpoint_sub.subscribe()) {
		PX4_ERR("Failed to subscribe to drivetrain_setpoint instance %d", wheel_instance);
		return false;
	}

	if (!_param_update_sub.subscribe()) {
		PX4_ERR("Failed to subscribe to parameter_update");
		return false;
	}

	// Initialize performance counters
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	_control_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": control");

	if (!_loop_perf || !_control_perf) {
		PX4_ERR("Failed to allocate performance counters");
		return false;
	}

	// Configure PID controller with validated parameters
	float p_gain = _param_speed_p.get();
	float i_gain = _param_speed_i.get();
	float d_gain = _param_speed_d.get();
	float i_max = _param_integrator_max.get();

	if (p_gain < 0.0f || i_gain < 0.0f || d_gain < 0.0f || i_max <= 0.0f) {
		PX4_ERR("Invalid PID parameters: P=%f I=%f D=%f I_max=%f",
			(double)p_gain, (double)i_gain, (double)d_gain, (double)i_max);
		return false;
	}

	_speed_controller.setGains(p_gain, i_gain, d_gain);
	_speed_controller.setIntegralLimit(i_max);
	_speed_controller.setOutputLimit(MAX_PWM_VALUE);

	// Initialize speed filter with validated frequency
	float filter_freq = _param_filter_freq.get();
	if (filter_freq <= 0.0f || filter_freq > 100.0f) {
		PX4_ERR("Invalid filter frequency: %f Hz", (double)filter_freq);
		return false;
	}
	_speed_filter.set_cutoff_frequency(CONTROL_DT, filter_freq);

	// Initialize state
	_state = {}; // Reset all state to zero/false
	_state.initialized = true;

	// Start the work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	PX4_INFO("Wheel controller initialized - Encoder: %d, Motor: %d, Front: %s",
		 encoder_instance, motor_channel,
		 (_param_is_front_wheel.get() == 1) ? "YES" : "NO");

	return true;
}

void DrivetrainController::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	perf_begin(_loop_perf);

	// Check for parameter updates
	if (_param_update_sub.updated()) {
		parameter_update_s param_update;
		_param_update_sub.copy(&param_update);
		parameters_update();
	}

	// Update sensor data
	update_encoder_feedback();
	update_hbridge_status();

	// Safety checks first
	check_safety_conditions();

	if (!_state.emergency_stop) {
		// Update setpoint and run control
		if (update_speed_setpoint()) {
			perf_begin(_control_perf);
			run_speed_controller();
			perf_end(_control_perf);
		} else {
			// No valid setpoint - stop motor
			_state.setpoint_rad_s = 0.0f;
			_state.pwm_output = 0.0f;
		}

	} else {
		// Emergency stop active
		_state.setpoint_rad_s = 0.0f;
		_state.pwm_output = 0.0f;
	}

	// Publish motor command
	publish_motor_command();

	perf_end(_loop_perf);
}

bool DrivetrainController::update_speed_setpoint()
{
	drivetrain_setpoint_s setpoint;

	if (_drivetrain_setpoint_sub.update(&setpoint)) {
		// Check if this wheel controller should respond to this setpoint
		// (based on instance ID or wheel identification)

		// Use the wheel speed setpoint
		_state.setpoint_rad_s = math::constrain(setpoint.wheel_speed_rad_s,
							-_param_max_speed.get(),
							_param_max_speed.get());
		_state.last_setpoint_us = hrt_absolute_time();

		// Handle emergency stop
		if (setpoint.emergency_stop) {
			_state.emergency_stop = true;
			_state.setpoint_rad_s = 0.0f;
		} else {
			_state.emergency_stop = false;
		}

		return true;
	}

	return is_setpoint_valid();
}void DrivetrainController::update_encoder_feedback()
{
	sensor_quad_encoder_s encoder;
	uint8_t encoder_instance = static_cast<uint8_t>(_param_encoder_id.get());

	// Check if we have a valid encoder instance ID
	if (encoder_instance >= ORB_MULTI_MAX_INSTANCES) {
		PX4_ERR("Invalid encoder instance: %d", encoder_instance);
		return;
	}

	if (_encoder_sub[encoder_instance].updated() &&
		_encoder_sub[encoder_instance].copy(&encoder)) {
		// Verify encoder data is from correct instance and is valid
		if (encoder.instance == encoder_instance &&
			encoder.timestamp > _state.last_encoder_us) {
			// Convert encoder velocity to rad/s
			// Note: velocity field should already be in rad/s from quadrature encoder driver
			float raw_speed = encoder.velocity;

			// Apply low-pass filtering to reduce noise
			_state.speed_rad_s = _speed_filter.apply(raw_speed);
			_state.last_encoder_us = encoder.timestamp;
		}
	}
}

void DrivetrainController::run_speed_controller()
{
	// Calculate speed error
	float speed_error = _state.setpoint_rad_s - _state.speed_rad_s;

	// Run PID controller
	float pid_output = _speed_controller.update(speed_error, CONTROL_DT);

	// Apply output constraints and deadband
	_state.pwm_output = constrain_pwm(pid_output);

	// Apply deadband for small setpoints to prevent dither
	static constexpr float SPEED_DEADBAND = 0.1f; // rad/s
	if (fabsf(_state.setpoint_rad_s) < SPEED_DEADBAND) {
		_state.pwm_output = 0.0f;
	}
}

void DrivetrainController::publish_motor_command()
{
	hbridge_setpoint_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.instance = static_cast<uint8_t>(_param_motor_channel.get());
	cmd.duty_cycle = _state.pwm_output;
	cmd.enable = _state.motor_enabled && !_state.emergency_stop;

	// Publish using PublicationMulti (no instance parameter needed)
	_motor_cmd_pub.publish(cmd);
}

void DrivetrainController::update_hbridge_status()
{
	hbridge_status_s status;
	uint8_t motor_channel = static_cast<uint8_t>(_param_motor_channel.get());

	// Check if we have a valid motor channel ID
	if (motor_channel >= ORB_MULTI_MAX_INSTANCES) {
		PX4_ERR("Invalid motor channel: %d", motor_channel);
		return;
	}

	if (_hbridge_status_sub[motor_channel].updated() &&
		_hbridge_status_sub[motor_channel].copy(&status)) {
		// Verify status data is from correct instance
		if (status.instance == motor_channel) {
			_state.motor_enabled = status.enabled;

			// Check for limit sensor activation which could indicate obstacles or faults
			if (status.forward_limit || status.reverse_limit) {
				PX4_WARN("Motor channel %d limit sensor active - F:%d R:%d",
					motor_channel, status.forward_limit, status.reverse_limit);

				// Could implement more sophisticated limit handling here
				// For now, just warn - the H-bridge driver handles the actual limiting
			}
		}
	}
}

void DrivetrainController::check_safety_conditions()
{
	const uint64_t now = hrt_absolute_time();
	bool previous_emergency_state = _state.emergency_stop;
	_state.emergency_stop = false;

	// Check setpoint timeout
	if (!is_setpoint_valid()) {
		if (!previous_emergency_state) {
			PX4_WARN("Setpoint timeout - stopping motor");
		}
		_state.emergency_stop = true;
	}

	// Check encoder timeout
	if (now - _state.last_encoder_us > ENCODER_TIMEOUT_US) {
		if (!previous_emergency_state) {
			PX4_WARN("Encoder timeout - stopping motor");
		}
		_state.emergency_stop = true;
	}

	// Check if motor is enabled via H-bridge status
	if (!_state.motor_enabled) {
		if (!previous_emergency_state) {
			PX4_WARN("Motor disabled by H-bridge - stopping control");
		}
		_state.emergency_stop = true;
	}

	// Log when recovering from emergency stop
	if (previous_emergency_state && !_state.emergency_stop) {
		PX4_INFO("Recovering from emergency stop");
	}
}

void DrivetrainController::parameters_update()
{
	updateParams();

	// Validate and update PID gains
	float p_gain = _param_speed_p.get();
	float i_gain = _param_speed_i.get();
	float d_gain = _param_speed_d.get();
	float i_max = _param_integrator_max.get();

	if (p_gain >= 0.0f && i_gain >= 0.0f && d_gain >= 0.0f && i_max > 0.0f) {
		_speed_controller.setGains(p_gain, i_gain, d_gain);
		_speed_controller.setIntegralLimit(i_max);
	} else {
		PX4_ERR("Invalid PID parameters during update - keeping current values");
	}

	// Validate and update filter frequency
	float filter_freq = _param_filter_freq.get();
	if (filter_freq > 0.0f && filter_freq <= 100.0f) {
		_speed_filter.set_cutoff_frequency(CONTROL_DT, filter_freq);
	} else {
		PX4_ERR("Invalid filter frequency during update - keeping current value");
	}
}

float DrivetrainController::constrain_pwm(float value) const
{
	return math::constrain(value, MIN_PWM_VALUE, MAX_PWM_VALUE);
}

bool DrivetrainController::is_setpoint_valid() const
{
	const uint64_t timeout_us = static_cast<uint64_t>(_param_setpoint_timeout.get() * 1e6f);
	return (hrt_absolute_time() - _state.last_setpoint_us) < timeout_us;
}

int DrivetrainController::print_status()
{
	PX4_INFO("=== Wheel Controller Status ===");
	PX4_INFO("Configuration:");
	PX4_INFO("  Encoder ID: %ld, Motor Channel: %ld",
		 _param_encoder_id.get(), _param_motor_channel.get());
	PX4_INFO("  Wheel Type: %s", (_param_is_front_wheel.get() == 1) ? "Front" : "Rear");
	PX4_INFO("  Max Speed: %.1f rad/s", (double)_param_max_speed.get());

	PX4_INFO("Control State:");
	PX4_INFO("  Speed: %.2f rad/s (target: %.2f rad/s)",
		 (double)_state.speed_rad_s, (double)_state.setpoint_rad_s);
	PX4_INFO("  PWM Output: %.3f", (double)_state.pwm_output);
	PX4_INFO("  Speed Error: %.2f rad/s",
		 (double)(_state.setpoint_rad_s - _state.speed_rad_s));

	PX4_INFO("System Status:");
	PX4_INFO("  Initialized: %s", _state.initialized ? "YES" : "NO");
	PX4_INFO("  Motor Enabled: %s", _state.motor_enabled ? "YES" : "NO");
	PX4_INFO("  Emergency Stop: %s", _state.emergency_stop ? "YES" : "NO");
	PX4_INFO("  Setpoint Valid: %s", is_setpoint_valid() ? "YES" : "NO");

	PX4_INFO("PID Configuration:");
	PX4_INFO("  P: %.3f, I: %.3f, D: %.3f",
		 (double)_param_speed_p.get(),
		 (double)_param_speed_i.get(),
		 (double)_param_speed_d.get());
	PX4_INFO("  I_max: %.1f, Filter: %.1f Hz",
		 (double)_param_integrator_max.get(),
		 (double)_param_filter_freq.get());

	PX4_INFO("Timing:");
	uint64_t now = hrt_absolute_time();
	PX4_INFO("  Last Setpoint: %llu us ago", now - _state.last_setpoint_us);
	PX4_INFO("  Last Encoder: %llu us ago", now - _state.last_encoder_us);

	if (_loop_perf && _control_perf) {
		PX4_INFO("Performance:");
		perf_print_counter(_loop_perf);
		perf_print_counter(_control_perf);
	}

	return 0;
}

int DrivetrainController::task_spawn(int argc, char *argv[])
{
	DrivetrainController *instance = new DrivetrainController();

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	if (!instance->init()) {
		delete instance;
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	return PX4_OK;
}

int DrivetrainController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Individual wheel controller with PID speed control using quadrature encoder feedback.
Provides precise speed control for articulated chassis wheel systems.

### Implementation
This module implements:
- PID-based speed control with encoder feedback
- Motor output through H-bridge interface
- Emergency stop and safety monitoring
- Comprehensive status reporting and diagnostics

### Examples
Start the wheel controller for front wheel:
$ wheel_controller start -e 0 -m 0 -f

Check status:
$ wheel_controller status

Set target speed to 5.0 rad/s:
$ wheel_controller speed 5.0

Stop the motor:
$ wheel_controller stop

Tune PID gains:
$ wheel_controller tune_p 1.5
$ wheel_controller tune_i 0.1
$ wheel_controller tune_d 0.05
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wheel_controller", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the wheel controller");
	PRINT_MODULE_USAGE_PARAM_INT('e', 0, 0, 7, "Encoder instance", true);
	PRINT_MODULE_USAGE_PARAM_INT('m', 0, 0, 1, "Motor channel (0 or 1)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Front wheel (default: rear)", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the wheel controller");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status information");
	PRINT_MODULE_USAGE_COMMAND_DESCR("speed", "Set target speed (rad/s)");
	PRINT_MODULE_USAGE_ARG("<speed>", "Target speed in rad/s", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("emergency_stop", "Trigger emergency stop");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset_emergency", "Reset emergency stop");
	PRINT_MODULE_USAGE_COMMAND_DESCR("enable", "Enable motor output");
	PRINT_MODULE_USAGE_COMMAND_DESCR("disable", "Disable motor output");
	PRINT_MODULE_USAGE_COMMAND_DESCR("tune_p", "Set proportional gain");
	PRINT_MODULE_USAGE_ARG("<gain>", "P gain value", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("tune_i", "Set integral gain");
	PRINT_MODULE_USAGE_ARG("<gain>", "I gain value", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("tune_d", "Set derivative gain");
	PRINT_MODULE_USAGE_ARG("<gain>", "D gain value", false);

	return 0;
}

void DrivetrainController::set_speed_setpoint(float speed_rad_s)
{
	_state.setpoint_rad_s = math::constrain(speed_rad_s, -_param_max_speed.get(), _param_max_speed.get());
	_state.last_setpoint_us = hrt_absolute_time();
}

int DrivetrainController::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	const char *command = argv[0];

	// Speed control command
	if (strcmp(command, "speed") == 0) {
		if (argc < 2) {
			PX4_ERR("speed command requires speed value (rad/s)");
			return -1;
		}

		float speed = static_cast<float>(atof(argv[1]));
		DrivetrainController *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("drivetrain_controller is not running");
			return -1;
		}

		instance->set_speed_setpoint(speed);
		PX4_INFO("Set target speed to %.2f rad/s", (double)speed);
		return 0;
	}

	// Emergency stop commands
	if (strcmp(command, "emergency_stop") == 0) {
		DrivetrainController *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("drivetrain_controller is not running");
			return -1;
		}

		instance->_state.emergency_stop = true;
		PX4_WARN("Emergency stop activated");
		return 0;
	}

	if (strcmp(command, "reset_emergency") == 0) {
		DrivetrainController *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("drivetrain_controller is not running");
			return -1;
		}

		instance->_state.emergency_stop = false;
		PX4_INFO("Emergency stop cleared");
		return 0;
	}

	// Motor enable/disable commands
	if (strcmp(command, "enable") == 0) {
		DrivetrainController *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("drivetrain_controller is not running");
			return -1;
		}

		instance->_state.motor_enabled = true;
		PX4_INFO("Motor enabled");
		return 0;
	}

	if (strcmp(command, "disable") == 0) {
		DrivetrainController *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("drivetrain_controller is not running");
			return -1;
		}

		instance->_state.motor_enabled = false;
		PX4_INFO("Motor disabled");
		return 0;
	}

	// PID tuning commands
	if (strcmp(command, "tune_p") == 0) {
		if (argc < 2) {
			PX4_ERR("tune_p requires a value");
			return -1;
		}

		DrivetrainController *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("drivetrain_controller is not running");
			return -1;
		}

		float p = static_cast<float>(atof(argv[1]));
		instance->_speed_controller.setGains(p, instance->_param_speed_i.get(), instance->_param_speed_d.get());
		PX4_INFO("Set P gain to %.3f", (double)p);
		return 0;
	}

	if (strcmp(command, "tune_i") == 0) {
		if (argc < 2) {
			PX4_ERR("tune_i requires a value");
			return -1;
		}

		DrivetrainController *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("drivetrain_controller is not running");
			return -1;
		}

		float i = static_cast<float>(atof(argv[1]));
		instance->_speed_controller.setGains(instance->_param_speed_p.get(), i, instance->_param_speed_d.get());
		PX4_INFO("Set I gain to %.3f", (double)i);
		return 0;
	}

	if (strcmp(command, "tune_d") == 0) {
		if (argc < 2) {
			PX4_ERR("tune_d requires a value");
			return -1;
		}

		DrivetrainController *instance = get_instance();

		if (instance == nullptr) {
			PX4_ERR("drivetrain_controller is not running");
			return -1;
		}

		float d = static_cast<float>(atof(argv[1]));
		instance->_speed_controller.setGains(instance->_param_speed_p.get(), instance->_param_speed_i.get(), d);
		PX4_INFO("Set D gain to %.3f", (double)d);
		return 0;
	}

	return print_usage("unknown command");
}
