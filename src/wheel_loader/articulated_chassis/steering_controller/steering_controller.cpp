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
 * @file steering_controller.cpp
 * Steering controller implementation for articulated wheel loader
 *
 * @author PX4 Development Team
 */

#include "steering_controller.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

#define MODULE_NAME "steering_controller"

SteeringController::SteeringController() :
	ModuleBase(),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

bool SteeringController::init()
{
	// Update parameters from configuration
	updateParams();

	// Schedule at configured rate
	ScheduleOnInterval(CONTROL_INTERVAL_US);

	PX4_INFO("Steering Controller initialized");
	PX4_INFO("Max angle: ±%.1f°, ST3125 Servo ID: %d",
		 (double)math::degrees(_max_steering_angle.get()),
		 (int)_st3125_servo_id.get());

	return true;
}

void SteeringController::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Process core functions
	process_steering_command();
	process_servo_feedback();
	process_limit_sensors();
	handle_abnormal_events();

	// Publish steering status
	publish_steering_status();
}

void SteeringController::process_steering_command()
{
	steering_setpoint_s setpoint;

	if (_steering_setpoint_sub.update(&setpoint)) {
		_last_command_time = hrt_absolute_time();

		// Validate setpoint
		if (!PX4_ISFINITE(setpoint.steering_angle_rad)) {
			PX4_WARN("Invalid steering setpoint: non-finite angle");
			return;
		}

		// Get target angle and constrain to limits
		_target_angle_rad = saturate_angle(setpoint.steering_angle_rad);

		// Check if command is safe based on limit sensors
		bool safe_to_move = true;

		if (_target_angle_rad < 0.0f && _left_limit_active) {
			safe_to_move = false;  // Don't move left if left limit is active
			PX4_DEBUG("Left limit active, blocking leftward movement");
		}

		if (_target_angle_rad > 0.0f && _right_limit_active) {
			safe_to_move = false;  // Don't move right if right limit is active
			PX4_DEBUG("Right limit active, blocking rightward movement");
		}

		if (safe_to_move && _servo_healthy && !_emergency_stop) {
			send_servo_command(_target_angle_rad);

		} else {
			// Emergency: command to center position
			if (!_servo_healthy) {
				PX4_DEBUG("Servo unhealthy, commanding center position");
			}

			if (_emergency_stop) {
				PX4_DEBUG("Emergency stop active, commanding center position");
			}

			send_servo_command(0.0f);
		}
	}
}

void SteeringController::send_servo_command(float position_rad)
{
	// Validate position input
	if (!PX4_ISFINITE(position_rad)) {
		PX4_ERR("Invalid servo position command: non-finite value");
		return;
	}

	robotic_servo_setpoint_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.id = _st3125_servo_id.get();
	cmd.command_type = 0; // position control
	cmd.goal_position = position_rad;
	cmd.goal_velocity = 0.0f; // Let ST3125 handle velocity
	cmd.goal_current = _st3125_current_limit.get();
	cmd.torque_enable = true;

	_servo_command_pub.publish(cmd);
}

void SteeringController::process_servo_feedback()
{
	robotic_servo_status_s feedback;

	if (_servo_feedback_sub.update(&feedback)) {
		if (feedback.id == _st3125_servo_id.get()) {
			_last_feedback_time = hrt_absolute_time();

			// Update servo state
			_current_angle_rad = feedback.position;
			_servo_error_flags = feedback.error_flags;
			_servo_current_a = feedback.current;
			_servo_temperature_c = feedback.temperature;

			// Check servo health
			_servo_healthy = (feedback.error_flags == 0) && feedback.torque_enabled;
		}
	}
}

void SteeringController::process_limit_sensors()
{
	sensor_limit_switch_s limit_msg;

	// Check left limit sensor for this instance
	uint8_t left_limit_id = get_left_limit();
	if (left_limit_id != 255 && _limit_sensor_sub[left_limit_id].updated() && _limit_sensor_sub[left_limit_id].copy(&limit_msg)) {
		_left_limit_active = limit_msg.state;
		_limit_sensors_healthy = !limit_msg.redundancy_fault;
	}

	// Check right limit sensor for this instance
	uint8_t right_limit_id = get_right_limit();
	if (right_limit_id != 255 && _limit_sensor_sub[right_limit_id].updated() && _limit_sensor_sub[right_limit_id].copy(&limit_msg)) {
		_right_limit_active = limit_msg.state;
		_limit_sensors_healthy = !limit_msg.redundancy_fault;
	}
}

void SteeringController::handle_abnormal_events()
{
	bool has_fault = false;

	// Check for command timeout
	if (is_command_timeout()) {
		if (_last_command_time > 0) {  // Only warn if we've received at least one command
			PX4_WARN("Steering command timeout - returning to center");
		}

		_target_angle_rad = 0.0f;
		send_servo_command(0.0f);
		has_fault = true;
	}

	// Check for feedback timeout
	if (is_feedback_timeout()) {
		if (_last_feedback_time > 0) {  // Only warn if we've received at least one feedback
			PX4_WARN("Servo feedback timeout");
		}

		_servo_healthy = false;
		has_fault = true;
	}

	// Check servo error flags
	if (_servo_error_flags != 0) {
		PX4_WARN("Servo error flags: 0x%04x", _servo_error_flags);
		_servo_healthy = false;
		_emergency_stop = true;
		send_servo_command(0.0f); // Emergency stop - return to center
		has_fault = true;
	}

	// Check limit sensor faults
	if (!_limit_sensors_healthy) {
		PX4_WARN("Limit sensor fault detected");
		// Continue operation but with caution - this is not a critical fault
	}

	// Check for overcurrent (with hysteresis to avoid chattering)
	const float current_limit = _st3125_current_limit.get();

	if (_servo_current_a > current_limit * 1.1f) {  // 10% hysteresis
		PX4_WARN("Servo overcurrent: %.2fA > %.2fA", (double)_servo_current_a, (double)current_limit);
		_emergency_stop = true;
		send_servo_command(0.0f);
		has_fault = true;

	} else if (_emergency_stop && _servo_current_a < current_limit * 0.9f) {
		// Allow recovery when current drops below 90% of limit
		_emergency_stop = false;
		PX4_INFO("Overcurrent condition cleared");
	}

	// Check for overtemperature (with hysteresis)
	const float temp_limit = 80.0f; // ST3125 operating limit

	if (_servo_temperature_c > temp_limit + 5.0f) {  // 5°C hysteresis
		PX4_WARN("Servo overtemperature: %.1f°C > %.1f°C", (double)_servo_temperature_c, (double)temp_limit);
		_emergency_stop = true;
		send_servo_command(0.0f);
		has_fault = true;

	} else if (_emergency_stop && _servo_temperature_c < temp_limit) {
		// Allow recovery when temperature drops below limit
		_emergency_stop = false;
		PX4_INFO("Overtemperature condition cleared");
	}

	// If no faults, clear emergency stop (but keep it if manually set)
	if (!has_fault && _servo_healthy) {
		// Emergency stop may be cleared naturally when conditions improve
	}
}

void SteeringController::publish_steering_status()
{
	steering_status_s status{};
	status.timestamp = hrt_absolute_time();

	// Convert angles to degrees for legacy fields
	status.steering_angle_deg = math::degrees(_current_angle_rad);
	status.steering_angle_setpoint_deg = math::degrees(_target_angle_rad);

	// Calculate steering rate (basic differentiation)
	static float prev_angle_rad = 0.0f;
	static hrt_abstime prev_timestamp = 0;

	if (prev_timestamp > 0) {
		const float dt = (status.timestamp - prev_timestamp) / 1e6f; // Convert to seconds
		if (dt > 0.001f) { // Avoid division by very small numbers
			const float rate_rad_s = (_current_angle_rad - prev_angle_rad) / dt;
			status.steering_rate_deg_s = math::degrees(rate_rad_s);
			status.actual_rate_rad_s = rate_rad_s;
		}
	}
	prev_angle_rad = _current_angle_rad;
	prev_timestamp = status.timestamp;

	// Set current values in radians
	status.actual_angle_rad = _current_angle_rad;
	status.servo_position_rad = _current_angle_rad;

	// Servo status
	status.servo_healthy = _servo_healthy;
	status.steering_temperature_c = _servo_temperature_c;

	// System health
	status.is_healthy = _servo_healthy && _limit_sensors_healthy && !_emergency_stop;
	status.position_valid = _servo_healthy && !is_feedback_timeout();
	status.emergency_stop = _emergency_stop;
	status.emergency_stop_active = _emergency_stop; // Legacy field

	// Limit sensor status
	status.limit_left_active = _left_limit_active;
	status.limit_right_active = _right_limit_active;
	status.limit_sensors_healthy = _limit_sensors_healthy;

	// Safety status
	status.safety_violation = _emergency_stop || (!_limit_sensors_healthy);

	// Error flags and control mode
	status.error_flags = _servo_error_flags;
	status.control_mode = steering_status_s::CONTROL_MODE_POSITION; // We're always in position mode

	// Power steering (always active for ST3125)
	status.power_steering_active = true;

	// Torque information (not available from ST3125, set defaults)
	status.servo_torque_nm = 0.0f; // Not available from ST3125 feedback
	status.power_steering_pressure = 0.0f; // Not applicable for servo system
	status.max_torque_available = 50.0f; // ST3125 rated torque
	status.slip_compensation_deg = 0.0f; // Not implemented

	_steering_status_pub.publish(status);
}

float SteeringController::saturate_angle(float angle_rad)
{
	return math::constrain(angle_rad, -_max_steering_angle.get(), _max_steering_angle.get());
}

bool SteeringController::is_command_timeout()
{
	return (hrt_absolute_time() - _last_command_time) > (_command_timeout_ms.get() * 1000);
}

bool SteeringController::is_feedback_timeout()
{
	return (hrt_absolute_time() - _last_feedback_time) > (_feedback_timeout_ms.get() * 1000);
}

void SteeringController::update_parameters()
{
	// ModuleParams automatically handles the parameter updates
	updateParams();
}

int SteeringController::print_status()
{
	PX4_INFO("Steering Controller Status:");
	PX4_INFO("  Target angle: %.1f°", (double)math::degrees(_target_angle_rad));
	PX4_INFO("  Current angle: %.1f°", (double)math::degrees(_current_angle_rad));
	PX4_INFO("  Servo healthy: %s", _servo_healthy ? "YES" : "NO");
	PX4_INFO("  Error flags: 0x%04x", _servo_error_flags);
	PX4_INFO("  Current: %.2fA", (double)_servo_current_a);
	PX4_INFO("  Temperature: %.1f°C", (double)_servo_temperature_c);
	PX4_INFO("  Left limit: %s", _left_limit_active ? "ACTIVE" : "inactive");
	PX4_INFO("  Right limit: %s", _right_limit_active ? "ACTIVE" : "inactive");
	PX4_INFO("  Emergency stop: %s", _emergency_stop ? "ACTIVE" : "inactive");

	return 0;
}

int SteeringController::task_spawn(int argc, char *argv[])
{
	SteeringController *instance = new SteeringController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int SteeringController::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int SteeringController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Steering controller for articulated wheel loader.

Controls ST3125 servo-based steering with safety features:
- Position-based command processing
- Limit sensor integration for safety
- Servo health monitoring
- Emergency stop handling

### Implementation
The controller runs at 50Hz and sends position commands to the ST3125 servo.
The servo handles its own PID control internally.

### Examples
$ steering_controller start
$ steering_controller status
$ steering_controller stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("steering_controller", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
