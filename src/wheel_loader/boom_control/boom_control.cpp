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

#include "boom_control.hpp"

#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>
#include <math.h>
#include <cmath>

BoomControl::BoomControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_kinematics(this),
	_hardware_interface(this),
	_motion_controller(this),
	_state_manager(this),
	_cycle_perf(perf_alloc(PC_ELAPSED, "boom_control_cycle")),
	_control_latency_perf(perf_alloc(PC_ELAPSED, "boom_control_latency"))
{
}

BoomControl::~BoomControl()
{
	// Free performance counters
	perf_free(_cycle_perf);
	perf_free(_control_latency_perf);
}

bool BoomControl::init()
{
	// Check if module is enabled
	if (_param_enabled.get() == 0) {
		PX4_INFO("Boom control module disabled");
		return false;
	}

	// Initialize components
	if (!_hardware_interface.initialize(0, 0)) {
		PX4_ERR("Failed to initialize hardware interface");
		return false;
	}

	if (!_motion_controller.initialize()) {
		PX4_ERR("Failed to initialize motion controller");
		return false;
	}

	// Update configurations
	_kinematics.update_configuration();

	if (!_kinematics.validate_configuration()) {
		PX4_ERR("Invalid kinematic configuration");
		return false;
	}

	// Set initial state
	_state_manager.request_transition(BoomStateManager::OperationalState::IDLE);

	// Start periodic execution at configured rate
	float update_rate = _param_update_rate.get();

	// Validate update rate parameter
	if (update_rate <= 0.0f || update_rate > 200.0f) {
		PX4_WARN("Invalid update rate %.1f Hz, using default 50.0 Hz", (double)update_rate);
		update_rate = 50.0f;
	}

	uint32_t interval_us = static_cast<uint32_t>(1000000.0f / update_rate);
	interval_us = math::max(interval_us, CONTROL_INTERVAL_US);  // Minimum 50 Hz

	ScheduleOnInterval(interval_us);

	PX4_INFO("Boom control initialized (update rate: %.1f Hz)", (double)update_rate);
	return true;
}

void BoomControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// Main control pipeline
	update_sensors();
	process_commands();
	update_motion_planning();
	execute_control();
	publish_telemetry();

	perf_end(_cycle_perf);
}

void BoomControl::update_sensors()
{
	// Check for parameter updates
	parameter_update_s param_update;

	if (_parameter_update_sub.update(&param_update)) {
		update_parameters();
	}

	// Update sensor readings
	BoomHardwareInterface::SensorData sensor_data;

	if (_hardware_interface.update_sensors(sensor_data)) {
		// Calculate actuator length from sensor angle using kinematics
		_current_actuator_length = _kinematics.sensor_angle_to_actuator_length(sensor_data.calibrated_angle);
		_current_boom_angle = _kinematics.actuator_length_to_boom_angle(_current_actuator_length);
	}

	// Update H-bridge status
	BoomHardwareInterface::HbridgeStatus hbridge_status;
	_hardware_interface.update_status(hbridge_status);

	// Update state manager with system health
	_state_manager.update(
		_hardware_interface.is_healthy(),
		_hardware_interface.is_healthy(),
		false // at_target will be updated by motion controller
	);
}

void BoomControl::process_commands()
{
	boom_control_setpoint_s control_setpoint;

	if (_boom_control_setpoint_sub.update(&control_setpoint)) {
		_last_command_time = hrt_absolute_time();

		// Check if setpoint is valid
		if (!control_setpoint.valid) {
			// If setpoint is not valid, hold current position
			_motion_controller.set_mode(BoomMotionController::ControlMode::POSITION);
			_motion_controller.set_target_position(_current_boom_angle, 0.5f); // Conservative velocity
			return;
		}

		// Check if system is operational
		if (!_state_manager.is_operational()) {
			PX4_WARN("System not operational, ignoring control setpoint");
			return;
		}

		// Validate bucket height bounds
		if (!std::isfinite(control_setpoint.bucket_height) ||
		    control_setpoint.bucket_height < 0.0f ||
		    control_setpoint.bucket_height > 10.0f) {
			PX4_WARN("Invalid bucket height: %.2f m", (double)control_setpoint.bucket_height);
			return;
		}

		// Compute boom angle from bucket height using IK
		_target_boom_angle = computeBoomAngleFromHeight(control_setpoint.bucket_height);

		// Compute velocity limit from bucket height velocity (optional)
		float max_velocity = 1.0f; // Default safe velocity
		if (std::isfinite(control_setpoint.bucket_height_velocity) &&
		    control_setpoint.bucket_height_velocity > 0.0f) {
			// Convert height velocity to angular velocity estimate
			// angular_velocity ≈ height_velocity / (boom_length * cos(boom_angle))
			float boom_length = _param_boom_length.get();
			if (boom_length > 0.1f) {
				float cos_angle = cosf(_target_boom_angle);
				if (fabsf(cos_angle) > 0.1f) {
					max_velocity = fabsf(control_setpoint.bucket_height_velocity / (boom_length * cos_angle));
					max_velocity = math::constrain(max_velocity, 0.1f, 2.0f);
				}
			}
		}

		// Set target position with computed angle
		_motion_controller.set_mode(BoomMotionController::ControlMode::POSITION);
		_motion_controller.set_target_position(_target_boom_angle, max_velocity);
		_state_manager.request_transition(BoomStateManager::OperationalState::MOVING);
	}

	// Check for command timeout
	static constexpr hrt_abstime COMMAND_TIMEOUT_US = 1000000; // 1 second

	if ((hrt_absolute_time() - _last_command_time) > COMMAND_TIMEOUT_US) {
		if (_state_manager.get_state_info().state == BoomStateManager::OperationalState::MOVING) {
			_state_manager.request_transition(BoomStateManager::OperationalState::HOLDING);
		}
	}
}

float BoomControl::computeBoomAngleFromHeight(float bucket_height)
{
	// Inverse kinematics: bucket_height = pivot_height + boom_length * sin(boom_angle)
	// boom_angle = asin((bucket_height - pivot_height) / boom_length)

	const float pivot_height = _param_boom_pivot_height.get();
	const float boom_length = _param_boom_length.get();

	if (boom_length < 0.1f) {
		return 0.0f;  // Invalid configuration
	}

	float height_delta = bucket_height - pivot_height;
	float sin_angle = height_delta / boom_length;

	// Clamp to valid sine range
	sin_angle = math::constrain(sin_angle, -1.0f, 1.0f);

	float boom_angle = asinf(sin_angle);

	// Convert angle limits from degrees to radians
	float angle_min_rad = math::radians(_param_boom_angle_min.get());
	float angle_max_rad = math::radians(_param_boom_angle_max.get());

	// Apply joint limits
	return math::constrain(boom_angle, angle_min_rad, angle_max_rad);
}

void BoomControl::update_motion_planning()
{
	auto state_info = _state_manager.get_state_info();

	// Skip if in error or uninitialized state
	if (state_info.state == BoomStateManager::OperationalState::ERROR ||
	    state_info.state == BoomStateManager::OperationalState::UNINITIALIZED) {
		return;
	}

	// Update trajectory if moving
	if (state_info.state == BoomStateManager::OperationalState::MOVING) {
		float dt = static_cast<float>(CONTROL_INTERVAL_US) * 1e-6f;

		// Update trajectory planning
		auto setpoint = _motion_controller.update_trajectory(_current_boom_angle, dt);

		// Check if at target
		static constexpr float POSITION_TOLERANCE = 0.01f; // ~0.5 degrees
		static constexpr float VELOCITY_TOLERANCE = 0.001f;

		if (fabsf(setpoint.position - _current_boom_angle) < POSITION_TOLERANCE &&
		    fabsf(setpoint.velocity) < VELOCITY_TOLERANCE) {
			_state_manager.request_transition(BoomStateManager::OperationalState::HOLDING);
		}
	}
}

void BoomControl::execute_control()
{
	perf_begin(_control_latency_perf);

	auto state_info = _state_manager.get_state_info();

	// Handle emergency stop
	if (state_info.state == BoomStateManager::OperationalState::EMERGENCY_STOP) {
		_hardware_interface.emergency_stop();
		perf_end(_control_latency_perf);
		return;
	}

	// Skip control if not operational
	if (!_state_manager.is_operational()) {
		BoomHardwareInterface::HbridgeSetpoint cmd{};
		cmd.duty_cycle = 0.0f;
		cmd.enable = false;
		_hardware_interface.send_command(cmd);
		perf_end(_control_latency_perf);
		return;
	}

	// Get motion setpoint
	float dt = static_cast<float>(CONTROL_INTERVAL_US) * 1e-6f;
	auto setpoint = _motion_controller.update_trajectory(_current_boom_angle, dt);

	// Estimate current velocity (simple derivative)
	static float last_position = 0.0f;
	float current_velocity = (_current_boom_angle - last_position) / dt;
	last_position = _current_boom_angle;

	// Compute control output
	auto control_output = _motion_controller.compute_control(
				      setpoint,
				      _current_boom_angle,
				      current_velocity,
				      dt
			      );

	// Send to H-bridge
	BoomHardwareInterface::HbridgeSetpoint hbridge_cmd{};
	hbridge_cmd.duty_cycle = control_output.duty_cycle;
	hbridge_cmd.enable = (state_info.state != BoomStateManager::OperationalState::ERROR);
	hbridge_cmd.mode = 0; // PWM mode

	_hardware_interface.send_command(hbridge_cmd);

	perf_end(_control_latency_perf);
}

void BoomControl::publish_telemetry()
{
	boom_status_s status{};
	status.timestamp = hrt_absolute_time();

	// Position and velocity
	status.angle = _current_boom_angle;

	// Estimate velocity from change in position
	static float last_angle = 0.0f;
	static hrt_abstime last_time = 0;
	hrt_abstime now = hrt_absolute_time();

	if (last_time > 0) {
		float dt = (now - last_time) * 1e-6f;

		if (dt > 0.0f) {
			status.velocity = (_current_boom_angle - last_angle) / dt;
		}
	}

	last_angle = _current_boom_angle;
	last_time = now;

	// Load estimation (simplified)
	auto actuator_cmd = _hardware_interface.get_last_command();
	status.load = fabsf(actuator_cmd.duty_cycle);

	// Motor information from H-bridge interface
	BoomHardwareInterface::HbridgeStatus hbridge_status;

	if (_hardware_interface.update_status(hbridge_status)) {
		// Note: hbridge_status_s doesn't provide current, voltage, temperature, or fault fields
		status.motor_current = 0.0f;       // Not available
		status.motor_voltage = 0.0f;       // Not available
		status.motor_temperature_c = 0.0f; // Not available
		status.motor_fault = false;        // Not available
	}

	// Sensor status
	status.encoder_fault = !_hardware_interface.is_healthy();

	// State mapping
	auto state_info = _state_manager.get_state_info();

	switch (state_info.state) {
	case BoomStateManager::OperationalState::IDLE:
	case BoomStateManager::OperationalState::HOLDING:
		status.state = 2; // ready
		break;

	case BoomStateManager::OperationalState::MOVING:
		status.state = 3; // moving
		break;

	case BoomStateManager::OperationalState::ERROR:
	case BoomStateManager::OperationalState::EMERGENCY_STOP:
		status.state = 4; // error
		break;

	default:
		status.state = 0; // unknown
		break;
	}

	_boom_status_pub.publish(status);
}

void BoomControl::update_parameters()
{
	updateParams();

	// Propagate parameter updates to components
	_kinematics.update_configuration();
	_hardware_interface.update_parameters();
	_motion_controller.update_parameters();
	_state_manager.update_parameters();
	// Components will update their own parameters through ModuleParams
}

void BoomControl::handle_emergency_stop()
{
	_state_manager.emergency_stop("Manual emergency stop");
	_motion_controller.emergency_stop();
	_hardware_interface.emergency_stop();
	PX4_ERR("Emergency stop activated");
}

bool BoomControl::check_system_health()
{
	bool healthy = true;

	// Check hardware health (unified interface)
	if (!_hardware_interface.is_healthy()) {
		PX4_WARN("Hardware unhealthy");
		healthy = false;
	}

	// Check for sensor timeout
	if (_hardware_interface.time_since_last_update() > BoomHardwareInterface::SENSOR_TIMEOUT_US) {
		PX4_WARN("Sensor timeout");
		healthy = false;
	}

	return healthy;
}

// Static methods for module management
int BoomControl::task_spawn(int argc, char *argv[])
{
	BoomControl *instance = new BoomControl();

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

int BoomControl::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (argc > 0) {
		if (strcmp(argv[0], "calibrate") == 0) {
			// Trigger calibration
			get_instance()->_state_manager.start_calibration();
			return PX4_OK;
		}

		if (strcmp(argv[0], "emergency_stop") == 0) {
			// Trigger emergency stop
			get_instance()->handle_emergency_stop();
			return PX4_OK;
		}

		if (strcmp(argv[0], "clear_emergency") == 0) {
			// Clear emergency stop
			if (get_instance()->_state_manager.clear_emergency_stop()) {
				return PX4_OK;

			} else {
				return PX4_ERROR;
			}
		}

		if (strcmp(argv[0], "status") == 0) {
			// Print detailed status
			auto state_info = get_instance()->_state_manager.get_state_info();
			PX4_INFO("State: %d, Errors: 0x%lx, Message: %s",
				 static_cast<int>(state_info.state),
				 (unsigned long)state_info.error_flags,
				 state_info.status_message ? state_info.status_message : "None");
			return PX4_OK;
		}
	}

	return print_usage("unknown command");
}

int BoomControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Boom control module for wheel loader lifting system.

This module manages the boom actuator control including:
- Position and velocity control
- Trajectory planning with jerk limiting
- AS5600 magnetic encoder feedback
- H-bridge motor driver interface
- Safety monitoring and emergency stop

### Implementation
The module uses a component-based architecture with:
- Kinematics calculations for actuator-to-angle conversion
- Sensor interface for encoder reading and calibration
- Motion controller for trajectory planning and PID control
- Actuator interface for H-bridge communication
- State manager for operational states and safety

### Examples
Start the module:
$ boom_control start

Stop the module:
$ boom_control stop

Trigger calibration:
$ boom_control calibrate

Emergency stop:
$ boom_control emergency_stop

Clear emergency stop:
$ boom_control clear_emergency

Check status:
$ boom_control status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("boom_control", "actuator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("calibrate");
	PRINT_MODULE_USAGE_COMMAND("emergency_stop");
	PRINT_MODULE_USAGE_COMMAND("clear_emergency");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}
