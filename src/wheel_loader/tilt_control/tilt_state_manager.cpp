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

#include "tilt_state_manager.hpp"
#include <px4_platform_common/log.h>

TiltStateManager::TiltStateManager(ModuleParams *parent) :
	ModuleParams(parent)
{
	_state_entry_time = hrt_absolute_time();
}

void TiltStateManager::update(bool sensors_valid, bool hardware_healthy, bool command_timeout,
				float position, bool at_load_limit, bool at_dump_limit)
{
	// Update error flags based on inputs
	if (!sensors_valid) {
		set_error_flag(ERROR_SENSOR_TIMEOUT, "Sensor timeout");
	} else {
		clear_error_flag(ERROR_SENSOR_TIMEOUT);
	}

	if (!hardware_healthy) {
		set_error_flag(ERROR_HARDWARE_FAULT, "Hardware fault");
	} else {
		clear_error_flag(ERROR_HARDWARE_FAULT);
	}

	if (command_timeout) {
		set_error_flag(ERROR_COMMAND_TIMEOUT, "Command timeout");
	} else {
		clear_error_flag(ERROR_COMMAND_TIMEOUT);
	}

	// Handle emergency stop state
	if (_emergency_stop_active) {
		_current_state = OperationalState::EMERGENCY_STOP;
		return;
	}

	// Update main state machine
	switch (_current_state) {
		case OperationalState::UNINITIALIZED:
			// Check if we need calibration
			if (is_calibration_required()) {
				request_state_transition(OperationalState::CALIBRATING, "Auto-calibration required");
			} else {
				request_state_transition(OperationalState::READY, "System initialized");
			}
			break;

		case OperationalState::CALIBRATING:
			update_calibration_state_machine(position, at_load_limit, at_dump_limit);

			// Check for calibration timeout
			if (check_calibration_timeout()) {
				set_error_flag(ERROR_CALIBRATION_FAILED, "Calibration timeout");
				request_state_transition(OperationalState::FAULT, "Calibration failed");
			}
			break;

		case OperationalState::READY:
			// Can transition to ACTIVE when commands are received
			// This would be triggered externally via request_state_transition()
			break;

		case OperationalState::ACTIVE:
			// Can transition back to READY when no active commands
			// Check for faults that require stopping
			if (_error_flags & (ERROR_HARDWARE_FAULT | ERROR_SENSOR_TIMEOUT)) {
				request_state_transition(OperationalState::FAULT, "Hardware/sensor error");
			}
			break;

		case OperationalState::FAULT:
			// Can recover to READY if errors are cleared
			if (_error_flags == 0) {
				request_state_transition(OperationalState::READY, "Errors cleared");
			}
			break;

		case OperationalState::EMERGENCY_STOP:
			// Can only exit via explicit clear_emergency_stop() call
			break;
	}
}

bool TiltStateManager::request_state_transition(OperationalState new_state, const char* reason)
{
	if (!is_transition_valid(_current_state, new_state)) {
		set_error_flag(ERROR_INVALID_TRANSITION, "Invalid state transition");
		PX4_WARN("Invalid transition from %s to %s",
			 get_state_name(_current_state), get_state_name(new_state));
		return false;
	}

	// Log state transition
	if (reason) {
		PX4_INFO("State transition: %s -> %s (%s)",
			 get_state_name(_current_state), get_state_name(new_state), reason);
	} else {
		PX4_INFO("State transition: %s -> %s",
			 get_state_name(_current_state), get_state_name(new_state));
	}

	_current_state = new_state;
	_state_entry_time = hrt_absolute_time();

	return true;
}

bool TiltStateManager::start_calibration()
{
	if (_current_state != OperationalState::UNINITIALIZED &&
	    _current_state != OperationalState::READY &&
	    _current_state != OperationalState::FAULT) {
		PX4_WARN("Cannot start calibration from state %s", get_state_name(_current_state));
		return false;
	}

	_calibration_in_progress = true;
	_calibration_start_time = hrt_absolute_time();
	_phase_start_time = _calibration_start_time;
	_calibration_phase = CalibrationPhase::FINDING_LOAD_LIMIT;

	// Reset calibration results
	_calibration_results = CalibrationResults{};

	request_state_transition(OperationalState::CALIBRATING, "Manual calibration started");

	PX4_INFO("Starting bucket calibration sequence");
	return true;
}

void TiltStateManager::update_calibration_state_machine(float position, bool at_load_limit, bool at_dump_limit)
{
	switch (_calibration_phase) {
		case CalibrationPhase::FINDING_LOAD_LIMIT:
			if (at_load_limit) {
				_load_position = position;
				_calibration_phase = CalibrationPhase::SETTLING_AT_LOAD;
				_phase_start_time = hrt_absolute_time();
				PX4_INFO("Load limit reached at position %.1f mm", (double)position);
			}
			break;

		case CalibrationPhase::SETTLING_AT_LOAD:
			if (hrt_elapsed_time(&_phase_start_time) > SETTLING_TIME_US) {
				_calibration_phase = CalibrationPhase::FINDING_DUMP_LIMIT;
				_phase_start_time = hrt_absolute_time();
				PX4_INFO("Moving to find dump limit");
			}
			break;

		case CalibrationPhase::FINDING_DUMP_LIMIT:
			if (at_dump_limit) {
				_dump_position = position;
				_calibration_phase = CalibrationPhase::SETTLING_AT_DUMP;
				_phase_start_time = hrt_absolute_time();
				PX4_INFO("Dump limit reached at position %.1f mm", (double)position);
			}
			break;

		case CalibrationPhase::SETTLING_AT_DUMP:
			if (hrt_elapsed_time(&_phase_start_time) > SETTLING_TIME_US) {
				_calibration_phase = CalibrationPhase::COMPUTING_PARAMETERS;
				_phase_start_time = hrt_absolute_time();
			}
			break;

		case CalibrationPhase::COMPUTING_PARAMETERS:
			finalize_calibration();
			_calibration_phase = CalibrationPhase::COMPLETE;
			break;

		case CalibrationPhase::COMPLETE:
			_calibration_in_progress = false;
			request_state_transition(OperationalState::READY, "Calibration completed");
			break;

		case CalibrationPhase::IDLE:
		default:
			// Should not reach here during calibration
			break;
	}
}

void TiltStateManager::finalize_calibration()
{
	// Compute calibration parameters
	float margin = _param_calibration_margin.get();

	_calibration_results.actuator_min_length = _load_position + margin;
	_calibration_results.actuator_max_length = _dump_position - margin;
	_calibration_results.calibration_valid = true;
	_calibration_results.calibration_time = hrt_absolute_time();

	// Validate calibration results
	if (_calibration_results.actuator_max_length <= _calibration_results.actuator_min_length) {
		PX4_ERR("Invalid calibration: max <= min (%.1f <= %.1f)",
			(double)_calibration_results.actuator_max_length,
			(double)_calibration_results.actuator_min_length);
		_calibration_results.calibration_valid = false;
		set_error_flag(ERROR_CALIBRATION_FAILED, "Invalid calibration range");
		return;
	}

	float range = _calibration_results.actuator_max_length - _calibration_results.actuator_min_length;
	PX4_INFO("Calibration completed: range=%.1f mm (%.1f - %.1f)",
		 (double)range,
		 (double)_calibration_results.actuator_min_length,
		 (double)_calibration_results.actuator_max_length);
}

bool TiltStateManager::get_calibration_command(float& position_target, float& velocity_target) const
{
	if (_current_state != OperationalState::CALIBRATING) {
		return false;
	}

	float fast_speed = _param_calibration_fast_speed.get();
	float slow_speed = _param_calibration_slow_speed.get();

	switch (_calibration_phase) {
		case CalibrationPhase::FINDING_LOAD_LIMIT:
			position_target = 0.0f;  // Move to minimum position
			velocity_target = -fast_speed * 0.5f;  // Moderate downward speed
			return true;

		case CalibrationPhase::SETTLING_AT_LOAD:
			position_target = _load_position;
			velocity_target = -slow_speed;  // Use slow speed for settling
			return true;

		case CalibrationPhase::FINDING_DUMP_LIMIT:
			position_target = 1000.0f;  // Move to maximum position
			velocity_target = fast_speed;
			return true;

		case CalibrationPhase::SETTLING_AT_DUMP:
			position_target = _dump_position;
			velocity_target = slow_speed;  // Use slow speed for settling
			return true;

		default:
			position_target = 0.0f;
			velocity_target = 0.0f;
			return true;
	}
}

TiltStateManager::StateInfo TiltStateManager::get_state_info() const
{
	StateInfo info{};
	info.state = _current_state;
	info.calibration_phase = _calibration_phase;
	info.error_flags = _error_flags;
	info.state_entry_time = _state_entry_time;
	info.time_in_state = hrt_elapsed_time(&_state_entry_time);

	// Set status message based on state
	if (_emergency_stop_active) {
		info.status_message = _emergency_stop_reason ? _emergency_stop_reason : "Emergency stop active";
	} else if (_error_flags != 0) {
		info.status_message = _last_error_message ? _last_error_message : "Error condition";
	} else {
		info.status_message = get_state_name(_current_state);
	}

	// Calculate calibration progress
	if (_current_state == OperationalState::CALIBRATING && _calibration_in_progress) {
		switch (_calibration_phase) {
			case CalibrationPhase::FINDING_LOAD_LIMIT:
				info.calibration_progress = 0.2f;
				break;
			case CalibrationPhase::SETTLING_AT_LOAD:
				info.calibration_progress = 0.4f;
				break;
			case CalibrationPhase::FINDING_DUMP_LIMIT:
				info.calibration_progress = 0.6f;
				break;
			case CalibrationPhase::SETTLING_AT_DUMP:
				info.calibration_progress = 0.8f;
				break;
			case CalibrationPhase::COMPUTING_PARAMETERS:
				info.calibration_progress = 0.9f;
				break;
			case CalibrationPhase::COMPLETE:
				info.calibration_progress = 1.0f;
				break;
			default:
				info.calibration_progress = 0.0f;
				break;
		}
	} else {
		info.calibration_progress = 0.0f;
	}

	return info;
}

void TiltStateManager::trigger_emergency_stop(const char* reason)
{
	_emergency_stop_active = true;
	_emergency_stop_reason = reason;
	_emergency_stop_time = hrt_absolute_time();

	set_error_flag(ERROR_EMERGENCY_STOP, reason);

	PX4_ERR("EMERGENCY STOP: %s", reason ? reason : "Unknown");
}

bool TiltStateManager::clear_emergency_stop()
{
	if (!_emergency_stop_active) {
		return true;
	}

	// Check cooldown period
	if (hrt_elapsed_time(&_emergency_stop_time) < EMERGENCY_STOP_COOLDOWN_US) {
		PX4_WARN("Emergency stop cooldown active");
		return false;
	}

	_emergency_stop_active = false;
	_emergency_stop_reason = nullptr;
	clear_error_flag(ERROR_EMERGENCY_STOP);

	request_state_transition(OperationalState::FAULT, "Emergency stop cleared");

	PX4_INFO("Emergency stop cleared");
	return true;
}

void TiltStateManager::update_parameters()
{
	// Update parameters from parameter system
	ModuleParams::updateParams();

	// Get new parameter values
	float new_fast_speed = _param_calibration_fast_speed.get();
	float new_slow_speed = _param_calibration_slow_speed.get();
	int new_calibration_required = _param_calibration_required.get();
	float new_calibration_margin = _param_calibration_margin.get();

	// Log parameter changes for debugging
	PX4_DEBUG("State manager parameters updated:");
	PX4_DEBUG("  Calibration fast speed: %.1f mm/s", (double)new_fast_speed);
	PX4_DEBUG("  Calibration slow speed: %.1f mm/s", (double)new_slow_speed);
	PX4_DEBUG("  Calibration required: %s", new_calibration_required ? "true" : "false");
	PX4_DEBUG("  Calibration margin: %.1f mm", (double)new_calibration_margin);

	// If calibration is in progress and calibration parameters changed significantly,
	// the operator may want to restart calibration with new parameters
	if (_calibration_in_progress) {
		PX4_WARN("Calibration parameters changed during calibration - consider restarting calibration");
	}

	// If calibration requirement setting changed, update state accordingly
	if (!new_calibration_required && is_calibration_required()) {
		PX4_INFO("Calibration requirement disabled - marking as calibrated");
		_calibration_results.calibration_valid = true;
		// Could transition to READY state if appropriate
	} else if (new_calibration_required && !_calibration_results.calibration_valid) {
		PX4_INFO("Calibration requirement enabled - calibration needed");
		// System will need to be calibrated before operation
	}
}

bool TiltStateManager::is_operational() const
{
	return (_current_state == OperationalState::READY ||
		_current_state == OperationalState::ACTIVE) &&
		!_emergency_stop_active;
}

bool TiltStateManager::is_calibration_required() const
{
	return (_param_calibration_required.get() != 0) ||
		!_calibration_results.calibration_valid;
}

const char* TiltStateManager::get_state_name(OperationalState state)
{
	switch (state) {
		case OperationalState::UNINITIALIZED: return "UNINITIALIZED";
		case OperationalState::CALIBRATING:   return "CALIBRATING";
		case OperationalState::READY:         return "READY";
		case OperationalState::ACTIVE:        return "ACTIVE";
		case OperationalState::FAULT:         return "FAULT";
		case OperationalState::EMERGENCY_STOP: return "EMERGENCY_STOP";
		default:                               return "UNKNOWN";
	}
}

const char* TiltStateManager::get_phase_name(CalibrationPhase phase)
{
	switch (phase) {
		case CalibrationPhase::IDLE:                 return "IDLE";
		case CalibrationPhase::FINDING_LOAD_LIMIT:   return "FINDING_LOAD_LIMIT";
		case CalibrationPhase::SETTLING_AT_LOAD:     return "SETTLING_AT_LOAD";
		case CalibrationPhase::FINDING_DUMP_LIMIT:   return "FINDING_DUMP_LIMIT";
		case CalibrationPhase::SETTLING_AT_DUMP:     return "SETTLING_AT_DUMP";
		case CalibrationPhase::COMPUTING_PARAMETERS: return "COMPUTING_PARAMETERS";
		case CalibrationPhase::COMPLETE:             return "COMPLETE";
		default:                                     return "UNKNOWN";
	}
}

// Helper functions for error management
void TiltStateManager::set_error_flag(uint32_t error_flag, const char* message)
{
	if ((_error_flags & error_flag) == 0) {
		_error_flags |= error_flag;
		_last_error_message = message;
		PX4_WARN("Error set: %s (flag=0x%lx)", message, (unsigned long)error_flag);
	}
}

void TiltStateManager::clear_error_flag(uint32_t error_flag)
{
	_error_flags &= ~error_flag;
}

bool TiltStateManager::is_transition_valid(OperationalState from, OperationalState to) const
{
	// Define valid state transitions
	switch (from) {
		case OperationalState::UNINITIALIZED:
			return (to == OperationalState::CALIBRATING || to == OperationalState::READY);

		case OperationalState::CALIBRATING:
			return (to == OperationalState::READY || to == OperationalState::FAULT ||
				to == OperationalState::EMERGENCY_STOP);

		case OperationalState::READY:
			return (to == OperationalState::ACTIVE || to == OperationalState::CALIBRATING ||
				to == OperationalState::FAULT || to == OperationalState::EMERGENCY_STOP);

		case OperationalState::ACTIVE:
			return (to == OperationalState::READY || to == OperationalState::FAULT ||
				to == OperationalState::EMERGENCY_STOP);

		case OperationalState::FAULT:
			return (to == OperationalState::READY || to == OperationalState::CALIBRATING ||
				to == OperationalState::EMERGENCY_STOP);

		case OperationalState::EMERGENCY_STOP:
			return (to == OperationalState::FAULT);  // Only via clear_emergency_stop()

		default:
			return false;
	}
}

bool TiltStateManager::check_calibration_timeout() const
{
	if (_current_state != OperationalState::CALIBRATING) {
		return false;
	}

	// Check if calibration has been running too long
	hrt_abstime calibration_timeout = 30 * 1000000;  // 30 seconds timeout
	return (_calibration_start_time != 0) &&
		   (hrt_elapsed_time(&_calibration_start_time) > calibration_timeout);
}
