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

#include "boom_state_manager.hpp"
#include <px4_platform_common/log.h>

BoomStateManager::BoomStateManager(ModuleParams *parent) :
	ModuleParams(parent)
{
	_state_entry_time = hrt_absolute_time();
}

void BoomStateManager::update(bool sensor_valid, bool actuator_healthy, bool at_target)
{
	// State machine logic
	switch (_current_state) {
	case OperationalState::UNINITIALIZED:
		if (sensor_valid && actuator_healthy) {
			if (_param_calibration_required.get() && _calibration_state == CalibrationState::NOT_CALIBRATED) {
				request_transition(OperationalState::CALIBRATING);

			} else {
				request_transition(OperationalState::IDLE);
			}
		}

		break;

	case OperationalState::CALIBRATING:

		// Calibration logic would go here
		// For now, just timeout after a while
		if (hrt_elapsed_time(&_state_entry_time) > (_param_calibration_timeout.get() * 1000000)) {
			_calibration_state = CalibrationState::COMPLETE;
			request_transition(OperationalState::IDLE);
		}

		break;

	case OperationalState::IDLE:
		// Ready to accept commands
		break;

	case OperationalState::MOVING:
		if (at_target) {
			request_transition(OperationalState::HOLDING);
		}

		break;

	case OperationalState::HOLDING:
		// Maintaining position
		break;

	case OperationalState::ERROR:

		// Check if errors are cleared
		if (sensor_valid && actuator_healthy && (_error_flags == 0)) {
			request_transition(OperationalState::IDLE);
		}

		break;

	case OperationalState::EMERGENCY_STOP:
		// Emergency stop - can only be cleared manually
		break;
	}

	// Monitor system health
	if (!sensor_valid) {
		set_error(ERROR_SENSOR_FAULT, "Sensor fault");

	} else {
		clear_error(ERROR_SENSOR_FAULT);
	}

	if (!actuator_healthy) {
		set_error(ERROR_ACTUATOR_FAULT, "Actuator fault");

	} else {
		clear_error(ERROR_ACTUATOR_FAULT);
	}

	// Transition to error state if faults detected
	if (_error_flags != 0 && _current_state != OperationalState::EMERGENCY_STOP) {
		request_transition(OperationalState::ERROR);
	}
}

bool BoomStateManager::request_transition(OperationalState new_state)
{
	if (!is_transition_valid(_current_state, new_state)) {
		PX4_WARN("Invalid state transition: %d -> %d",
			 static_cast<int>(_current_state), static_cast<int>(new_state));
		return false;
	}

	PX4_INFO("State transition: %d -> %d",
		 static_cast<int>(_current_state), static_cast<int>(new_state));

	_current_state = new_state;
	_state_entry_time = hrt_absolute_time();

	return true;
}

bool BoomStateManager::start_calibration()
{
	if (_current_state == OperationalState::IDLE || _current_state == OperationalState::HOLDING) {
		_calibration_state = CalibrationState::FINDING_MIN;
		_calibration_start_time = hrt_absolute_time();
		return request_transition(OperationalState::CALIBRATING);
	}

	PX4_WARN("Cannot start calibration in current state: %d", static_cast<int>(_current_state));
	return false;
}

bool BoomStateManager::update_calibration(float position, bool at_limit)
{
	if (_current_state != OperationalState::CALIBRATING) {
		return false;
	}

	// Simple calibration state machine
	switch (_calibration_state) {
	case CalibrationState::FINDING_MIN:
		if (at_limit) {
			_calibration_min = position;
			_calibration_state = CalibrationState::FINDING_MAX;
		}

		break;

	case CalibrationState::FINDING_MAX:
		if (at_limit) {
			_calibration_max = position;
			_calibration_range = _calibration_max - _calibration_min;
			_calibration_state = CalibrationState::SETTLING;
		}

		break;

	case CalibrationState::SETTLING:

		// Wait for system to settle
		if (hrt_elapsed_time(&_calibration_start_time) > (_param_settle_time.get() * 1000000)) {
			_calibration_state = CalibrationState::COMPLETE;
			request_transition(OperationalState::IDLE);
			PX4_INFO("Calibration complete: range = %.2f", (double)_calibration_range);
			return false; // Calibration finished
		}

		break;

	default:
		break;
	}

	return true; // Continue calibration
}

void BoomStateManager::emergency_stop(const char *reason)
{
	_current_state = OperationalState::EMERGENCY_STOP;
	_state_entry_time = hrt_absolute_time();
	set_error(ERROR_EMERGENCY_STOP, reason);

	PX4_ERR("Emergency stop triggered: %s", reason ? reason : "Unknown");
}

bool BoomStateManager::clear_emergency_stop()
{
	if (_current_state == OperationalState::EMERGENCY_STOP) {
		clear_error(ERROR_EMERGENCY_STOP);
		return request_transition(OperationalState::IDLE);
	}

	return false;
}

BoomStateManager::StateInfo BoomStateManager::get_state_info() const
{
	StateInfo info{};
	info.state = _current_state;
	info.calibration_state = _calibration_state;
	info.error_flags = _error_flags;
	info.status_message = _last_error_message;
	info.state_entry_time = _state_entry_time;

	return info;
}

bool BoomStateManager::is_operational() const
{
	return (_current_state == OperationalState::IDLE ||
		_current_state == OperationalState::MOVING ||
		_current_state == OperationalState::HOLDING);
}

float BoomStateManager::get_calibration_progress() const
{
	if (_current_state != OperationalState::CALIBRATING) {
		return (_calibration_state == CalibrationState::COMPLETE) ? 100.0f : 0.0f;
	}

	// Simple progress calculation
	switch (_calibration_state) {
	case CalibrationState::FINDING_MIN:
		return 25.0f;

	case CalibrationState::FINDING_MAX:
		return 50.0f;

	case CalibrationState::SETTLING:
		return 75.0f;

	case CalibrationState::COMPLETE:
		return 100.0f;

	default:
		return 0.0f;
	}
}

bool BoomStateManager::is_transition_valid(OperationalState from, OperationalState to) const
{
	// Define valid state transitions
	switch (from) {
	case OperationalState::UNINITIALIZED:
		return (to == OperationalState::CALIBRATING || to == OperationalState::IDLE ||
			to == OperationalState::EMERGENCY_STOP);

	case OperationalState::CALIBRATING:
		return (to == OperationalState::IDLE || to == OperationalState::ERROR ||
			to == OperationalState::EMERGENCY_STOP);

	case OperationalState::IDLE:
		return (to == OperationalState::MOVING || to == OperationalState::CALIBRATING ||
			to == OperationalState::ERROR || to == OperationalState::EMERGENCY_STOP);

	case OperationalState::MOVING:
		return (to == OperationalState::HOLDING || to == OperationalState::IDLE ||
			to == OperationalState::ERROR || to == OperationalState::EMERGENCY_STOP);

	case OperationalState::HOLDING:
		return (to == OperationalState::MOVING || to == OperationalState::IDLE ||
			to == OperationalState::ERROR || to == OperationalState::EMERGENCY_STOP);

	case OperationalState::ERROR:
		return (to == OperationalState::IDLE || to == OperationalState::EMERGENCY_STOP);

	case OperationalState::EMERGENCY_STOP:
		return (to == OperationalState::IDLE); // Only through clear_emergency_stop()

	default:
		return false;
	}
}

void BoomStateManager::set_error(uint32_t flag, const char *message)
{
	_error_flags |= flag;
	_last_error_message = message;

	if (message) {
		PX4_WARN("Error set: %s (flags: 0x%lx)", message, (unsigned long)_error_flags);
	}
}

void BoomStateManager::clear_error(uint32_t flag)
{
	_error_flags &= ~flag;

	if (_error_flags == 0) {
		_last_error_message = nullptr;
	}
}

void BoomStateManager::update_parameters()
{
	// Update all parameters from the parameter system
	updateParams();

	PX4_DEBUG("State manager parameters updated: cal_req=%d, cal_timeout=%.1f, settle_time=%.1f",
		  _param_calibration_required.get(),
		  (double)_param_calibration_timeout.get(),
		  (double)_param_settle_time.get());
}
