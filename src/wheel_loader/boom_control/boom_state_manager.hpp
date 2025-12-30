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

#pragma once

#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>

/**
 * @brief State manager for boom control system
 *
 * Manages operational states and transitions:
 * - State machine logic
 * - Calibration sequences
 * - Fault handling
 * - Safety interlocks
 */
class BoomStateManager : public ModuleParams
{
public:
	// Error flag definitions
	static constexpr uint32_t ERROR_SENSOR_FAULT = (1 << 0);
	static constexpr uint32_t ERROR_ACTUATOR_FAULT = (1 << 1);
	static constexpr uint32_t ERROR_CALIBRATION_FAILED = (1 << 2);
	static constexpr uint32_t ERROR_POSITION_LIMIT = (1 << 3);
	static constexpr uint32_t ERROR_EMERGENCY_STOP = (1 << 4);

	enum class OperationalState {
		UNINITIALIZED,   // System not ready
		CALIBRATING,     // Calibration in progress
		IDLE,           // Ready but not moving
		MOVING,         // Active movement
		HOLDING,        // Maintaining position
		ERROR,          // Fault condition
		EMERGENCY_STOP  // Emergency stop active
	};

	enum class CalibrationState {
		NOT_CALIBRATED,
		FINDING_MIN,
		FINDING_MAX,
		SETTLING,
		COMPLETE
	};

	struct StateInfo {
		OperationalState state;
		CalibrationState calibration_state;
		uint32_t error_flags;
		const char *status_message;
		hrt_abstime state_entry_time;
	};

	explicit BoomStateManager(ModuleParams *parent);

	/**
	 * @brief Update state machine
	 * @param sensor_valid Sensor data validity
	 * @param actuator_healthy Actuator health status
	 * @param at_target Whether at target position
	 */
	void update(bool sensor_valid, bool actuator_healthy, bool at_target);

	/**
	 * @brief Request state transition
	 * @param new_state Desired state
	 * @return True if transition allowed
	 */
	bool request_transition(OperationalState new_state);

	/**
	 * @brief Start calibration sequence
	 * @return True if calibration started
	 */
	bool start_calibration();

	/**
	 * @brief Update calibration progress
	 * @param position Current position
	 * @param at_limit At mechanical limit
	 * @return True if calibration should continue
	 */
	bool update_calibration(float position, bool at_limit);

	/**
	 * @brief Trigger emergency stop
	 * @param reason Reason for emergency stop
	 */
	void emergency_stop(const char *reason);

	/**
	 * @brief Clear emergency stop
	 * @return True if emergency stop cleared
	 */
	bool clear_emergency_stop();

	/**
	 * @brief Get current state information
	 * @return State information structure
	 */
	StateInfo get_state_info() const;

	/**
	 * @brief Check if system is operational
	 * @return True if system can accept commands
	 */
	bool is_operational() const;

	/**
	 * @brief Get calibration progress
	 * @return Progress percentage (0-100)
	 */
	float get_calibration_progress() const;

	/**
	 * @brief Update state manager parameters from parameter system
	 * This should be called when parameters change to reconfigure the state manager
	 */
	void update_parameters();

private:
	// Current state
	OperationalState _current_state{OperationalState::UNINITIALIZED};
	CalibrationState _calibration_state{CalibrationState::NOT_CALIBRATED};
	hrt_abstime _state_entry_time{0};

	// Calibration data
	float _calibration_min{0.0f};
	float _calibration_max{0.0f};
	float _calibration_range{0.0f};
	hrt_abstime _calibration_start_time{0};

	// Error tracking
	uint32_t _error_flags{0};
	const char *_last_error_message{nullptr};

	// State management parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BOOM_CAL_REQ>) _param_calibration_required,
		(ParamFloat<px4::params::BOOM_CAL_TO>) _param_calibration_timeout,
		(ParamFloat<px4::params::BOOM_SET_TIME>) _param_settle_time
	)

	/**
	 * @brief Check if state transition is valid
	 * @param from Current state
	 * @param to Target state
	 * @return True if transition allowed
	 */
	bool is_transition_valid(OperationalState from, OperationalState to) const;

	/**
	 * @brief Set error flag
	 * @param flag Error flag to set
	 * @param message Error message
	 */
	void set_error(uint32_t flag, const char *message);

	/**
	 * @brief Clear error flag
	 * @param flag Error flag to clear
	 */
	void clear_error(uint32_t flag);
};
