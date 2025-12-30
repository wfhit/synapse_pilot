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

// System includes
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

/**
 * @brief Manages operational states and transitions for tilt control
 *
 * This class handles:
 * - System state machine (initialization, calibration, operation)
 * - Fault detection and recovery
 * - Calibration sequence management
 * - Safety monitoring and emergency stops
 */
class TiltStateManager : public ModuleParams
{
public:
	/**
	 * @brief Main operational states
	 */
	enum class OperationalState : uint8_t {
		UNINITIALIZED = 0,    // System not ready
		CALIBRATING = 1,      // Running calibration sequence
		READY = 2,           // Ready for operation
		ACTIVE = 3,          // Actively controlling bucket
		FAULT = 4,           // Fault detected, limited operation
		EMERGENCY_STOP = 5   // Emergency stop, no operation
	};

	/**
	 * @brief Calibration sequence phases
	 */
	enum class CalibrationPhase : uint8_t {
		IDLE = 0,                    // Not calibrating
		FINDING_LOAD_LIMIT = 1,      // Moving to load limit switch
		SETTLING_AT_LOAD = 2,        // Settling at load position
		FINDING_DUMP_LIMIT = 3,      // Moving to dump limit switch
		SETTLING_AT_DUMP = 4,        // Settling at dump position
		COMPUTING_PARAMETERS = 5,    // Computing calibration parameters
		COMPLETE = 6                 // Calibration complete
	};

	/**
	 * @brief Complete state information structure
	 */
	struct StateInfo {
		OperationalState state;
		CalibrationPhase calibration_phase;
		uint32_t error_flags;
		hrt_abstime state_entry_time;
		hrt_abstime time_in_state;
		const char* status_message;
		float calibration_progress;     // 0.0 to 1.0
	};

	/**
	 * @brief Calibration results structure
	 */
	struct CalibrationResults {
		float actuator_min_length;      // Measured minimum actuator length (mm)
		float actuator_max_length;      // Measured maximum actuator length (mm)
		float bucket_angle_at_load;     // Bucket angle at load position (rad)
		float bucket_angle_at_dump;     // Bucket angle at dump position (rad)
		bool calibration_valid;         // True if calibration completed successfully
		hrt_abstime calibration_time;   // Time when calibration completed
	};

	explicit TiltStateManager(ModuleParams *parent);

	/**
	 * @brief Update state machine based on system conditions
	 * @param sensors_valid True if sensor readings are valid
	 * @param hardware_healthy True if hardware is functioning
	 * @param command_timeout True if command timeout occurred
	 * @param position Current actuator position (mm)
	 * @param at_load_limit True if at load limit switch
	 * @param at_dump_limit True if at dump limit switch
	 */
	void update(bool sensors_valid, bool hardware_healthy, bool command_timeout,
		    float position, bool at_load_limit, bool at_dump_limit);

	/**
	 * @brief Request state transition
	 * @param new_state Requested new state
	 * @param reason Reason for state change (for logging)
	 * @return True if transition is valid and accepted
	 */
	bool request_state_transition(OperationalState new_state, const char* reason = nullptr);

	/**
	 * @brief Start calibration sequence
	 * @return True if calibration started successfully
	 */
	bool start_calibration();

	/**
	 * @brief Stop calibration sequence
	 * @param reason Reason for stopping calibration
	 */
	void stop_calibration(const char* reason);

	/**
	 * @brief Trigger emergency stop
	 * @param reason Reason for emergency stop
	 */
	void trigger_emergency_stop(const char* reason);

	/**
	 * @brief Clear emergency stop (manual recovery)
	 * @return True if emergency stop cleared successfully
	 */
	bool clear_emergency_stop();

	/**
	 * @brief Update state manager parameters from parameter system
	 * This should be called when parameters change to reconfigure the state manager
	 */
	void update_parameters();

	/**
	 * @brief Get current state information
	 * @return Current state info structure
	 */
	StateInfo get_state_info() const;

	/**
	 * @brief Get calibration results
	 * @return Calibration results structure
	 */
	const CalibrationResults& get_calibration_results() const { return _calibration_results; }

	/**
	 * @brief Check if system is operational (ready or active)
	 * @return True if system can accept commands
	 */
	bool is_operational() const;

	/**
	 * @brief Check if calibration is required
	 * @return True if system needs calibration
	 */
	bool is_calibration_required() const;

	/**
	 * @brief Get current calibration command
	 * @param position_target Output: target position for calibration (mm)
	 * @param velocity_target Output: target velocity for calibration (mm/s)
	 * @return True if calibration command is active
	 */
	bool get_calibration_command(float& position_target, float& velocity_target) const;

private:
	// Current state
	OperationalState _current_state{OperationalState::UNINITIALIZED};
	CalibrationPhase _calibration_phase{CalibrationPhase::IDLE};

	// State timing
	hrt_abstime _state_entry_time{0};
	hrt_abstime _calibration_start_time{0};
	hrt_abstime _phase_start_time{0};

	// Calibration data
	CalibrationResults _calibration_results{};
	float _load_position{0.0f};          // Position when load limit reached
	float _dump_position{0.0f};          // Position when dump limit reached
	bool _calibration_in_progress{false};

	// Error tracking
	uint32_t _error_flags{0};
	const char* _last_error_message{nullptr};

	// Emergency stop state
	bool _emergency_stop_active{false};
	const char* _emergency_stop_reason{nullptr};
	hrt_abstime _emergency_stop_time{0};

	// Error flag definitions
	static constexpr uint32_t ERROR_SENSOR_TIMEOUT     = (1 << 0);
	static constexpr uint32_t ERROR_HARDWARE_FAULT     = (1 << 1);
	static constexpr uint32_t ERROR_COMMAND_TIMEOUT    = (1 << 2);
	static constexpr uint32_t ERROR_LIMIT_VIOLATION    = (1 << 3);
	static constexpr uint32_t ERROR_CALIBRATION_FAILED = (1 << 4);
	static constexpr uint32_t ERROR_EMERGENCY_STOP     = (1 << 5);
	static constexpr uint32_t ERROR_INVALID_TRANSITION = (1 << 6);

	// Timing constants
	static constexpr hrt_abstime CALIBRATION_TIMEOUT_US    = 120000000;  // 2 minutes
	static constexpr hrt_abstime PHASE_TIMEOUT_US          = 30000000;   // 30 seconds per phase
	static constexpr hrt_abstime SETTLING_TIME_US          = 2000000;    // Settling time
	static constexpr hrt_abstime EMERGENCY_STOP_COOLDOWN_US = 5000000;   // Cooldown before recovery

	/**
	 * @brief Validate state transition
	 * @param from Source state
	 * @param to Target state
	 * @return True if transition is valid
	 */
	bool is_transition_valid(OperationalState from, OperationalState to) const;

	/**
	 * @brief Handle calibration state machine
	 * @param position Current actuator position (mm)
	 * @param at_load_limit True if at load limit
	 * @param at_dump_limit True if at dump limit
	 */
	void update_calibration_state_machine(float position, bool at_load_limit, bool at_dump_limit);

	/**
	 * @brief Check for calibration timeout
	 * @return True if calibration has timed out
	 */
	bool check_calibration_timeout() const;

	/**
	 * @brief Finalize calibration with computed parameters
	 */
	void finalize_calibration();

	/**
	 * @brief Set error flag and update error state
	 * @param error_flag Error flag to set
	 * @param message Error message for logging
	 */
	void set_error_flag(uint32_t error_flag, const char* message);

	/**
	 * @brief Clear error flag
	 * @param error_flag Error flag to clear
	 */
	void clear_error_flag(uint32_t error_flag);

	/**
	 * @brief Get human-readable state name
	 * @param state Operational state
	 * @return State name string
	 */
	static const char* get_state_name(OperationalState state);

	/**
	 * @brief Get human-readable calibration phase name
	 * @param phase Calibration phase
	 * @return Phase name string
	 */
	static const char* get_phase_name(CalibrationPhase phase);

	// Calibration parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BCT_CAL_FAST_SPD>) _param_calibration_fast_speed,
		(ParamFloat<px4::params::BCT_CAL_SLOW_SPD>) _param_calibration_slow_speed,
		(ParamInt<px4::params::BCT_CAL_REQUIRED>) _param_calibration_required,
		(ParamFloat<px4::params::BCT_CAL_MARGIN>) _param_calibration_margin
	)
};
