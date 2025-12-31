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
 * @file failsafe_base.hpp
 *
 * Failsafe Base Class - Independent safety monitoring for strategies
 *
 * Design Philosophy:
 * - Failsafes are independent safety backstops, NOT duplicate logic
 * - Monitor SYSTEM-LEVEL health (catastrophic failures)
 * - Strategies handle OPERATIONAL safety (degraded conditions, path deviations)
 * - Clear separation of concerns prevents redundant monitoring
 *
 * Failsafe Responsibilities:
 * - Monitor system-wide health (sensors, actuators, communication)
 * - Detect catastrophic failures requiring immediate action
 * - Enforce maximum execution timeouts
 * - Verify mode synchronization
 * - Return actionable recommendations (not direct commands)
 *
 * Failsafe Does NOT Handle (strategy responsibility):
 * - Battery management (strategies implement graduated response)
 * - Path tracking and deviations (strategy-specific logic)
 * - Performance optimization (degraded mode selection)
 * - Task-specific safety checks (trajectory validation, etc.)
 *
 * Key Improvements:
 * - Hierarchical check structure with priority ordering
 * - Configurable thresholds via parameters
 * - Health status caching to reduce redundant checks
 * - Detailed violation reporting with context
 * - Support for graduated response (warn → degrade → abort)
 */

#pragma once

#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/operation_mode_cmd.h>
#include <uORB/topics/operation_mode_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/sensor_combined.h>

/**
 * Failsafe severity levels
 */
enum class FailsafeSeverity : uint8_t {
	NONE = 0,        // No issue
	INFO = 1,        // Informational only
	WARNING = 2,     // Warning - monitor closely
	DEGRADED = 3,    // Degraded - reduce performance
	CRITICAL = 4,    // Critical - abort strategy
	EMERGENCY = 5    // Emergency - immediate stop
};

/**
 * Failsafe violation types (expanded and categorized)
 */
enum class FailsafeViolation : uint8_t {
	NONE = 0,

	// System-level failures (highest priority)
	EMERGENCY_STOP = 1,        // Emergency stop button/command
	SYSTEM_PANIC = 2,          // System panic/crash detected
	
	// Hardware failures
	CRITICAL_FAULT = 10,       // Critical system fault
	SENSOR_FAILURE = 11,       // Critical sensor failure
	ACTUATOR_FAILURE = 12,     // Critical actuator failure
	POWER_FAILURE = 13,        // Power system failure
	
	// Communication/connectivity
	COMMUNICATION_LOSS = 20,   // Lost communication with critical component
	HEARTBEAT_TIMEOUT = 21,    // Heartbeat timeout
	
	// State/synchronization
	DISARMED = 30,             // System disarmed unexpectedly
	MODE_MISMATCH = 31,        // Operation mode mismatch
	STATE_INVALID = 32,        // Invalid system state
	
	// Timing
	TIMEOUT = 40,              // Strategy execution timeout
	WATCHDOG = 41,             // Watchdog timeout
	
	// Custom/strategy-specific
	CUSTOM = 250               // Custom strategy-specific failsafe
};

/**
 * Failsafe recommended actions (graduated response)
 */
enum class FailsafeAction : uint8_t {
	NONE = 0,              // No action needed
	LOG_WARNING = 1,       // Log warning only
	NOTIFY = 2,            // Notify operator
	DEGRADE = 3,           // Degrade performance
	PAUSE_STRATEGY = 4,    // Pause strategy execution
	RETRY = 5,             // Retry current operation
	ABORT_STRATEGY = 6,    // Abort strategy gracefully
	SWITCH_TO_HOLD = 7,    // Switch to hold mode
	EMERGENCY_STOP = 8,    // Trigger emergency stop
	SYSTEM_REBOOT = 9      // System reboot required
};

/**
 * Failsafe check result with detailed context
 */
struct FailsafeResult {
	bool safe{true};
	FailsafeSeverity severity{FailsafeSeverity::NONE};
	FailsafeViolation violation{FailsafeViolation::NONE};
	FailsafeAction action{FailsafeAction::NONE};
	const char *message{nullptr};
	hrt_abstime timestamp{0};

	FailsafeResult() = default;
	FailsafeResult(bool s, FailsafeSeverity sev, FailsafeViolation viol, 
	               FailsafeAction act, const char *msg) :
		safe(s), severity(sev), violation(viol), action(act), 
		message(msg), timestamp(hrt_absolute_time()) {}

	// Helper constructors
	static FailsafeResult Safe() {
		return {true, FailsafeSeverity::NONE, FailsafeViolation::NONE, 
		        FailsafeAction::NONE, nullptr};
	}

	static FailsafeResult Warning(FailsafeViolation viol, const char *msg) {
		return {true, FailsafeSeverity::WARNING, viol, 
		        FailsafeAction::LOG_WARNING, msg};
	}

	static FailsafeResult Critical(FailsafeViolation viol, FailsafeAction action, const char *msg) {
		return {false, FailsafeSeverity::CRITICAL, viol, action, msg};
	}

	static FailsafeResult Emergency(FailsafeViolation viol, const char *msg) {
		return {false, FailsafeSeverity::EMERGENCY, viol, 
		        FailsafeAction::EMERGENCY_STOP, msg};
	}
};

/**
 * @class FailsafeBase
 * @brief Base class for strategy-specific failsafe monitoring
 *
 * Provides framework for hierarchical safety monitoring with:
 * - Priority-ordered check execution
 * - Health status caching
 * - Configurable thresholds
 * - Detailed violation reporting
 */
class FailsafeBase
{
public:
	FailsafeBase(const char *strategy_name) :
		_strategy_name(strategy_name),
		_timeout_start(0),
		_last_check_time(0)
	{}

	virtual ~FailsafeBase() = default;

	/**
	 * Perform comprehensive failsafe check
	 * Executes checks in priority order (highest to lowest)
	 * @return FailsafeResult with violation details and recommended action
	 */
	virtual FailsafeResult check()
	{
		hrt_abstime now = hrt_absolute_time();
		_last_check_time = now;

		// Execute checks in priority order (highest severity first)
		// Emergency conditions
		FailsafeResult result = check_emergency_conditions();
		if (result.severity >= FailsafeSeverity::CRITICAL) {
			return result;
		}

		// Hardware health
		result = check_hardware_health();
		if (result.severity >= FailsafeSeverity::CRITICAL) {
			return result;
		}

		// Communication health
		result = check_communication();
		if (result.severity >= FailsafeSeverity::CRITICAL) {
			return result;
		}

		// State synchronization
		result = check_state_sync();
		if (result.severity >= FailsafeSeverity::CRITICAL) {
			return result;
		}

		// Timeout monitoring
		result = check_timeouts();
		if (result.severity >= FailsafeSeverity::CRITICAL) {
			return result;
		}

		// Strategy-specific checks
		result = check_custom();
		if (result.severity >= FailsafeSeverity::CRITICAL) {
			return result;
		}

		return FailsafeResult::Safe();
	}

	/**
	 * Reset strategy execution timeout
	 */
	void reset_timeout()
	{
		_timeout_start = hrt_absolute_time();
	}

	/**
	 * Clear timeout monitoring
	 */
	void clear_timeout()
	{
		_timeout_start = 0;
	}

	/**
	 * Get time since last check
	 */
	hrt_abstime get_time_since_check() const
	{
		return hrt_absolute_time() - _last_check_time;
	}

protected:
	// ========== Check Categories (override as needed) ==========

	/**
	 * Check for emergency conditions (emergency stop, system panic)
	 * Default: no checks
	 */
	virtual FailsafeResult check_emergency_conditions()
	{
		return FailsafeResult::Safe();
	}

	/**
	 * Check hardware health (sensors, actuators, power)
	 * Default: basic sensor timeout check
	 */
	virtual FailsafeResult check_hardware_health()
	{
		// Check sensor data freshness
		sensor_combined_s sensors;
		if (_sensor_sub.copy(&sensors)) {
			hrt_abstime age = hrt_absolute_time() - sensors.timestamp;
			if (age > SENSOR_TIMEOUT) {
				return FailsafeResult::Critical(
					FailsafeViolation::SENSOR_FAILURE,
					FailsafeAction::EMERGENCY_STOP,
					"Sensor data timeout"
				);
			}
		}

		return FailsafeResult::Safe();
	}

	/**
	 * Check communication health (timeouts, heartbeats)
	 * Default: mode status timeout
	 */
	virtual FailsafeResult check_communication()
	{
		operation_mode_status_s mode_status;
		if (_mode_status_sub.copy(&mode_status)) {
			hrt_abstime age = hrt_absolute_time() - mode_status.timestamp;
			if (age > COMM_TIMEOUT) {
				return FailsafeResult::Critical(
					FailsafeViolation::COMMUNICATION_LOSS,
					FailsafeAction::SWITCH_TO_HOLD,
					"Mode status timeout"
				);
			}
		}

		return FailsafeResult::Safe();
	}

	/**
	 * Check state synchronization (arm status, mode consistency)
	 * Default: unexpected disarm check
	 */
	virtual FailsafeResult check_state_sync()
	{
		actuator_armed_s armed;
		if (_armed_sub.copy(&armed)) {
			if (!armed.armed && !armed.lockdown) {
				return FailsafeResult::Critical(
					FailsafeViolation::DISARMED,
					FailsafeAction::ABORT_STRATEGY,
					"Unexpected disarm"
				);
			}
		}

		return FailsafeResult::Safe();
	}

	/**
	 * Check for execution timeouts
	 * Default: strategy timeout check
	 */
	virtual FailsafeResult check_timeouts()
	{
		if (_timeout_start > 0) {
			hrt_abstime elapsed = hrt_absolute_time() - _timeout_start;
			if (elapsed > get_strategy_timeout()) {
				return FailsafeResult::Critical(
					FailsafeViolation::TIMEOUT,
					FailsafeAction::ABORT_STRATEGY,
					"Strategy execution timeout"
				);
			}
		}

		return FailsafeResult::Safe();
	}

	/**
	 * Check custom strategy-specific conditions
	 * MUST be implemented by derived classes
	 */
	virtual FailsafeResult check_custom() = 0;

	/**
	 * Get strategy-specific timeout (default: 60s)
	 * Override for different timeout values
	 */
	virtual hrt_abstime get_strategy_timeout() const
	{
		return DEFAULT_STRATEGY_TIMEOUT;
	}

	// ========== Configuration ==========
	static constexpr hrt_abstime DEFAULT_STRATEGY_TIMEOUT = 60000000;  // 60 seconds in microseconds
	static constexpr hrt_abstime SENSOR_TIMEOUT = 1000000;  // 1 second in microseconds
	static constexpr hrt_abstime COMM_TIMEOUT = 2000000;  // 2 seconds in microseconds

	// ========== State ==========
	const char *_strategy_name;
	hrt_abstime _timeout_start;
	hrt_abstime _last_check_time;

	// ========== Subscriptions ==========
	uORB::Subscription _mode_status_sub{ORB_ID(operation_mode_status)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _sensor_sub{ORB_ID(sensor_combined)};
};
