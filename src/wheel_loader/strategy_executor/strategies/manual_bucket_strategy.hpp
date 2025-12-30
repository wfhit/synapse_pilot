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
 * @file manual_bucket_strategy.hpp
 *
 * Manual bucket control strategy with operator input and safety monitoring
 *
 * State Flow:
 *   PRECHECK → INIT → RUNNING (continuous monitoring)
 *                        ↓
 *                  Degraded → Reduced speed operation
 *                  Critical → Failsafe (HOLD or EMERGENCY_STOP)
 *
 * Safety Hierarchy:
 *   1. EMERGENCY_STOP: Hydraulic pressure loss, actuator failure, battery <5%
 *   2. HOLD: Battery <10%, pressure low, position limits exceeded
 *   3. DEGRADED: Battery <15%, pressure warning, approaching limits
 *   4. WARNING: Battery <20%, normal operation with monitoring
 */

#pragma once

#include "../strategy_base.hpp"
#include "manual_bucket_failsafe.hpp"
#include <uORB/topics/strategy_cmd.h>
#include <uORB/topics/operation_mode_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/Subscription.hpp>
#include <lib/mathlib/mathlib.h>

class ManualBucketStrategy : public StrategyBase
{
public:
	// Step definitions
	enum Step : uint8_t {
		STEP_ACTIVATE_MODE = 0,
		STEP_MANUAL_CONTROL = 1,
		STEP_COMPLETE = 2,
		STEP_COUNT = 3
	};

	ManualBucketStrategy() :
		StrategyBase("ManualBucket", strategy_cmd_s::STRATEGY_MANUAL_BUCKET)
	{
		_total_steps = STEP_COUNT;
	}

	// ============ Safety Thresholds ============

	// Battery thresholds (percentage 0.0-1.0)
	static constexpr float BATTERY_MIN_START = 0.15f;      // 15% minimum to start
	static constexpr float BATTERY_WARN = 0.20f;           // 20% warning
	static constexpr float BATTERY_DEGRADED = 0.10f;       // 10% degraded mode
	static constexpr float BATTERY_CRITICAL = 0.05f;       // 5% emergency stop

	// Hydraulic pressure thresholds (bar)
	static constexpr float PRESSURE_MIN = 50.0f;           // 50 bar minimum
	static constexpr float PRESSURE_WARN = 100.0f;         // 100 bar warning
	static constexpr float PRESSURE_CRITICAL = 200.0f;     // 200 bar critical high

	// Bucket position limits (degrees)
	static constexpr float BUCKET_MIN_ANGLE = -30.0f;      // -30° minimum tilt
	static constexpr float BUCKET_MAX_ANGLE = 60.0f;       // 60° maximum tilt
	static constexpr float BUCKET_WARN_MARGIN = 5.0f;      // 5° warning before limit

	// Timeouts (microseconds)
	static constexpr uint64_t CONTROL_INPUT_TIMEOUT = 5000000;  // 5s no input → auto-stop
	static constexpr uint64_t SENSOR_TIMEOUT = 1000000;         // 1s sensor data timeout
	static constexpr uint64_t MODE_SWITCH_TIMEOUT = 500000;     // 500ms mode switch timeout

	StrategyResult precheck() override
	{
		// ========== 1. ARM STATUS ==========
		actuator_armed_s armed;
		if (!_armed_sub.copy(&armed) || !armed.armed) {
			PX4_ERR("ManualBucket: Vehicle not armed");
			return StrategyResult::Failure("Vehicle not armed");
		}

		// ========== 2. BATTERY CHECK ==========
		battery_status_s battery;
		if (!_battery_sub.copy(&battery)) {
			return StrategyResult::Failure("Battery status unavailable");
		}

		if (battery.remaining < BATTERY_MIN_START) {
			PX4_ERR("ManualBucket: Battery too low (%.1f%% < %.1f%%)",
			        (double)(battery.remaining * 100.0f), (double)(BATTERY_MIN_START * 100.0f));
			return StrategyResult::Failure("Battery too low to start");
		}

		// ========== 3. MANUAL CONTROL AVAILABLE ==========
		manual_control_setpoint_s manual_control;
		if (!_manual_control_sub.copy(&manual_control)) {
			return StrategyResult::Failure("Manual control unavailable");
		}

		hrt_abstime now = hrt_absolute_time();
		if (now - manual_control.timestamp > CONTROL_INPUT_TIMEOUT) {
			return StrategyResult::Failure("Manual control input stale");
		}

		// ========== 4. HYDRAULIC SYSTEM CHECK ==========
		// TODO: Check hydraulic pressure sensor
		// For now, assume available if battery is good

		// ========== 5. BUCKET POSITION SENSOR ==========
		// TODO: Verify bucket position encoder is functioning
		// TODO: Check bucket is within safe operating range

		PX4_INFO("ManualBucket: Precheck passed - ready for manual control");

		return StrategyResult::Success();
	}

	StrategyResult init() override
	{
		// Switch to manual bucket operation mode
		command_mode(operation_mode_cmd_s::MODE_WL_MANUAL_BUCKET, 100);

		// Reset state
		_is_degraded = false;
		_last_warn_time = 0;
		_last_input_time = hrt_absolute_time();

		PX4_INFO("ManualBucket: Initialized - manual bucket mode activated");

		return StrategyResult::Success();
	}

	StrategyResult update() override
	{
		hrt_abstime now = hrt_absolute_time();

		switch (_current_step) {
		case STEP_ACTIVATE_MODE:
			// ========== STEP: Wait for manual bucket mode activation ==========
			{
				operation_mode_status_s mode_status;

				if (_mode_status_sub.copy(&mode_status)) {
					if (mode_status.current_mode == operation_mode_cmd_s::MODE_WL_MANUAL_BUCKET) {
						PX4_INFO("ManualBucket: Mode activated - manual control ready");
					_current_step = STEP_MANUAL_CONTROL;
						_control_start_time = get_step_elapsed();
					}
				}
			}
			break;

		case STEP_MANUAL_CONTROL:
			// ========== STEP: Active manual control with safety monitoring ==========
			{
				// Verify mode is still active
				operation_mode_status_s mode_status;
				if (_mode_status_sub.copy(&mode_status)) {
					if (mode_status.current_mode != operation_mode_cmd_s::MODE_WL_MANUAL_BUCKET) {
						return StrategyResult::Failure("Manual bucket mode lost");
					}
				}

				bool degraded_condition = false;

				// --- Battery monitoring ---
				battery_status_s battery;
				if (_battery_sub.copy(&battery)) {
					if (battery.remaining < BATTERY_CRITICAL) {
						PX4_ERR("ManualBucket: CRITICAL - Battery %.1f%%", (double)(battery.remaining * 100.0f));
						return StrategyResult::Failure("Battery critically low");
					} else if (battery.remaining < BATTERY_DEGRADED) {
						PX4_WARN("ManualBucket: Low battery %.1f%% - entering degraded mode",
						         (double)(battery.remaining * 100.0f));
						degraded_condition = true;
					} else if (battery.remaining < BATTERY_WARN) {
						warn_throttled(now, "Battery low: %.1f%%", (double)(battery.remaining * 100.0f));
					}
				}

				// --- Manual control input monitoring ---
				manual_control_setpoint_s manual_control;
				if (_manual_control_sub.copy(&manual_control)) {
					// Check if operator is providing input
					bool has_input = (fabsf(manual_control.roll) > 0.01f ||
					                  fabsf(manual_control.pitch) > 0.01f ||
					                  fabsf(manual_control.throttle) > 0.01f);

					if (has_input) {
						_last_input_time = now;
					}

					// Check for input timeout (operator not providing commands)
					if (now - _last_input_time > CONTROL_INPUT_TIMEOUT) {
						PX4_WARN("ManualBucket: No operator input for %.1fs - completing strategy",
						         (double)((now - _last_input_time) / 1e6));
					_current_step = STEP_COMPLETE;
						break;
					}
				} else {
					return StrategyResult::Failure("Manual control input lost");
				}

				// --- Hydraulic pressure monitoring ---
				// TODO: Implement when hydraulic pressure sensor is available
				// float hydraulic_pressure = get_hydraulic_pressure();
				// if (hydraulic_pressure < PRESSURE_MIN) {
				//     PX4_ERR("ManualBucket: Hydraulic pressure too low: %.1f bar", hydraulic_pressure);
				//     return StrategyResult::Failure("Hydraulic pressure critical");
				// }

				// --- Bucket position monitoring ---
				// TODO: Implement when bucket position encoder is available
				// float bucket_angle = get_bucket_angle();
				// if (bucket_angle < BUCKET_MIN_ANGLE || bucket_angle > BUCKET_MAX_ANGLE) {
				//     PX4_ERR("ManualBucket: Bucket position limit exceeded: %.1f°", bucket_angle);
				//     return StrategyResult::Failure("Bucket position limit");
				// }

				// --- Switch monitoring (check for mode change request) ---
				manual_control_switches_s switches;
				if (_switches_sub.copy(&switches)) {
					// Check if operator requested mode change via switch
					// TODO: Define specific switch for exiting manual mode
					if (switches.return_switch == manual_control_switches_s::SWITCH_POS_ON) {
						PX4_INFO("ManualBucket: Return switch activated - exiting manual mode");
					_current_step = STEP_COMPLETE;
						break;
					}
				}

				// --- Apply degraded mode if needed ---
				if (degraded_condition && !_is_degraded) {
					_is_degraded = true;
					PX4_WARN("ManualBucket: Entering DEGRADED mode - reduced response");
					// TODO: Command reduced hydraulic flow rate
				} else if (!degraded_condition && _is_degraded) {
					_is_degraded = false;
					PX4_INFO("ManualBucket: Exiting DEGRADED mode - normal response");
					// TODO: Command normal hydraulic flow rate
				}

				// --- Check for operator-requested completion ---
				// Strategy continues indefinitely until operator exits or timeout
			}
			break;

		case STEP_COMPLETE:
			// ========== STEP: Manual control complete - success ==========
			PX4_INFO("ManualBucket: Manual bucket control completed");
			// Transition to IDLE - strategy complete
			return StrategyResult::Success();
		}

		// Still running
		return StrategyResult::Success();
	}

	const char *get_step_name() const override
	{
		const char *names[] = {"Activate Mode", "Manual Control", "Complete"};
		return names[_current_step];
	}

protected:
	FailsafeBase *create_failsafe() override
	{
		return new ManualBucketFailsafe();
	}

private:
	// Subscriptions
	uORB::Subscription _mode_status_sub{ORB_ID(operation_mode_status)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _switches_sub{ORB_ID(manual_control_switches)};

	// State tracking
	bool _is_degraded{false};
	hrt_abstime _last_warn_time{0};
	hrt_abstime _last_input_time{0};
	hrt_abstime _control_start_time{0};

	/**
	 * Throttled warning (max once per second)
	 */
	void warn_throttled(hrt_abstime now, const char *fmt, ...)
	{
		if (now - _last_warn_time > 1_s) {
			va_list args;
			va_start(args, fmt);
			px4_vlog(PX4_LOG_LEVEL_WARN, "ManualBucket", __LINE__, fmt, args);
			va_end(args);
			_last_warn_time = now;
		}
	}
};
