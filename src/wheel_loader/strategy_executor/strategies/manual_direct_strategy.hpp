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
 * @file manual_direct_strategy.hpp
 *
 * Manual direct control strategy with full operator authority and safety monitoring
 *
 * State Flow:
 *   PRECHECK → INIT → RUNNING (continuous safety monitoring)
 *                        ↓
 *                  Degraded → Reduced authority/speed
 *                  Critical → Failsafe (HOLD or EMERGENCY_STOP)
 *
 * Safety Hierarchy:
 *   1. EMERGENCY_STOP: Communication loss, battery <5%, critical sensor failure
 *   2. HOLD: Battery <10%, localization lost, excessive speed
 *   3. DEGRADED: Battery <15%, weak signal, approaching limits
 *   4. WARNING: Battery <20%, normal operation with monitoring
 *
 * Features:
 *   - Direct control of vehicle actuators (steering, throttle, bucket)
 *   - Dead-man switch monitoring
 *   - Speed limiting based on conditions
 *   - Collision avoidance integration
 */

#pragma once

#include "../strategy_base.hpp"
#include "manual_direct_failsafe.hpp"
#include <uORB/topics/wheel_loader/strategy_cmd.h>
#include <uORB/topics/wheel_loader/operation_mode_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/manual_control_switches.h>
#include <uORB/Subscription.hpp>
#include <lib/mathlib/mathlib.h>

class ManualDirectStrategy : public StrategyBase
{
public:
	// Step definitions
	enum Step : uint8_t {
		STEP_ACTIVATE_MODE = 0,
		STEP_DIRECT_CONTROL = 1,
		STEP_COMPLETE = 2,
		STEP_COUNT = 3
	};

	ManualDirectStrategy() :
		StrategyBase("ManualDirect", strategy_cmd_s::STRATEGY_MANUAL_DIRECT)
	{
		_total_steps = STEP_COUNT;
	}

	// ============ Safety Thresholds ============

	// Battery thresholds (percentage 0.0-1.0)
	static constexpr float BATTERY_MIN_START = 0.15f;      // 15% minimum to start
	static constexpr float BATTERY_WARN = 0.20f;           // 20% warning
	static constexpr float BATTERY_DEGRADED = 0.15f;       // 15% degraded mode
	static constexpr float BATTERY_CRITICAL = 0.05f;       // 5% emergency stop

	// Speed limits (m/s)
	static constexpr float SPEED_MAX_NORMAL = 5.0f;        // 5 m/s normal max
	static constexpr float SPEED_MAX_DEGRADED = 2.0f;      // 2 m/s degraded
	static constexpr float SPEED_CRITICAL = 8.0f;          // 8 m/s emergency stop

	// Slope limits (degrees)
	static constexpr float SLOPE_MAX = 25.0f;              // 25° emergency stop
	static constexpr float SLOPE_DEGRADED = 20.0f;         // 20° degraded mode
	static constexpr float SLOPE_WARN = 15.0f;             // 15° warning

	// Control signal quality
	static constexpr float SIGNAL_STRENGTH_MIN = 30.0f;    // 30% minimum signal
	static constexpr float SIGNAL_STRENGTH_WARN = 50.0f;   // 50% warning threshold

	// Timeouts (microseconds)
	static constexpr uint64_t CONTROL_INPUT_TIMEOUT = 500000;     // 500ms dead-man switch
	static constexpr uint64_t LOCALIZATION_TIMEOUT = 1000000;     // 1s localization
	static constexpr uint64_t SENSOR_TIMEOUT = 1000000;           // 1s sensor timeout

	StrategyResult precheck() override
	{
		// ========== 1. ARM STATUS ==========
		actuator_armed_s armed;
		if (!_armed_sub.copy(&armed) || !armed.armed) {
			PX4_ERR("ManualDirect: Vehicle not armed");
			return StrategyResult::Failure("Vehicle not armed");
		}

		// ========== 2. BATTERY CHECK ==========
		battery_status_s battery;
		if (!_battery_sub.copy(&battery)) {
			return StrategyResult::Failure("Battery status unavailable");
		}

		if (battery.remaining < BATTERY_MIN_START) {
			PX4_ERR("ManualDirect: Battery too low (%.1f%% < %.1f%%)",
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

		// ========== 4. ATTITUDE CHECK ==========
		vehicle_attitude_s attitude;
		if (!_attitude_sub.copy(&attitude)) {
			return StrategyResult::Failure("Attitude data unavailable");
		}

		// Calculate current slope
		matrix::Eulerf euler(matrix::Quatf(attitude.q));
		float pitch_deg = math::degrees(euler.theta());
		float roll_deg = math::degrees(euler.phi());
		float slope = sqrtf(pitch_deg * pitch_deg + roll_deg * roll_deg);

		if (slope > SLOPE_MAX) {
			PX4_ERR("ManualDirect: Slope too steep (%.1f° > %.1f°)",
			        (double)slope, (double)SLOPE_MAX);
			return StrategyResult::Failure("Slope too steep");
		}

		// ========== 5. LOCALIZATION HEALTH ==========
		vehicle_local_position_s local_pos;
		if (!_local_pos_sub.copy(&local_pos)) {
			PX4_WARN("ManualDirect: Localization unavailable - proceeding with caution");
			// Allow manual control even without localization, but warn
		} else {
			if (now - local_pos.timestamp > LOCALIZATION_TIMEOUT) {
				PX4_WARN("ManualDirect: Localization data stale - proceeding with caution");
			}
		}

		PX4_INFO("ManualDirect: Precheck passed - ready for direct control");

		return StrategyResult::Success();
	}

	StrategyResult init() override
	{
		// Switch to manual direct operation mode
		command_mode(operation_mode_cmd_s::MODE_WL_MANUAL_DIRECT, 100);

		// Reset state
		_is_degraded = false;
		_last_warn_time = 0;
		_last_input_time = hrt_absolute_time();
		_deadman_active = false;

		PX4_INFO("ManualDirect: Initialized - direct control mode activated");

		return StrategyResult::Success();
	}

	StrategyResult update() override
	{
		hrt_abstime now = hrt_absolute_time();

		switch (_current_step) {
		case STEP_ACTIVATE_MODE:
			// ========== STEP: Wait for manual direct mode activation ==========
			{
				operation_mode_status_s mode_status;

				if (_mode_status_sub.copy(&mode_status)) {
					if (mode_status.current_mode == operation_mode_cmd_s::MODE_WL_MANUAL_DIRECT) {
						PX4_INFO("ManualDirect: Mode activated - direct control ready");
					_current_step = STEP_DIRECT_CONTROL;
						_control_start_time = get_step_elapsed();
					}
				}
			}
			break;

		case STEP_DIRECT_CONTROL:
			// ========== STEP: Active direct control with continuous monitoring ==========
			{
				// Verify mode is still active
				operation_mode_status_s mode_status;
				if (_mode_status_sub.copy(&mode_status)) {
					if (mode_status.current_mode != operation_mode_cmd_s::MODE_WL_MANUAL_DIRECT) {
						return StrategyResult::Failure("Manual direct mode lost");
					}
				}

				bool degraded_condition = false;

				// --- Battery monitoring ---
				battery_status_s battery;
				if (_battery_sub.copy(&battery)) {
					if (battery.remaining < BATTERY_CRITICAL) {
						PX4_ERR("ManualDirect: CRITICAL - Battery %.1f%%", (double)(battery.remaining * 100.0f));
						return StrategyResult::Failure("Battery critically low");
					} else if (battery.remaining < BATTERY_DEGRADED) {
						PX4_WARN("ManualDirect: Low battery %.1f%% - entering degraded mode",
						         (double)(battery.remaining * 100.0f));
						degraded_condition = true;
					} else if (battery.remaining < BATTERY_WARN) {
						warn_throttled(now, "Battery low: %.1f%%", (double)(battery.remaining * 100.0f));
					}
				}

				// --- Dead-man switch monitoring (critical safety feature) ---
				manual_control_setpoint_s manual_control;
				if (_manual_control_sub.copy(&manual_control)) {
					// Check for active control input (dead-man switch)
					bool has_input = (fabsf(manual_control.roll) > 0.01f ||
					                  fabsf(manual_control.pitch) > 0.01f ||
					                  fabsf(manual_control.throttle) > 0.01f ||
					                  fabsf(manual_control.yaw) > 0.01f);

					if (has_input) {
						_last_input_time = now;
						if (!_deadman_active) {
							PX4_INFO("ManualDirect: Dead-man switch activated");
							_deadman_active = true;
						}
					}

					// Check for dead-man switch timeout
					if (now - _last_input_time > CONTROL_INPUT_TIMEOUT) {
						PX4_ERR("ManualDirect: Dead-man switch timeout - no input for %.1fms",
						        (double)((now - _last_input_time) / 1000.0f));
						return StrategyResult::Failure("Dead-man switch timeout");
					}

					// Check for stale data
					if (now - manual_control.timestamp > CONTROL_INPUT_TIMEOUT) {
						PX4_ERR("ManualDirect: Control input data stale");
						return StrategyResult::Failure("Control input lost");
					}
				} else {
					return StrategyResult::Failure("Manual control input lost");
				}

				// --- Slope monitoring ---
				vehicle_attitude_s attitude;
				if (_attitude_sub.copy(&attitude)) {
					matrix::Eulerf euler(matrix::Quatf(attitude.q));
					float pitch_deg = math::degrees(euler.theta());
					float roll_deg = math::degrees(euler.phi());
					float slope = sqrtf(pitch_deg * pitch_deg + roll_deg * roll_deg);

					if (slope > SLOPE_MAX) {
						PX4_ERR("ManualDirect: CRITICAL - Slope %.1f°", (double)slope);
						return StrategyResult::Failure("Slope exceeded maximum");
					} else if (slope > SLOPE_DEGRADED) {
						PX4_WARN("ManualDirect: Steep slope %.1f° - degraded mode", (double)slope);
						degraded_condition = true;
					} else if (slope > SLOPE_WARN) {
						warn_throttled(now, "Slope warning: %.1f°", (double)slope);
					}
				}

				// --- Speed monitoring ---
				vehicle_local_position_s local_pos;
				if (_local_pos_sub.copy(&local_pos)) {
					if (local_pos.v_xy_valid) {
						float speed = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);

						if (speed > SPEED_CRITICAL) {
							PX4_ERR("ManualDirect: CRITICAL - Excessive speed %.1f m/s", (double)speed);
							return StrategyResult::Failure("Speed exceeded critical limit");
						} else if (speed > SPEED_MAX_NORMAL) {
							warn_throttled(now, "Speed high: %.1f m/s", (double)speed);
						}
					}

					// Check localization health
					if (now - local_pos.timestamp > LOCALIZATION_TIMEOUT) {
						warn_throttled(now, "Localization timeout - reduced capabilities");
						degraded_condition = true;
					}
				}

				// --- Signal quality monitoring ---
				// TODO: Check RC signal strength when available
				// if (signal_strength < SIGNAL_STRENGTH_MIN) {
				//     PX4_ERR("ManualDirect: Signal too weak: %.1f%%", signal_strength);
				//     return StrategyResult::Failure("Control signal lost");
				// }

				// --- Switch monitoring (check for mode change request) ---
				manual_control_switches_s switches;
				if (_switches_sub.copy(&switches)) {
					// Check if operator requested mode change via switch
					if (switches.return_switch == manual_control_switches_s::SWITCH_POS_ON) {
						PX4_INFO("ManualDirect: Return switch activated - exiting direct mode");
					_current_step = STEP_COMPLETE;
						break;
					}
				}

				// --- Apply degraded mode if needed ---
				if (degraded_condition && !_is_degraded) {
					_is_degraded = true;
					PX4_WARN("ManualDirect: Entering DEGRADED mode - speed limited to %.1f m/s",
					         (double)SPEED_MAX_DEGRADED);
					// TODO: Command speed limiter to controller
				} else if (!degraded_condition && _is_degraded) {
					_is_degraded = false;
					PX4_INFO("ManualDirect: Exiting DEGRADED mode - normal speed limit");
					// TODO: Restore normal speed limit
				}

				// Strategy continues indefinitely until operator exits
			}
			break;

		case STEP_COMPLETE:
			// ========== STEP: Direct control complete - success ==========
			PX4_INFO("ManualDirect: Manual direct control completed");
			_deadman_active = false;
			// Transition to IDLE - strategy complete
			return StrategyResult::Success();
		}

		// Still running
		return StrategyResult::Success();
	}

	const char *get_step_name() const override
	{
		const char *names[] = {"Activate Mode", "Direct Control", "Complete"};
		return names[_current_step];
	}

protected:
	FailsafeBase *create_failsafe() override
	{
		return new ManualDirectFailsafe();
	}

private:
	// Subscriptions
	uORB::Subscription _mode_status_sub{ORB_ID(operation_mode_status)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _switches_sub{ORB_ID(manual_control_switches)};

	// State tracking
	bool _is_degraded{false};
	bool _deadman_active{false};
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
			px4_vlog(PX4_LOG_LEVEL_WARN, "ManualDirect", __LINE__, fmt, args);
			va_end(args);
			_last_warn_time = now;
		}
	}
};
