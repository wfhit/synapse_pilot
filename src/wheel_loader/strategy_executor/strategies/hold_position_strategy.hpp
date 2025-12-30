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
 * @file hold_position_strategy.hpp
 *
 * Hold position strategy - maintains current position and heading
 *
 * State Flow:
 *   PRECHECK → INIT → HOLDING (continuous position control)
 *                        ↓
 *                  Degraded → Reduced precision
 *                  Critical → Failsafe (EMERGENCY_STOP)
 *
 * Safety Hierarchy:
 *   1. EMERGENCY_STOP: Localization lost, battery <5%, excessive drift
 *   2. HOLD: Battery <10%, sensor degradation, position error >2m
 *   3. DEGRADED: Battery <15%, weak GPS, position error >0.5m
 *   4. WARNING: Battery <20%, normal hold with monitoring
 *
 * Use Cases:
 *   - Emergency stop during autonomous operations
 *   - Pause for obstacle clearance
 *   - Operator intervention hold
 *   - System diagnostics while stationary
 *   - Failsafe recovery position
 */

#pragma once

#include "../strategy_base.hpp"
#include "hold_position_failsafe.hpp"
#include <uORB/topics/strategy_cmd.h>
#include <uORB/topics/operation_mode_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/Subscription.hpp>
#include <lib/mathlib/mathlib.h>

class HoldPositionStrategy : public StrategyBase
{
public:
	// Step definitions
	enum Step : uint8_t {
		STEP_ACTIVATE_MODE = 0,
		STEP_CAPTURE_POSITION = 1,
		STEP_HOLDING = 2,
		STEP_COMPLETE = 3,
		STEP_COUNT = 4
	};

	HoldPositionStrategy() :
		StrategyBase("HoldPosition", strategy_cmd_s::STRATEGY_HOLD)
	{
		_total_steps = STEP_COUNT;
	}

	// ============ Safety Thresholds ============

	// Battery thresholds (percentage 0.0-1.0)
	static constexpr float BATTERY_MIN_START = 0.10f;      // 10% minimum to start
	static constexpr float BATTERY_WARN = 0.20f;           // 20% warning
	static constexpr float BATTERY_DEGRADED = 0.15f;       // 15% degraded mode
	static constexpr float BATTERY_CRITICAL = 0.05f;       // 5% emergency stop

	// Position error thresholds (meters)
	static constexpr float POSITION_ERROR_MAX = 2.0f;      // 2m max deviation
	static constexpr float POSITION_ERROR_DEGRADED = 0.5f; // 0.5m degraded threshold
	static constexpr float POSITION_ERROR_WARN = 0.3f;     // 0.3m warning

	// Heading error thresholds (degrees)
	static constexpr float HEADING_ERROR_MAX = 30.0f;      // 30° max deviation
	static constexpr float HEADING_ERROR_WARN = 15.0f;     // 15° warning

	// Velocity thresholds (m/s) - should be near zero when holding
	static constexpr float VELOCITY_HOLD_TARGET = 0.0f;    // Target velocity
	static constexpr float VELOCITY_HOLD_MAX = 0.2f;       // 0.2 m/s max while holding

	// Timeouts (microseconds)
	static constexpr uint64_t LOCALIZATION_TIMEOUT = 500000;   // 500ms
	static constexpr uint64_t SENSOR_TIMEOUT = 1000000;        // 1s
	static constexpr uint64_t POSITION_CAPTURE_TIMEOUT = 2000000; // 2s

	StrategyResult precheck() override
	{
		// ========== 1. ARM STATUS ==========
		actuator_armed_s armed;
		if (!_armed_sub.copy(&armed) || !armed.armed) {
			PX4_ERR("HoldPosition: Vehicle not armed");
			return StrategyResult::Failure("Vehicle not armed");
		}

		// ========== 2. BATTERY CHECK ==========
		battery_status_s battery;
		if (!_battery_sub.copy(&battery)) {
			return StrategyResult::Failure("Battery status unavailable");
		}

		if (battery.remaining < BATTERY_MIN_START) {
			PX4_ERR("HoldPosition: Battery too low (%.1f%% < %.1f%%)",
			        (double)(battery.remaining * 100.0f), (double)(BATTERY_MIN_START * 100.0f));
			return StrategyResult::Failure("Battery too low to hold");
		}

		// ========== 3. LOCALIZATION HEALTH ==========
		vehicle_local_position_s local_pos;
		if (!_local_pos_sub.copy(&local_pos)) {
			return StrategyResult::Failure("Localization unavailable");
		}

		if (!local_pos.xy_valid || !local_pos.v_xy_valid) {
			return StrategyResult::Failure("Localization not valid");
		}

		hrt_abstime now = hrt_absolute_time();
		if (now - local_pos.timestamp > LOCALIZATION_TIMEOUT) {
			return StrategyResult::Failure("Localization data stale");
		}

		// ========== 4. ATTITUDE CHECK ==========
		vehicle_attitude_s attitude;
		if (!_attitude_sub.copy(&attitude)) {
			return StrategyResult::Failure("Attitude data unavailable");
		}

		// ========== 5. SENSOR HEALTH ==========
		sensor_combined_s sensors;
		if (!_sensor_sub.copy(&sensors)) {
			return StrategyResult::Failure("Sensor data unavailable");
		}

		if (now - sensors.timestamp > SENSOR_TIMEOUT) {
			return StrategyResult::Failure("Sensor data stale");
		}

		PX4_INFO("HoldPosition: Precheck passed - ready to hold position");

		return StrategyResult::Success();
	}

	StrategyResult init() override
	{
		// Switch to hold position operation mode
		command_mode(operation_mode_cmd_s::MODE_WL_HOLD, 100);

		// Reset state
		_is_degraded = false;
		_last_warn_time = 0;
		_position_captured = false;
		_hold_x = 0.0f;
		_hold_y = 0.0f;
		_hold_z = 0.0f;
		_hold_yaw = 0.0f;

		PX4_INFO("HoldPosition: Initialized - preparing to hold");

		return StrategyResult::Success();
	}

	StrategyResult update() override
	{
		hrt_abstime now = hrt_absolute_time();

		switch (_current_step) {
		case STEP_ACTIVATE_MODE:
			// ========== STEP: Wait for hold mode activation ==========
			{
				operation_mode_status_s mode_status;

				if (_mode_status_sub.copy(&mode_status)) {
					if (mode_status.current_mode == operation_mode_cmd_s::MODE_WL_HOLD) {
						PX4_INFO("HoldPosition: Mode activated - capturing position");
					_current_step = STEP_CAPTURE_POSITION;
					}
				}
			}
			break;

		case STEP_CAPTURE_POSITION:
			// ========== STEP: Capture current position to hold ==========
			{
				vehicle_local_position_s local_pos;
				if (_local_pos_sub.copy(&local_pos)) {
					if (local_pos.xy_valid && local_pos.z_valid) {
						_hold_x = local_pos.x;
						_hold_y = local_pos.y;
						_hold_z = local_pos.z;

						vehicle_attitude_s attitude;
						if (_attitude_sub.copy(&attitude)) {
							matrix::Eulerf euler(matrix::Quatf(attitude.q));
							_hold_yaw = euler.psi();
						}

						_position_captured = true;
						_capture_time = now;

						PX4_INFO("HoldPosition: Position captured - X:%.2f Y:%.2f Z:%.2f Yaw:%.1f°",
						         (double)_hold_x, (double)_hold_y, (double)_hold_z,
						         (double)math::degrees(_hold_yaw));

					_current_step = STEP_HOLDING;
					}
				}

				// Timeout check
				if (now - get_step_start() > POSITION_CAPTURE_TIMEOUT) {
					return StrategyResult::Failure("Failed to capture position");
				}
			}
			break;

		case STEP_HOLDING:
			// ========== STEP: Active position holding with monitoring ==========
			{
				// Verify mode is still active
				operation_mode_status_s mode_status;
				if (_mode_status_sub.copy(&mode_status)) {
					if (mode_status.current_mode != operation_mode_cmd_s::MODE_WL_HOLD) {
						return StrategyResult::Failure("Hold mode lost");
					}
				}

				bool degraded_condition = false;

				// --- Battery monitoring ---
				battery_status_s battery;
				if (_battery_sub.copy(&battery)) {
					if (battery.remaining < BATTERY_CRITICAL) {
						PX4_ERR("HoldPosition: CRITICAL - Battery %.1f%%", (double)(battery.remaining * 100.0f));
						return StrategyResult::Failure("Battery critically low");
					} else if (battery.remaining < BATTERY_DEGRADED) {
						warn_throttled(now, "Low battery %.1f%% - degraded mode",
						               (double)(battery.remaining * 100.0f));
						degraded_condition = true;
					} else if (battery.remaining < BATTERY_WARN) {
						warn_throttled(now, "Battery low: %.1f%%", (double)(battery.remaining * 100.0f));
					}
				}

				// --- Position error monitoring ---
				vehicle_local_position_s local_pos;
				if (_local_pos_sub.copy(&local_pos)) {
					// Check localization health
					if (!local_pos.xy_valid || !local_pos.z_valid) {
						PX4_ERR("HoldPosition: Localization lost");
						return StrategyResult::Failure("Localization invalid");
					}

					if (now - local_pos.timestamp > LOCALIZATION_TIMEOUT) {
						PX4_ERR("HoldPosition: Localization timeout");
						return StrategyResult::Failure("Localization timeout");
					}

					// Calculate position error
					float dx = local_pos.x - _hold_x;
					float dy = local_pos.y - _hold_y;
					float dz = local_pos.z - _hold_z;
					float position_error = sqrtf(dx * dx + dy * dy);
					float altitude_error = fabsf(dz);

					if (position_error > POSITION_ERROR_MAX) {
						PX4_ERR("HoldPosition: CRITICAL - Position error %.2fm", (double)position_error);
						return StrategyResult::Failure("Excessive position drift");
					} else if (position_error > POSITION_ERROR_DEGRADED) {
						warn_throttled(now, "Position error %.2fm - degraded mode", (double)position_error);
						degraded_condition = true;
					} else if (position_error > POSITION_ERROR_WARN) {
						warn_throttled(now, "Position error: %.2fm", (double)position_error);
					}

					// Check velocity (should be near zero)
					if (local_pos.v_xy_valid) {
						float velocity = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);
						if (velocity > VELOCITY_HOLD_MAX) {
							warn_throttled(now, "Velocity high while holding: %.2f m/s", (double)velocity);
							degraded_condition = true;
						}
					}
				}

				// --- Heading error monitoring ---
				vehicle_attitude_s attitude;
				if (_attitude_sub.copy(&attitude)) {
					matrix::Eulerf euler(matrix::Quatf(attitude.q));
					float current_yaw = euler.psi();
					float yaw_error = fabsf(math::degrees(current_yaw - _hold_yaw));

					// Normalize to 0-180 degrees
					if (yaw_error > 180.0f) {
						yaw_error = 360.0f - yaw_error;
					}

					if (yaw_error > HEADING_ERROR_MAX) {
						warn_throttled(now, "Heading error: %.1f°", (double)yaw_error);
						degraded_condition = true;
					} else if (yaw_error > HEADING_ERROR_WARN) {
						warn_throttled(now, "Heading drift: %.1f°", (double)yaw_error);
					}
				}

				// --- Sensor health ---
				sensor_combined_s sensors;
				if (_sensor_sub.copy(&sensors)) {
					if (now - sensors.timestamp > SENSOR_TIMEOUT) {
						PX4_ERR("HoldPosition: Sensor timeout");
						return StrategyResult::Failure("Sensor data lost");
					}
				}

				// --- Apply degraded mode if needed ---
				if (degraded_condition && !_is_degraded) {
					_is_degraded = true;
					PX4_WARN("HoldPosition: Entering DEGRADED mode - reduced precision");
					// TODO: Notify controller of degraded hold (looser position tolerance)
				} else if (!degraded_condition && _is_degraded) {
					_is_degraded = false;
					PX4_INFO("HoldPosition: Exiting DEGRADED mode - normal precision");
					// TODO: Restore normal hold precision
				}

				// Hold continues indefinitely until commanded to stop
				// or preempted by another strategy
			}
			break;

		case STEP_COMPLETE:
			// ========== STEP: Hold released - complete ==========
			PX4_INFO("HoldPosition: Hold position strategy completed");
			return StrategyResult::Success();
		}

		// Still running
		return StrategyResult::Success();
	}

	const char *get_step_name() const override
	{
		const char *names[] = {"Activate Mode", "Capture Position", "Holding", "Complete"};
		return names[_current_step];
	}

protected:
	FailsafeBase *create_failsafe() override
	{
		return new HoldPositionFailsafe();
	}

private:
	// Subscriptions
	uORB::Subscription _mode_status_sub{ORB_ID(operation_mode_status)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _sensor_sub{ORB_ID(sensor_combined)};

	// Hold position state
	bool _is_degraded{false};
	bool _position_captured{false};
	hrt_abstime _last_warn_time{0};
	hrt_abstime _capture_time{0};

	// Target hold position
	float _hold_x{0.0f};
	float _hold_y{0.0f};
	float _hold_z{0.0f};
	float _hold_yaw{0.0f};

	/**
	 * Throttled warning (max once per second)
	 */
	void warn_throttled(hrt_abstime now, const char *fmt, ...)
	{
		if (now - _last_warn_time > 1_s) {
			va_list args;
			va_start(args, fmt);
			px4_vlog(PX4_LOG_LEVEL_WARN, "HoldPosition", __LINE__, fmt, args);
			va_end(args);
			_last_warn_time = now;
		}
	}
};
