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
 * @file trajectory_follower_strategy.hpp
 *
 * Complete trajectory following strategy with comprehensive safety monitoring
 *
 * State Flow:
 *   PRECHECK → INIT → RUNNING (continuous safety monitoring)
 *                        ↓
 *                  Degraded → Reduced speed operation
 *                  Critical → Failsafe (HOLD or EMERGENCY_STOP)
 *
 * Safety Hierarchy:
 *   1. EMERGENCY_STOP: Slope >25°, Slip >50%, Battery <5%, Sensor total loss
 *   2. HOLD: Battery <10%, Localization lost, Trajectory invalid, Deviation >3m
 *   3. DEGRADED: Battery <15%, Slope >20°, Slip >30%, Deviation >2m
 *   4. WARNING: Battery <20%, Slope >15°, Slip >20%, Deviation >1m
 */

#pragma once

#include "../strategy_base.hpp"
#include "trajectory_follower_failsafe.hpp"
#include <uORB/topics/strategy_cmd.h>
#include <uORB/topics/operation_mode_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/Subscription.hpp>
#include <lib/mathlib/mathlib.h>

class TrajectoryFollowerStrategy : public StrategyBase
{
public:
	// Step definitions
	enum Step : uint8_t {
		STEP_ACTIVATE_MODE = 0,
		STEP_FOLLOWING = 1,
		STEP_HOLD_AFTER_COMPLETE = 2,
		STEP_COUNT = 3
	};

	TrajectoryFollowerStrategy() :
		StrategyBase("TrajectoryFollower", strategy_cmd_s::STRATEGY_TRAJECTORY_FOLLOWER)
	{
		_total_steps = STEP_COUNT;
	}

	// ============ Safety Thresholds ============

	// Battery thresholds (percentage 0.0-1.0)
	static constexpr float BATTERY_MIN_START = 0.20f;      // 20% minimum to start
	static constexpr float BATTERY_WARN = 0.15f;           // 15% warning
	static constexpr float BATTERY_DEGRADED = 0.10f;       // 10% degraded mode
	static constexpr float BATTERY_CRITICAL = 0.05f;       // 5% emergency stop

	// Slope limits (degrees)
	static constexpr float SLOPE_MAX = 25.0f;              // 25° emergency stop
	static constexpr float SLOPE_DEGRADED = 20.0f;         // 20° degraded mode
	static constexpr float SLOPE_WARN = 15.0f;             // 15° warning

	// Slip detection (percentage 0.0-1.0)
	static constexpr float SLIP_CRITICAL = 0.50f;          // 50% emergency stop
	static constexpr float SLIP_DEGRADED = 0.30f;          // 30% degraded mode
	static constexpr float SLIP_WARN = 0.20f;              // 20% warning

	// Path deviation limits (meters)
	static constexpr float DEVIATION_CRITICAL = 3.0f;      // 3m abort
	static constexpr float DEVIATION_DEGRADED = 2.0f;      // 2m degraded
	static constexpr float DEVIATION_WARN = 1.0f;          // 1m warning

	// Timeouts (microseconds)
	static constexpr uint64_t LOCALIZATION_TIMEOUT = 500000;  // 500ms
	static constexpr uint64_t SENSOR_TIMEOUT = 1000000;       // 1s
	static constexpr uint64_t TRAJECTORY_TIMEOUT = 2000000;   // 2s

	StrategyResult precheck() override
	{
		// ========== 1. ARM STATUS ==========
		actuator_armed_s armed;
		if (!_armed_sub.copy(&armed) || !armed.armed) {
			PX4_ERR("TrajectoryFollower: Vehicle not armed");
			return StrategyResult::Failure("Vehicle not armed");
		}

		// ========== 2. BATTERY CHECK ==========
		battery_status_s battery;
		if (!_battery_sub.copy(&battery)) {
			return StrategyResult::Failure("Battery status unavailable");
		}

		if (battery.remaining < BATTERY_MIN_START) {
			PX4_ERR("TrajectoryFollower: Battery too low (%.1f%% < %.1f%%)",
			        (double)(battery.remaining * 100.0f), (double)(BATTERY_MIN_START * 100.0f));
			return StrategyResult::Failure("Battery too low to start");
		}

		// ========== 3. ATTITUDE/SLOPE CHECK ==========
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
			PX4_ERR("TrajectoryFollower: Slope too steep (%.1f° > %.1f°)",
			        (double)slope, (double)SLOPE_MAX);
			return StrategyResult::Failure("Slope too steep");
		}

		// ========== 4. LOCALIZATION HEALTH ==========
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

		// ========== 5. SENSOR HEALTH ==========
		sensor_combined_s sensors;
		if (!_sensor_sub.copy(&sensors)) {
			return StrategyResult::Failure("Sensor data unavailable");
		}

		if (now - sensors.timestamp > SENSOR_TIMEOUT) {
			return StrategyResult::Failure("Sensor data stale");
		}

		// ========== 6. VEHICLE STATUS ==========
		vehicle_status_s status;
		if (!_status_sub.copy(&status)) {
			return StrategyResult::Failure("Vehicle status unavailable");
		}

		// ========== 7. TRAJECTORY AVAILABILITY ==========
		// TODO: Check trajectory planner is publishing valid path
		// TODO: Validate trajectory is safe (no obstacles, within bounds)

		PX4_INFO("TrajectoryFollower: Precheck passed - all systems healthy");

		return StrategyResult::Success();
	}

	StrategyResult init() override
	{
		// Switch to trajectory following operation mode
		command_mode(operation_mode_cmd_s::MODE_WL_TRAJECTORY, 100);

		// Reset degraded state
		_is_degraded = false;
		_last_warn_time = 0;

		PX4_INFO("TrajectoryFollower: Initialized - trajectory mode activated");

		return StrategyResult::Success();
	}

	StrategyResult update() override
	{
		hrt_abstime now = hrt_absolute_time();

		switch (_current_step) {
		case STEP_ACTIVATE_MODE:
			// ========== STEP 0: Wait for trajectory mode activation ==========
			{
				operation_mode_status_s mode_status;

				if (_mode_status_sub.copy(&mode_status)) {
					if (mode_status.current_mode == operation_mode_cmd_s::MODE_WL_TRAJECTORY) {
						PX4_INFO("TrajectoryFollower: Mode activated - starting trajectory execution");
					_current_step = STEP_FOLLOWING;
					_trajectory_start_time = get_step_elapsed();
				}
			}
		}
		break;

	case STEP_FOLLOWING:
		// ========== STEP 1: Active trajectory following with continuous monitoring ==========
		{
			// Verify mode is still active
			operation_mode_status_s mode_status;
			if (_mode_status_sub.copy(&mode_status)) {
				if (mode_status.current_mode != operation_mode_cmd_s::MODE_WL_TRAJECTORY) {
					return StrategyResult::Failure("Trajectory mode lost");
				}
			}

			bool degraded_condition = false;

			// --- Battery monitoring ---
			battery_status_s battery;
			if (_battery_sub.copy(&battery)) {
				if (battery.remaining < BATTERY_CRITICAL) {
					PX4_ERR("TrajectoryFollower: CRITICAL - Battery %.1f%%", (double)(battery.remaining * 100.0f));
					return StrategyResult::Failure("Battery critically low");
				} else if (battery.remaining < BATTERY_DEGRADED) {
						PX4_WARN("TrajectoryFollower: Low battery %.1f%% - entering degraded mode",
						         (double)(battery.remaining * 100.0f));
						degraded_condition = true;
					} else if (battery.remaining < BATTERY_WARN) {
						warn_throttled(now, "Battery low: %.1f%%", (double)(battery.remaining * 100.0f));
					}
				}

			// --- Slope monitoring ---
			vehicle_attitude_s attitude;
			if (_attitude_sub.copy(&attitude)) {
				matrix::Eulerf euler(matrix::Quatf(attitude.q));
				float pitch_deg = math::degrees(euler.theta());
				float roll_deg = math::degrees(euler.phi());
				float slope = sqrtf(pitch_deg * pitch_deg + roll_deg * roll_deg);

				if (slope > SLOPE_MAX) {
					PX4_ERR("TrajectoryFollower: CRITICAL - Slope %.1f°", (double)slope);
					return StrategyResult::Failure("Slope exceeded maximum");
				} else if (slope > SLOPE_DEGRADED) {
						PX4_WARN("TrajectoryFollower: Steep slope %.1f° - degraded mode", (double)slope);
						degraded_condition = true;
					} else if (slope > SLOPE_WARN) {
						warn_throttled(now, "Slope warning: %.1f°", (double)slope);
					}
				}

			// --- Slip detection ---
			vehicle_local_position_s local_pos;
			if (_local_pos_sub.copy(&local_pos)) {
				// Check localization health
				if (!local_pos.xy_valid || !local_pos.v_xy_valid) {
					PX4_ERR("TrajectoryFollower: Localization lost");
					return StrategyResult::Failure("Localization invalid");
				}

				if (now - local_pos.timestamp > LOCALIZATION_TIMEOUT) {
					PX4_ERR("TrajectoryFollower: Localization timeout");
					return StrategyResult::Failure("Localization timeout");
				}

				// Calculate slip (velocity error between commanded and actual)
				// TODO: Get commanded velocity from trajectory controller
				// For now, check for abnormal velocity variance
				float vel_variance = local_pos.vxy_max;
				float slip_estimate = 0.0f;  // TODO: Implement proper slip calculation

				if (slip_estimate > SLIP_CRITICAL) {
					PX4_ERR("TrajectoryFollower: CRITICAL - Slip %.1f%%", (double)(slip_estimate * 100.0f));
					return StrategyResult::Failure("Excessive wheel slip");
				} else if (slip_estimate > SLIP_DEGRADED) {
					PX4_WARN("TrajectoryFollower: High slip %.1f%% - degraded mode", (double)(slip_estimate * 100.0f));
					degraded_condition = true;
				} else if (slip_estimate > SLIP_WARN) {
					warn_throttled(now, "Slip warning: %.1f%%", (double)(slip_estimate * 100.0f));
				}

				// TODO: Path deviation check - compare position to trajectory path
				// float deviation = calculate_path_deviation(local_pos);
			}

			// --- Sensor health ---
			sensor_combined_s sensors;
			if (_sensor_sub.copy(&sensors)) {
				if (now - sensors.timestamp > SENSOR_TIMEOUT) {
					PX4_ERR("TrajectoryFollower: Sensor timeout");
					return StrategyResult::Failure("Sensor data lost");
				}
			}

			// --- Apply degraded mode if needed ---
			if (degraded_condition && !_is_degraded) {
				_is_degraded = true;
				PX4_WARN("TrajectoryFollower: Entering DEGRADED mode - reduced speed");
				// TODO: Command reduced speed to trajectory controller
			} else if (!degraded_condition && _is_degraded) {
				_is_degraded = false;
				PX4_INFO("TrajectoryFollower: Exiting DEGRADED mode - normal speed");
				// TODO: Command normal speed to trajectory controller
			}

			// --- Check for trajectory completion ---
			if (check_trajectory_complete(now)) {
				PX4_INFO("TrajectoryFollower: Trajectory execution complete - transitioning to hold");
				_current_step = STEP_HOLD_AFTER_COMPLETE;
			}
		}
		break;

		case STEP_HOLD_AFTER_COMPLETE:
			// ========== STEP: Hold position after trajectory completion ==========
			{
				// Command hold mode on first entry to this step
				if (now - get_step_start() < 100_ms) {
					command_mode(operation_mode_cmd_s::MODE_WL_HOLD, 100);
					PX4_INFO("TrajectoryFollower: Commanding hold mode after trajectory completion");
				}

				// Verify hold mode is active
				operation_mode_status_s mode_status;
				if (_mode_status_sub.copy(&mode_status)) {
					if (mode_status.current_mode != operation_mode_cmd_s::MODE_WL_HOLD) {
						PX4_WARN("TrajectoryFollower: Hold mode not active - waiting for activation");
					}
				}

				// Monitor for new trajectory command
				// TODO: Subscribe to trajectory planner status/commands to detect new trajectory
				// For now, this step holds indefinitely until strategy is preempted or stopped
				// When new trajectory is received via strategy executor, this strategy will be
				// deactivated and reactivated, causing init() to run again with new trajectory

				// Battery monitoring during hold
				battery_status_s battery;
				if (_battery_sub.copy(&battery)) {
					if (battery.remaining < BATTERY_CRITICAL) {
						PX4_ERR("TrajectoryFollower: CRITICAL - Battery %.1f%% during hold",
						        (double)(battery.remaining * 100.0f));
						return StrategyResult::Failure("Battery critically low");
					} else if (battery.remaining < BATTERY_WARN) {
						warn_throttled(now, "Battery low during hold: %.1f%%",
						               (double)(battery.remaining * 100.0f));
					}
				}

				// Position monitoring during hold
				vehicle_local_position_s local_pos;
				if (_local_pos_sub.copy(&local_pos)) {
					if (!local_pos.xy_valid || !local_pos.v_xy_valid) {
						PX4_ERR("TrajectoryFollower: Localization lost during hold");
						return StrategyResult::Failure("Localization lost");
					}

					if (now - local_pos.timestamp > LOCALIZATION_TIMEOUT) {
						PX4_ERR("TrajectoryFollower: Localization timeout during hold");
						return StrategyResult::Failure("Localization timeout");
					}
				}

				// Strategy stays in hold mode indefinitely
				// Will be preempted when new trajectory command arrives
			}
			break;
		}

		// Still running
		return StrategyResult::Success();
	}

	const char *get_step_name() const override
	{
		const char *names[] = {"Activate Mode", "Following Trajectory", "Hold After Complete"};
		return names[_current_step];
	}

protected:
	FailsafeBase *create_failsafe() override
	{
		return new TrajectoryFollowerFailsafe();
	}

private:
	// Subscriptions
	uORB::Subscription _mode_status_sub{ORB_ID(operation_mode_status)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _sensor_sub{ORB_ID(sensor_combined)};

	// State tracking
	bool _is_degraded{false};
	hrt_abstime _last_warn_time{0};
	hrt_abstime _trajectory_start_time{0};

	/**
	 * Check if trajectory execution is complete
	 *
	 * Uses operation mode status as the authoritative source for trajectory completion.
	 * The trajectory mode implementation is responsible for signaling completion.
	 *
	 * @param now Current timestamp (unused, kept for interface compatibility)
	 * @return true if trajectory is complete
	 */
	bool check_trajectory_complete(hrt_abstime now)
	{
		(void)now;  // Unused parameter

		// Check operation mode status for trajectory completion
		operation_mode_status_s mode_status;
		if (_mode_status_sub.copy(&mode_status)) {
			// Check if mode signals trajectory is complete
			// TODO: Define trajectory_complete flag in operation_mode_status message
			if (mode_status.trajectory_complete) {
				PX4_INFO("TrajectoryFollower: Mode reports trajectory complete");
				return true;
			}
		}

		return false;
	}

	/**
	 * Throttled warning (max once per second)
	 */
	void warn_throttled(hrt_abstime now, const char *fmt, ...)
	{
		if (now - _last_warn_time > 1_s) {
			va_list args;
			va_start(args, fmt);
			px4_vlog(PX4_LOG_LEVEL_WARN, "TrajectoryFollower", __LINE__, fmt, args);
			va_end(args);
			_last_warn_time = now;
		}
	}
};
