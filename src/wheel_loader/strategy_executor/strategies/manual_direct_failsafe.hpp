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
 * @file manual_direct_failsafe.hpp
 *
 * Failsafe monitor for manual direct control strategy
 *
 * Monitors:
 * - Communication link quality and timeout
 * - Dead-man switch enforcement
 * - Collision avoidance sensors
 * - System health during high-risk manual operation
 */

#pragma once

#include "../failsafe_base.hpp"
#include <uORB/topics/operation_mode_cmd.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/Subscription.hpp>

class ManualDirectFailsafe : public FailsafeBase
{
public:
	ManualDirectFailsafe() : FailsafeBase("ManualDirect") {}

	// Safety thresholds
	static constexpr uint64_t CONTROL_CRITICAL_TIMEOUT = 500000;  // 500ms dead-man
	static constexpr float OBSTACLE_MIN_DISTANCE = 2.0f;          // 2m minimum clearance

protected:
	FailsafeResult check_custom() override
	{
		hrt_abstime now = hrt_absolute_time();

		// ========== 1. CRITICAL: DEAD-MAN SWITCH ENFORCEMENT ==========
		manual_control_setpoint_s manual_control;

		if (_manual_control_sub.copy(&manual_control)) {
			// Check for control input timeout (dead-man switch)
			if (now - manual_control.timestamp > CONTROL_CRITICAL_TIMEOUT) {
				PX4_ERR("ManualDirectFailsafe: DEAD-MAN SWITCH TIMEOUT - no input for %.1fms",
					(double)((now - manual_control.timestamp) / 1000.0f));
				return FailsafeResult::Emergency(FailsafeViolation::HEARTBEAT_TIMEOUT, "Dead-man switch timeout");
			}

			// Validate control values are within bounds
			if (!PX4_ISFINITE(manual_control.roll) ||
			    !PX4_ISFINITE(manual_control.pitch) ||
			    !PX4_ISFINITE(manual_control.throttle) ||
			    !PX4_ISFINITE(manual_control.yaw)) {
				PX4_ERR("ManualDirectFailsafe: Invalid control values detected");
				return FailsafeResult::Emergency(FailsafeViolation::STATE_INVALID, "Invalid control input");
			}

		} else {
			PX4_ERR("ManualDirectFailsafe: No control input available");
			return FailsafeResult::Emergency(FailsafeViolation::HEARTBEAT_TIMEOUT, "Control input lost");
		}

		// ========== 2. COLLISION AVOIDANCE ==========
		// TODO: Monitor distance sensors / lidar
		// TODO: Check for obstacles in path of travel
		// TODO: Implement minimum safe distance enforcement
		// Example:
		// if (obstacle_distance < OBSTACLE_MIN_DISTANCE) {
		//     PX4_ERR("ManualDirectFailsafe: Obstacle too close: %.1fm", obstacle_distance);
		//     return FailsafeResult::Hold("Collision imminent");
		// }

		// ========== 3. POSITION VALIDITY ==========
		vehicle_local_position_s local_pos;

		if (_local_pos_sub.copy(&local_pos)) {
			// While localization is not required for manual control,
			// if available it should be valid
			if (local_pos.v_xy_valid) {
				// Check for excessive velocity (sanity check)
				float speed = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);

				if (speed > 10.0f) {  // 10 m/s absolute max
					PX4_ERR("ManualDirectFailsafe: Excessive velocity detected: %.1f m/s", (double)speed);
					return FailsafeResult::Critical(FailsafeViolation::STATE_INVALID, FailsafeAction::SWITCH_TO_HOLD,
									"Velocity exceeded safety limit");
				}
			}
		}

		// ========== 4. COMMUNICATION QUALITY ==========
		// TODO: Monitor RC signal strength / quality
		// TODO: Check for signal dropouts or interference
		// TODO: Validate communication latency is acceptable

		// ========== 5. ACTUATOR HEALTH ==========
		// TODO: Monitor steering actuator response
		// TODO: Check drive motor health and current
		// TODO: Verify brake system is responsive

		// ========== 6. EMERGENCY STOP BUTTON ==========
		// Emergency stop button is already monitored by StrategyExecutor
		// via check_safety_button(), so no additional check needed here

		return FailsafeResult::Safe();
	}

private:
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
};
