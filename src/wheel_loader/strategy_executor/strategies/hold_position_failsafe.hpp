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
 * @file hold_position_failsafe.hpp
 *
 * Failsafe monitor for hold position strategy
 *
 * Monitors:
 * - Position drift beyond acceptable limits
 * - Localization quality and continuity
 * - External disturbances (wind, collisions)
 * - Actuator responsiveness during hold
 */

#pragma once

#include "../failsafe_base.hpp"
#include <uORB/topics/wheel_loader/operation_mode_cmd.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/Subscription.hpp>

class HoldPositionFailsafe : public FailsafeBase
{
public:
	HoldPositionFailsafe() : FailsafeBase("HoldPosition") {}

	// Safety thresholds
	static constexpr float POSITION_DRIFT_CRITICAL = 3.0f;     // 3m critical drift
	static constexpr float VELOCITY_CRITICAL = 0.5f;           // 0.5 m/s max while holding
	static constexpr float ACCELERATION_CRITICAL = 1.0f;       // 1.0 m/s² critical accel
	static constexpr uint64_t LOCALIZATION_TIMEOUT = 500000;   // 500ms timeout

protected:
	FailsafeResult check_custom() override
	{
		hrt_abstime now = hrt_absolute_time();

		// ========== 1. LOCALIZATION HEALTH ==========
		vehicle_local_position_s local_pos;
		if (_local_pos_sub.copy(&local_pos)) {
			// Check position validity
			if (!local_pos.xy_valid || !local_pos.z_valid) {
				PX4_ERR("HoldPositionFailsafe: Localization invalid during hold");
				return FailsafeResult::EmergencyStop("Localization lost");
			}

			// Check for stale data
			if (now - local_pos.timestamp > LOCALIZATION_TIMEOUT) {
				PX4_ERR("HoldPositionFailsafe: Localization timeout during hold");
				return FailsafeResult::EmergencyStop("Localization timeout");
			}

			// Check for unexpected velocity (should be near zero)
			if (local_pos.v_xy_valid) {
				float velocity = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);

				if (velocity > VELOCITY_CRITICAL) {
					PX4_ERR("HoldPositionFailsafe: Unexpected velocity during hold: %.2f m/s",
					        (double)velocity);
					return FailsafeResult::Hold("Unexpected motion detected");
				}
			}

			// Check for unexpected acceleration (external disturbance)
			if (local_pos.a_xy_valid) {
				float acceleration = sqrtf(local_pos.ax * local_pos.ax + local_pos.ay * local_pos.ay);

				if (acceleration > ACCELERATION_CRITICAL) {
					PX4_WARN("HoldPositionFailsafe: High acceleration: %.2f m/s² - possible collision",
					         (double)acceleration);
					// Warning level - may indicate collision or external force
				}
			}
		} else {
			PX4_ERR("HoldPositionFailsafe: No localization data available");
			return FailsafeResult::EmergencyStop("Localization unavailable");
		}

		// ========== 2. ATTITUDE STABILITY ==========
		vehicle_attitude_s attitude;
		if (_attitude_sub.copy(&attitude)) {
			// Check for excessive rotation rates (should be near zero during hold)
			float angular_rate = sqrtf(attitude.rollspeed * attitude.rollspeed +
			                            attitude.pitchspeed * attitude.pitchspeed +
			                            attitude.yawspeed * attitude.yawspeed);

			if (angular_rate > 0.1f) {  // 0.1 rad/s threshold
				PX4_WARN("HoldPositionFailsafe: Unexpected rotation during hold: %.2f rad/s",
				         (double)angular_rate);
				// Warning - may indicate loss of heading control
			}

			// Check for extreme attitudes (tip-over detection)
			matrix::Eulerf euler(matrix::Quatf(attitude.q));
			float pitch_deg = fabsf(math::degrees(euler.theta()));
			float roll_deg = fabsf(math::degrees(euler.phi()));

			if (pitch_deg > 30.0f || roll_deg > 30.0f) {
				PX4_ERR("HoldPositionFailsafe: Extreme attitude - pitch: %.1f°, roll: %.1f°",
				        (double)pitch_deg, (double)roll_deg);
				return FailsafeResult::EmergencyStop("Vehicle tipping");
			}
		}

		// ========== 3. ACTUATOR HEALTH ==========
		// TODO: Monitor actuator responsiveness during hold
		// TODO: Check for actuator saturation (indicates external forces)
		// TODO: Verify steering/drive systems are responding correctly

		// ========== 4. POSITION CONTROL QUALITY ==========
		// TODO: Monitor position control error trends
		// TODO: Detect oscillations or hunting behavior
		// TODO: Check for steady-state error accumulation

		// ========== 5. ENVIRONMENTAL HAZARDS ==========
		// TODO: Check for obstacle proximity (should be clear)
		// TODO: Monitor terrain changes (slope, surface)
		// TODO: Detect wind or external forces affecting hold

		return FailsafeResult::Safe();
	}

private:
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
};
