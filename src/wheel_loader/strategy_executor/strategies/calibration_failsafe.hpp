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
 * @file calibration_failsafe.hpp
 *
 * Failsafe monitor for calibration strategy
 *
 * Monitors:
 * - Vehicle movement during calibration
 * - Operator safety during actuator movement
 * - System stability during data collection
 * - Parameter write integrity
 */

#pragma once

#include "../failsafe_base.hpp"
#include <uORB/topics/operation_mode_cmd.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/Subscription.hpp>

class CalibrationFailsafe : public FailsafeBase
{
public:
	CalibrationFailsafe() : FailsafeBase("Calibration") {}

	// Safety thresholds
	static constexpr float VELOCITY_MAX = 0.1f;            // 0.1 m/s max during calibration
	static constexpr float ANGULAR_RATE_MAX = 0.05f;       // 0.05 rad/s max rotation
	static constexpr float ACCELERATION_MAX = 0.5f;        // 0.5 m/s² max acceleration

protected:
	FailsafeResult check_custom() override
	{
		// ========== 1. MOVEMENT DETECTION ==========
		vehicle_local_position_s local_pos;
		if (_local_pos_sub.copy(&local_pos)) {
			if (local_pos.v_xy_valid) {
				float velocity = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);

				if (velocity > VELOCITY_MAX) {
					PX4_ERR("CalibrationFailsafe: Unexpected movement detected: %.3f m/s", (double)velocity);
					return FailsafeResult::EmergencyStop("Movement during calibration");
				}
			}

			if (local_pos.a_xy_valid) {
				float acceleration = sqrtf(local_pos.ax * local_pos.ax + local_pos.ay * local_pos.ay);

				if (acceleration > ACCELERATION_MAX) {
					PX4_WARN("CalibrationFailsafe: High acceleration detected: %.3f m/s²", (double)acceleration);
					// Warning only - don't abort yet
				}
			}
		}

		// ========== 2. ROTATION DETECTION ==========
		vehicle_attitude_s attitude;
		if (_attitude_sub.copy(&attitude)) {
			float angular_rate = sqrtf(attitude.rollspeed * attitude.rollspeed +
			                            attitude.pitchspeed * attitude.pitchspeed +
			                            attitude.yawspeed * attitude.yawspeed);

			if (angular_rate > ANGULAR_RATE_MAX) {
				PX4_ERR("CalibrationFailsafe: Unexpected rotation detected: %.3f rad/s", (double)angular_rate);
				return FailsafeResult::EmergencyStop("Rotation during calibration");
			}
		}

		// ========== 3. ATTITUDE STABILITY ==========
		// Ensure vehicle attitude hasn't changed significantly
		// This detects if vehicle is on unstable surface or being disturbed
		if (_attitude_sub.copy(&attitude)) {
			matrix::Eulerf euler(matrix::Quatf(attitude.q));
			float pitch_deg = fabsf(math::degrees(euler.theta()));
			float roll_deg = fabsf(math::degrees(euler.phi()));

			// Check for extreme angles (indicates unstable surface)
			if (pitch_deg > 15.0f || roll_deg > 15.0f) {
				PX4_ERR("CalibrationFailsafe: Unstable surface - pitch: %.1f°, roll: %.1f°",
				        (double)pitch_deg, (double)roll_deg);
				return FailsafeResult::Hold("Unstable vehicle attitude");
			}
		}

		// ========== 4. SENSOR DATA QUALITY ==========
		// Verify sensor data is stable and not showing anomalies
		// TODO: Check sensor data variance/noise levels
		// TODO: Detect sensor glitches or dropouts
		// TODO: Validate timestamp consistency

		// ========== 5. ACTUATOR LIMITS ==========
		// During actuator calibration, ensure we don't exceed safe limits
		// TODO: Monitor actuator position feedback
		// TODO: Check hydraulic pressure is within bounds
		// TODO: Verify no mechanical binding or obstruction

		// ========== 6. POWER STABILITY ==========
		// Ensure stable power during calibration to prevent corruption
		// TODO: Monitor battery voltage for drops/spikes
		// TODO: Check for power brownouts
		// TODO: Verify electrical system is stable

		return FailsafeResult::Safe();
	}

private:
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
};
