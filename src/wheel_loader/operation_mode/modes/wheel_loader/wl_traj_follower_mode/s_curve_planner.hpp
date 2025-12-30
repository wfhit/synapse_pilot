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
 * @file s_curve_planner.hpp
 *
 * S-curve trajectory planner for smooth motion profiles
 * Supports quintic polynomial and 7-segment S-curve profiles
 *
 * @author Your Name
 */

#pragma once

#include <matrix/matrix/math.hpp>
#include <drivers/drv_hrt.h>

enum class ProfileType {
	QUINTIC,       // Quintic polynomial (5th order)
	SEVEN_SEGMENT  // 7-segment S-curve (jerk-limited)
};

struct MotionLimits {
	float max_velocity{1.0f};       // Maximum velocity [units/s]
	float max_acceleration{1.0f};   // Maximum acceleration [units/s^2]
	float max_jerk{10.0f};          // Maximum jerk [units/s^3]
};

struct TrajectoryPoint {
	float position{0.f};
	float velocity{0.f};
	float acceleration{0.f};
};

class SCurvePlanner
{
public:
	SCurvePlanner();
	~SCurvePlanner() = default;

	/**
	 * Set motion limits
	 */
	void setLimits(const MotionLimits &limits);

	/**
	 * Set profile type
	 */
	void setProfileType(ProfileType type);

	/**
	 * Plan trajectory from current state to target
	 * @param current Current position/velocity/acceleration
	 * @param target Target position
	 * @param target_time Time to reach target [s] (0 = auto-calculate)
	 * @return true if planning successful
	 */
	bool planTrajectory(const TrajectoryPoint &current, float target_position, float target_time = 0.f);

	/**
	 * Get trajectory point at specific time
	 * @param t Time since trajectory start [s]
	 * @param point Output trajectory point
	 * @return true if within trajectory duration
	 */
	bool getTrajectoryPoint(float t, TrajectoryPoint &point);

	/**
	 * Get total trajectory duration
	 */
	float getDuration() const { return _duration; }

	/**
	 * Check if trajectory is complete
	 */
	bool isComplete(float t) const { return t >= _duration; }

	/**
	 * Reset planner
	 */
	void reset();

private:
	/**
	 * Plan using quintic polynomial
	 */
	bool planQuintic(const TrajectoryPoint &current, float target_position, float target_time);

	/**
	 * Plan using 7-segment S-curve
	 */
	bool planSevenSegment(const TrajectoryPoint &current, float target_position);

	/**
	 * Evaluate quintic polynomial at time t
	 */
	void evaluateQuintic(float t, TrajectoryPoint &point);

	/**
	 * Evaluate 7-segment profile at time t
	 */
	void evaluateSevenSegment(float t, TrajectoryPoint &point);

	ProfileType _profile_type{ProfileType::QUINTIC};
	MotionLimits _limits;

	// Trajectory parameters
	float _start_position{0.f};
	float _start_velocity{0.f};
	float _start_acceleration{0.f};
	float _end_position{0.f};
	float _duration{0.f};

	// Quintic polynomial coefficients: p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
	float _a0, _a1, _a2, _a3, _a4, _a5;

	// 7-segment parameters
	float _t_jerk_acc{0.f};      // Time for jerk acceleration phase
	float _t_constant_acc{0.f};  // Time for constant acceleration phase
	float _t_jerk_dec_acc{0.f};  // Time for jerk deceleration from acceleration
	float _t_constant_vel{0.f};  // Time for constant velocity phase
	float _t_jerk_dec{0.f};      // Time for jerk deceleration phase
	float _t_constant_dec{0.f};  // Time for constant deceleration phase
	float _t_jerk_stop{0.f};     // Time for final jerk to stop

	bool _is_valid{false};
};
