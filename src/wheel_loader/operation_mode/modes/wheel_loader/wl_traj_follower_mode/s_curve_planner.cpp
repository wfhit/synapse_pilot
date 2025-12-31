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

#include "s_curve_planner.hpp"
#include <mathlib/mathlib.h>

SCurvePlanner::SCurvePlanner()
{
	reset();
}

void SCurvePlanner::setLimits(const MotionLimits &limits)
{
	_limits = limits;
}

void SCurvePlanner::setProfileType(ProfileType type)
{
	_profile_type = type;
}

bool SCurvePlanner::planTrajectory(const TrajectoryPoint &current, float target_position, float target_time)
{
	_start_position = current.position;
	_start_velocity = current.velocity;
	_start_acceleration = current.acceleration;
	_end_position = target_position;

	bool success = false;

	if (_profile_type == ProfileType::QUINTIC) {
		success = planQuintic(current, target_position, target_time);

	} else if (_profile_type == ProfileType::SEVEN_SEGMENT) {
		success = planSevenSegment(current, target_position);
	}

	_is_valid = success;
	return success;
}

bool SCurvePlanner::getTrajectoryPoint(float t, TrajectoryPoint &point)
{
	if (!_is_valid) {
		return false;
	}

	// Clamp time to valid range
	t = math::constrain(t, 0.f, _duration);

	if (_profile_type == ProfileType::QUINTIC) {
		evaluateQuintic(t, point);

	} else if (_profile_type == ProfileType::SEVEN_SEGMENT) {
		evaluateSevenSegment(t, point);
	}

	return true;
}

void SCurvePlanner::reset()
{
	_start_position = 0.f;
	_start_velocity = 0.f;
	_start_acceleration = 0.f;
	_end_position = 0.f;
	_duration = 0.f;
	_is_valid = false;

	_a0 = _a1 = _a2 = _a3 = _a4 = _a5 = 0.f;
}

bool SCurvePlanner::planQuintic(const TrajectoryPoint &current, float target_position, float target_time)
{
	float p0 = current.position;
	float v0 = current.velocity;
	float a0 = current.acceleration;
	float pf = target_position;

	// Target velocity and acceleration are zero for smooth stop
	float vf = 0.f;
	float af = 0.f;

	// Auto-calculate duration if not specified
	if (target_time <= 0.f) {
		float distance = fabsf(pf - p0);
		// Estimate duration based on max velocity
		target_time = fmaxf(2.0f * distance / _limits.max_velocity, 0.1f);
	}

	_duration = target_time;
	float T = _duration;

	// Quintic polynomial boundary conditions:
	// p(0) = p0, p'(0) = v0, p''(0) = a0
	// p(T) = pf, p'(T) = vf, p''(T) = af

	// Solve for coefficients
	float T2 = T * T;
	float T3 = T2 * T;
	float T4 = T3 * T;
	float T5 = T4 * T;

	_a0 = p0;
	_a1 = v0;
	_a2 = 0.5f * a0;
	_a3 = (20.f * pf - 20.f * p0 - (8.f * vf + 12.f * v0) * T - (3.f * af - a0) * T2) / (2.f * T3);
	_a4 = (30.f * p0 - 30.f * pf + (14.f * vf + 16.f * v0) * T + (3.f * af - 2.f * a0) * T2) / (2.f * T4);
	_a5 = (12.f * pf - 12.f * p0 - (6.f * vf + 6.f * v0) * T - (af - a0) * T2) / (2.f * T5);

	return true;
}

bool SCurvePlanner::planSevenSegment(const TrajectoryPoint &current, float target_position)
{
	// Simplified 7-segment S-curve planning
	// Full implementation would compute exact jerk-limited trajectory
	// This is a placeholder - use standard time-optimal S-curve algorithm

	float distance = target_position - current.position;
	// float direction = (distance >= 0.f) ? 1.f : -1.f;
	distance = fabsf(distance);

	// Estimate duration based on trapezoidal velocity profile with jerk limits
	float v_max = _limits.max_velocity;
	float a_max = _limits.max_acceleration;
	float j_max = _limits.max_jerk;

	// Time to reach max acceleration with jerk limit
	float t_jerk = a_max / j_max;

	// Compute profile times (simplified)
	_t_jerk_acc = t_jerk;
	_t_constant_acc = fmaxf((v_max - a_max * t_jerk) / a_max, 0.f);
	_t_jerk_dec_acc = t_jerk;

	// Distance covered during acceleration phase
	float d_acc = 0.5f * a_max * t_jerk * t_jerk + a_max * _t_constant_acc * (t_jerk + 0.5f * _t_constant_acc) +
		      0.5f * a_max * t_jerk * t_jerk;

	// Distance covered during deceleration phase (symmetric)
	float d_dec = d_acc;

	// Remaining distance at constant velocity
	float d_const = distance - d_acc - d_dec;

	if (d_const > 0.f) {
		_t_constant_vel = d_const / v_max;

	} else {
		// Cannot reach max velocity, recalculate
		_t_constant_vel = 0.f;
		// Simplified: scale times proportionally
		float scale = sqrtf(distance / (d_acc + d_dec));
		_t_jerk_acc *= scale;
		_t_constant_acc *= scale;
		_t_jerk_dec_acc *= scale;
	}

	// Deceleration phase (symmetric to acceleration)
	_t_jerk_dec = _t_jerk_acc;
	_t_constant_dec = _t_constant_acc;
	_t_jerk_stop = _t_jerk_dec_acc;

	_duration = _t_jerk_acc + _t_constant_acc + _t_jerk_dec_acc +
		    _t_constant_vel +
		    _t_jerk_dec + _t_constant_dec + _t_jerk_stop;

	return true;
}

void SCurvePlanner::evaluateQuintic(float t, TrajectoryPoint &point)
{
	float t2 = t * t;
	float t3 = t2 * t;
	float t4 = t3 * t;
	float t5 = t4 * t;

	// Position: p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
	point.position = _a0 + _a1 * t + _a2 * t2 + _a3 * t3 + _a4 * t4 + _a5 * t5;

	// Velocity: p'(t) = a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
	point.velocity = _a1 + 2.f * _a2 * t + 3.f * _a3 * t2 + 4.f * _a4 * t3 + 5.f * _a5 * t4;

	// Acceleration: p''(t) = 2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
	point.acceleration = 2.f * _a2 + 6.f * _a3 * t + 12.f * _a4 * t2 + 20.f * _a5 * t3;
}

void SCurvePlanner::evaluateSevenSegment(float t, TrajectoryPoint &point)
{
	// Simplified 7-segment evaluation
	// Full implementation would properly compute position/velocity/acceleration
	// based on the 7 phases with jerk limits

	float j_max = _limits.max_jerk;
	(void)_limits.max_acceleration; // unused - for future implementation
	(void)_limits.max_velocity; // unused - for future implementation

	float time_acc = 0.f;
	float pos = _start_position;
	float vel = _start_velocity;
	float acc = _start_acceleration;

	// Phase 1: Jerk acceleration
	if (t <= _t_jerk_acc) {
		float jerk = j_max;
		acc = jerk * t;
		vel = 0.5f * jerk * t * t;
		pos = (1.f / 6.f) * jerk * t * t * t;

	} else {
		time_acc += _t_jerk_acc;
		pos += (1.f / 6.f) * j_max * _t_jerk_acc * _t_jerk_acc * _t_jerk_acc;
		vel += 0.5f * j_max * _t_jerk_acc * _t_jerk_acc;
		acc = j_max * _t_jerk_acc;

		// Continue through other phases...
		// (Simplified version - full implementation needed)
		float t_remaining = t - time_acc;
		vel += acc * t_remaining;
		pos += vel * t_remaining;
	}

	point.position = _start_position + pos;
	point.velocity = vel;
	point.acceleration = acc;
}
