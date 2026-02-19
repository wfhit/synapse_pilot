/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the distribution in the documentation
 *    and/or other materials provided with the distribution.
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

#include "tilt_kinematics_tilt.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>
#include <cmath>

TiltKinematicsTilt::TiltKinematicsTilt(ModuleParams *parent) :
	ModuleParams(parent)
{
	update_configuration();
}

void TiltKinematicsTilt::update_configuration()
{
	// Stage 2 - Bucket Tilt Linkage (Boom-End Frame)
	_config.bellcrank_joint_to_bucket_length = matrix::Vector2f(_param_bellcrank_joint_to_bucket_length_x.get(),
			_param_bellcrank_joint_to_bucket_length_y.get());

	// Link dimensions
	_config.bellcrank_length = _param_bellcrank_length.get();
	_config.coupler_length = _param_coupler_length.get();
	_config.bucket_arm_length = _param_bucket_arm_length.get();

	// Mechanical coupling and offsets
	_config.bucket_offset = _param_bucket_offset.get();

	// Physical and safety limits
	_config.bucket_angle_min = _param_bucket_angle_min.get();
	_config.bucket_angle_max = _param_bucket_angle_max.get();

	// Note: Triangle angles (bellcrank_internal_angle, bellcrank_boom_alignment_offset,
	// coupler_to_pivot_angle) are set via set_triangle_angles() method called from parent TiltKinematics
}

void TiltKinematicsTilt::set_triangle_angles(float bellcrank_internal_angle,
		float bellcrank_boom_alignment_offset,
		float coupler_to_pivot_angle)
{
	_config.bellcrank_internal_angle = bellcrank_internal_angle;
	_config.bellcrank_boom_alignment_offset = bellcrank_boom_alignment_offset;
	_config.coupler_to_pivot_angle = coupler_to_pivot_angle;

	// Log the angles for debugging
	PX4_INFO("Tilt: Set bellcrank_internal_angle = %.3f rad (%.1f deg)",
		 (double)_config.bellcrank_internal_angle,
		 (double)(_config.bellcrank_internal_angle * 180.0f / PI));

	PX4_INFO("Tilt: Set boom_alignment_offset = %.3f rad (%.1f deg)",
		 (double)_config.bellcrank_boom_alignment_offset,
		 (double)(_config.bellcrank_boom_alignment_offset * 180.0f / PI));

	PX4_INFO("Tilt: Set coupler_to_pivot_angle = %.3f rad (%.1f deg)",
		 (double)_config.coupler_to_pivot_angle,
		 (double)(_config.coupler_to_pivot_angle * 180.0f / PI));
}

// =========================
// MAIN KINEMATIC FUNCTIONS
// =========================

TiltKinematicsTilt::TiltState TiltKinematicsTilt::compute_forward_kinematics(
	float bellcrank_angle_drive, float boom_angle) const
{
	TiltState state{};
	state.is_valid = false;

	// Apply mechanical coupling: ∠OCB₂ = ∠OCB₁ + α
	state.bellcrank_angle = apply_stage_coupling(bellcrank_angle_drive);

	// Solve Stage 2: Bucket Tilt Linkage (Boom-End Frame)
	if (!solve_trigonometric(state.bellcrank_angle, boom_angle, state)) {
		PX4_WARN("Tilt Stage 2 solution failed for bellcrank angle");
		return state;
	}

	// Final validation
	state.is_valid = check_mechanical_limits(state);

	return state;
}

float TiltKinematicsTilt::compute_inverse_kinematics(
	float bucket_angle, float boom_angle) const
{
	// Stage 2 Inverse: Find required bellcrank angle for desired tilt angle
	float required_bellcrank_tilt = solve_inverse_trigonometric(bucket_angle, boom_angle);

	if (!std::isfinite(required_bellcrank_tilt)) {
		PX4_WARN("Tilt Stage 2 inverse failed for tilt angle");
		return NAN;
	}

	return required_bellcrank_tilt;
}

float TiltKinematicsTilt::apply_stage_coupling(float bellcrank_angle_drive) const
{
	// Coupling relationship: ∠OCB₂ = ∠OCB₁ + α
	// Where α is the internal angle between bellcrank planes
	return bellcrank_angle_drive + _config.bellcrank_internal_angle;
}

float TiltKinematicsTilt::remove_stage_coupling(float bellcrank_angle_tilt) const
{
	// Remove mechanical coupling to get Stage 1 angle
	// ∠OCB₁ = ∠OCB₂ - α
	return bellcrank_angle_tilt - _config.bellcrank_internal_angle;
}

// =========================
// TRIGONOMETRIC SOLUTION METHODS
// =========================

bool TiltKinematicsTilt::solve_trigonometric(float bellcrank_angle_tilt, float boom_angle, TiltState &state) const
{
	/**
	 * Stage 2: Bucket Tilt Linkage (OABC) - Boom-End Frame
	 *
	 * Points:
	 *   O: Boom end reference point (origin)
	 *   A: Bucket attachment (fixed to bucket)
	 *   B: Coupler joint (moving)
	 *   C: Bucket point (bellcrank pivot)
	 *
	 * Solution approach:
	 *   1. Calculate OB distance using law of cosines in triangle OBC
	 *   2. Find angles AOB and BOC using known link lengths
	 *   3. Compute tilt angle: AOC = AOB - BOC
	 *   4. Apply boom angle and offset compensation
	 */

	// Extract linkage dimensions with descriptive names
	const float bucket_arm_length = _config.bucket_arm_length;           // Distance O to A
	const float coupler_link_length = _config.coupler_length;            // Distance A to B
	const float bellcrank_length = _config.bellcrank_length;             // Distance B to C
	const float bellcrank_joint_to_bucket_length =
		_config.bellcrank_joint_to_bucket_length.norm();    			 // Distance O to C
	const float bellcrank_joint_angle = bellcrank_angle_tilt;            // Angle OCB

	// Step 1: Calculate coupler joint distance using law of cosines
	const float coupler_joint_to_pivot_distance =
		law_of_cosines_side(
			bellcrank_joint_to_bucket_length, bellcrank_length, bellcrank_joint_angle);

	if (!std::isfinite(coupler_joint_to_pivot_distance) ||
	    coupler_joint_to_pivot_distance <= GEOMETRIC_TOLERANCE) {
		return false;  // Invalid triangle geometry
	}

	// Step 2: Calculate internal triangle angles using law of cosines
	const float bucket_to_coupler_angle =
		law_of_cosines_angle(
			bucket_arm_length, coupler_joint_to_pivot_distance, coupler_link_length);
	const float coupler_to_pivot_angle =
		law_of_cosines_angle(
			coupler_joint_to_pivot_distance, bellcrank_joint_to_bucket_length, bellcrank_length);

	if (!std::isfinite(bucket_to_coupler_angle) || !std::isfinite(coupler_to_pivot_angle)) {
		return false;  // Invalid triangle configuration
	}

	// Step 3: Calculate tilt angle relative to boom frame
	const float bucket_angle_boom_relative = bucket_to_coupler_angle - coupler_to_pivot_angle;

	// Step 4: Apply boom angle, boom alignment offset, and calibration offset to get chassis-relative angle
	state.bucket_angle = bucket_angle_boom_relative + boom_angle + _config.bucket_offset +
			     _config.bellcrank_boom_alignment_offset;
	state.bellcrank_angle = bellcrank_angle_tilt;

	// Calculate supplementary state information for debugging and visualization
	// Apply bellcrank internal angle compensation to coupler angle calculation
	state.coupler_angle = atan2f(sinf(bucket_to_coupler_angle),
				     cosf(bucket_to_coupler_angle)) + _config.bellcrank_internal_angle;
	state.joint_B = matrix::Vector2f{
		coupler_joint_to_pivot_distance * cosf(coupler_to_pivot_angle),
		coupler_joint_to_pivot_distance * sinf(coupler_to_pivot_angle)
	};

	return true;
}

float TiltKinematicsTilt::solve_inverse_trigonometric(float bucket_angle, float boom_angle) const
{
	/**
	 * Inverse Stage 2: Given tilt angle (chassis-relative), find required bellcrank angle
	 *
	 * Approach:
	 *   1. Remove boom angle and offset to get boom-relative tilt angle
	 *   2. Calculate point A position from desired tilt angle
	 *   3. Find intersection of circles centered at A and C (joint B location)
	 *   4. Calculate bellcrank angle from point B position
	 *   5. Validate solution using forward kinematics
	 */

	// Step 1: Convert chassis-relative angle to boom-relative angle
	const float target_bucket_angle_boom_relative = bucket_angle - boom_angle - _config.bucket_offset -
			_config.bellcrank_boom_alignment_offset;

	// Extract linkage dimensions with descriptive names
	const float bucket_arm_length = _config.bucket_arm_length;           // Distance O to A
	const float coupler_link_length = _config.coupler_length;            // Distance A to B
	const float bellcrank_length = _config.bellcrank_length;         	 // Distance B to C
	const float bellcrank_joint_to_bucket_length =
		_config.bellcrank_joint_to_bucket_length.norm();    			 // Distance O to C

	// Step 2: Calculate bucket attachment point from desired angle
	const matrix::Vector2f bucket_attachment_point = {
		bucket_arm_length * cosf(target_bucket_angle_boom_relative),
		bucket_arm_length * sinf(target_bucket_angle_boom_relative)
	};

	// Bellcrank pivot point is fixed at known location
	const matrix::Vector2f bellcrank_pivot_point = {bellcrank_joint_to_bucket_length, 0.0f};

	// Step 3: Calculate distance between attachment and pivot points
	const matrix::Vector2f attachment_to_pivot_vector = bellcrank_pivot_point - bucket_attachment_point;
	const float attachment_to_pivot_distance = attachment_to_pivot_vector.norm();

	// Verify geometric feasibility using triangle inequality
	if (!is_triangle_valid(coupler_link_length, bellcrank_length, attachment_to_pivot_distance)) {
		return NAN;  // No valid solution exists
	}

	// Step 4: Calculate angle at bellcrank pivot using law of cosines
	const float pivot_internal_angle =
		law_of_cosines_angle(attachment_to_pivot_distance, bellcrank_length, coupler_link_length);

	if (!std::isfinite(pivot_internal_angle)) {
		return NAN;  // Invalid triangle configuration
	}

	const float attachment_direction_angle = atan2f(attachment_to_pivot_vector(1), attachment_to_pivot_vector(0));

	// Step 5: Calculate bellcrank angle candidates and select the best one
	const float bellcrank_angle_candidate_1 = attachment_direction_angle + pivot_internal_angle;
	const float bellcrank_angle_candidate_2 = attachment_direction_angle - pivot_internal_angle;

	return validate_and_select_best_candidate(bellcrank_angle_candidate_1, bellcrank_angle_candidate_2,
			bucket_angle, boom_angle);
}

// =========================
// GEOMETRIC HELPER FUNCTIONS
// =========================

bool TiltKinematicsTilt::is_triangle_valid(float a, float b, float c) const
{
	// Check triangle inequality: sum of any two sides > third side
	return (a + b > c) && (a + c > b) && (b + c > a) &&
	       (a > GEOMETRIC_TOLERANCE) && (b > GEOMETRIC_TOLERANCE) && (c > GEOMETRIC_TOLERANCE);
}

// =========================
// VALIDATION AND ANALYSIS
// =========================

bool TiltKinematicsTilt::check_mechanical_limits(const TiltState &state) const
{
	// Check tilt angle limits
	if (state.bucket_angle < _config.bucket_angle_min ||
	    state.bucket_angle > _config.bucket_angle_max) {
		return false;
	}

	return true;
}

bool TiltKinematicsTilt::validate_configuration() const
{
	// Check for positive link lengths
	if (_config.bellcrank_length <= 0.0f ||
	    _config.coupler_length <= 0.0f ||
	    _config.bucket_arm_length <= 0.0f) {
		PX4_ERR("Invalid tilt linkage dimensions: all lengths must be positive");
		return false;
	}

	// Check tilt angle range
	if (_config.bucket_angle_min >= _config.bucket_angle_max) {
		PX4_ERR("Invalid tilt angle range: min >= max");
		return false;
	}

	// Test trigonometric solutions at key points
	const float test_angles_drive[] = {
		0.0f,
		PI * 0.25f,
		PI * 0.5f,
		PI * 0.75f
	};

	for (float test_angle_drive : test_angles_drive) {
		TiltState state = compute_forward_kinematics(test_angle_drive, 0.0f);

		if (!state.is_valid) {
			continue;  // Some angles may be out of range, that's OK
		}

		// Test inverse kinematics round-trip
		float inverse_angle_tilt = compute_inverse_kinematics(state.bucket_angle, 0.0f);

		if (!std::isfinite(inverse_angle_tilt)) {
			PX4_ERR("Tilt inverse kinematics failed at test tilt angle");
			return false;
		}

		// Check round-trip accuracy
		float angle_error = fabsf(inverse_angle_tilt - state.bellcrank_angle);

		if (angle_error > 0.01f) {  // 0.01 rad tolerance
			PX4_ERR("Tilt round-trip error too large - tolerance exceeded");
			return false;
		}
	}

	PX4_INFO("Bucket tilt kinematics configuration validated successfully");
	PX4_INFO("Bucket angle range validated");

	return true;
}

float TiltKinematicsTilt::law_of_cosines_angle(float side_a, float side_b, float side_c) const
{
	// Calculate angle C using law of cosines: cos(C) = (a²+b²-c²)/(2ab)
	if (side_a <= 0.0f || side_b <= 0.0f || side_c <= 0.0f) {
		return NAN;
	}

	float cos_C = (side_a * side_a + side_b * side_b - side_c * side_c) / (2.0f * side_a * side_b);

	if (fabsf(cos_C) > 1.0f) {
		return NAN;  // Invalid triangle
	}

	return acosf(cos_C);
}

float TiltKinematicsTilt::law_of_cosines_side(float side_a, float side_b, float angle_between) const
{
	// Calculate third side using law of cosines: c^2 = a^2 + b^2 - 2ab*cos(C)
	if (side_a <= 0.0f || side_b <= 0.0f || !std::isfinite(angle_between)) {
		return NAN;
	}

	const float side_c_squared = side_a * side_a + side_b * side_b -
				     2.0f * side_a * side_b * cosf(angle_between);

	if (side_c_squared <= 0.0f) {
		return NAN;  // Invalid triangle geometry
	}

	return sqrtf(side_c_squared);
}

float TiltKinematicsTilt::validate_and_select_best_candidate(float candidate_1, float candidate_2,
		float target_bucket_angle, float boom_angle) const
{
	/**
	 * Validate both bellcrank angle candidates using forward kinematics and select the best one
	 *
	 * This function tests both possible solutions from the inverse kinematics calculation
	 * and selects the one that produces the closest match to the target tilt angle.
	 */

	// Validate both candidates using forward kinematics
	TiltState validation_state_1{}, validation_state_2{};
	const bool candidate_1_valid = solve_trigonometric(candidate_1, boom_angle, validation_state_1);
	const bool candidate_2_valid = solve_trigonometric(candidate_2, boom_angle, validation_state_2);

	if (candidate_1_valid && candidate_2_valid) {
		// Both candidates are valid - select the one with minimum error
		const float angle_error_1 = fabsf(validation_state_1.bucket_angle - target_bucket_angle);
		const float angle_error_2 = fabsf(validation_state_2.bucket_angle - target_bucket_angle);
		return (angle_error_1 < angle_error_2) ? candidate_1 : candidate_2;

	} else if (candidate_1_valid) {
		// Only first candidate is valid
		return candidate_1;

	} else if (candidate_2_valid) {
		// Only second candidate is valid
		return candidate_2;

	} else {
		// No valid solution found
		return NAN;
	}
}
