/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the distribution in the
 *    documentation and/or other materials provided with the
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

#include "tilt_kinematics_drive.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>
#include <cmath>

TiltKinematicsDrive::TiltKinematicsDrive(ModuleParams *parent) :
	ModuleParams(parent)
{
	update_configuration();
}

void TiltKinematicsDrive::update_configuration()
{
	// Stage 1 - Bucket Actuation Linkage (Machine Body Frame)
	_config.motor_base = matrix::Vector2f(_param_motor_base_x.get(), _param_motor_base_y.get());
	_config.crank_joint_to_pivot_length = _param_crank_pivot_len.get();
	_config.crank_joint_to_pivot_angle = _param_crank_pivot_ang.get();

	// Link dimensions
	_config.drive_bellcrank_length = _param_bellcrank_len.get();

	// Physical and safety limits from quad encoder
	_config.actuator_min_length = _param_encoder_pos_min.get();
	_config.actuator_max_length = _param_encoder_pos_max.get();

	// Note: Triangle angles (bellcrank_boom_alignment_offset, coupler_to_pivot_angle)
	// are set via set_triangle_angles() method called from parent TiltKinematics
}

void TiltKinematicsDrive::set_triangle_angles(float bellcrank_boom_alignment_offset,
		float coupler_to_pivot_angle)
{
	_config.bellcrank_boom_alignment_offset = bellcrank_boom_alignment_offset;
	_config.coupler_to_pivot_angle = coupler_to_pivot_angle;

	// Log the angles for debugging
	PX4_INFO("Drive: Set boom_alignment_offset = %.3f rad (%.1f deg)",
		 (double)_config.bellcrank_boom_alignment_offset,
		 (double)(_config.bellcrank_boom_alignment_offset * 180.0f / PI));

	PX4_INFO("Drive: Set coupler_to_pivot_angle = %.3f rad (%.1f deg)",
		 (double)_config.coupler_to_pivot_angle,
		 (double)(_config.coupler_to_pivot_angle * 180.0f / PI));
}

// =========================
// MAIN KINEMATIC FUNCTIONS
// =========================

TiltKinematicsDrive::DriveState TiltKinematicsDrive::compute_forward_kinematics(
	float actuator_length, float boom_angle) const
{
	DriveState state{};
	state.actuator_length = actuator_length;
	state.is_valid = false;

	// Solve Stage 1: Bucket Actuation Linkage (Machine Body Frame)
	if (!solve_trigonometric(actuator_length, boom_angle, state)) {
		PX4_WARN("Drive Stage 1 solution failed for AB length: %.2f mm", (double)actuator_length);
		return state;
	}

	// Compute condition number for singularity detection
	state.condition_number = compute_condition_number(state);

	// Final validation
	state.is_valid = check_mechanical_limits(state);

	return state;
}

float TiltKinematicsDrive::compute_inverse_kinematics(
	float bellcrank_angle, float boom_angle) const
{
	// Inverse Stage 1: Find required AB length for bellcrank angle
	float required_actuator_length = solve_inverse_trigonometric(bellcrank_angle, boom_angle);

	if (required_actuator_length < 0.0f) {
		PX4_DEBUG("Drive Stage 1 inverse failed for bellcrank angle: %.3f rad", (double)bellcrank_angle);
		return -1.0f;
	}

	// Validate against AB length limits from quad encoder
	if (required_actuator_length < _config.actuator_min_length ||
	    required_actuator_length > _config.actuator_max_length) {
		PX4_DEBUG("Drive required AB length %.2f mm outside limits [%.2f, %.2f]",
			  (double)required_actuator_length,
			  (double)_config.actuator_min_length,
			  (double)_config.actuator_max_length);
		return -1.0f;
	}

	return required_actuator_length;
}

// =========================
// TRIGONOMETRIC SOLUTION METHODS
// =========================

bool TiltKinematicsDrive::solve_trigonometric(float actuator_length, float boom_angle, DriveState &state) const
{
	// Direct trigonometric approach using law of cosines
	// Given: AB (actuator_length), BC (drive_bellcrank_length), boom_angle
	// Find: bellcrank_angle

	// Step 1: Get motor base position (point A) from parameters
	matrix::Vector2f point_A = _config.motor_base;

	// Step 2: Calculate bellcrank joint position (point C) from boom angle
	matrix::Vector2f point_C = calculate_bellcrank_joint_position(boom_angle);

	// Step 3: Calculate length AC
	matrix::Vector2f AC_vector = point_C - point_A;
	float length_AC = AC_vector.norm();

	// Step 4: Check triangle ABC validity
	float length_AB = actuator_length;
	float length_BC = _config.drive_bellcrank_length;

	if (!is_triangle_valid(length_AB, length_BC, length_AC)) {
		return false;  // Triangle ABC is impossible
	}

	// Step 5: Calculate angle OCA using direct trigonometry
	float angle_OCA = calculate_angle_OCA(point_C, point_A);

	// Step 6: Calculate angle ACB using law of cosines
	float angle_ACB = law_of_cosines_angle(length_AC, length_BC, length_AB);

	if (!std::isfinite(angle_ACB)) {
		return false;  // Invalid triangle
	}

	// Step 7: Calculate angle BCO = angle_ACB - angle_OCA
	float angle_BCO = angle_ACB - angle_OCA;

	// Step 8: Apply bellcrank-boom alignment offset and store results in state
	state.bellcrank_angle = angle_BCO + _config.bellcrank_boom_alignment_offset;

	return true;
}

float TiltKinematicsDrive::solve_inverse_trigonometric(float bellcrank_angle, float boom_angle) const
{
	// Inverse Stage 1: Given bellcrank angle (BCO), find required AB length

	// Step 1: Remove bellcrank-boom alignment offset from input angle
	float corrected_bellcrank_angle = bellcrank_angle - _config.bellcrank_boom_alignment_offset;

	// Step 2: Get motor base position (point A) from parameters
	matrix::Vector2f point_A = _config.motor_base;

	// Step 3: Calculate bellcrank joint position (point C) from boom angle
	matrix::Vector2f point_C = calculate_bellcrank_joint_position(boom_angle);

	// Step 4: Calculate angle OCA
	float angle_OCA = calculate_angle_OCA(point_C, point_A);

	// Step 5: Calculate angle BCA = BCO + OCA
	float angle_BCA = corrected_bellcrank_angle + angle_OCA;

	// Step 6: Find point B using angle BCA and BC length
	// Point B is at distance BC from C at angle BCA relative to CA direction
	matrix::Vector2f CA_vector = point_A - point_C;
	float CA_angle = atan2f(CA_vector(1), CA_vector(0));
	float B_angle = CA_angle + angle_BCA;

	matrix::Vector2f point_B = point_C + matrix::Vector2f{cosf(B_angle), sinf(B_angle)} * _config.drive_bellcrank_length;

	// Step 7: Calculate required AB length
	matrix::Vector2f AB_vector = point_B - point_A;
	float required_length = AB_vector.norm();

	// Step 7: Validate against AB length limits from quad encoder
	if (required_length < _config.actuator_min_length ||
	    required_length > _config.actuator_max_length) {
		return -1.0f;  // Invalid solution
	}

	return required_length;
}

// =========================
// =========================
// GEOMETRIC HELPER FUNCTIONS
// =========================

float TiltKinematicsDrive::law_of_cosines_angle(float side_a, float side_b, float side_c) const
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

// =========================
// GEOMETRIC HELPER FUNCTIONS
// =========================

matrix::Vector2f TiltKinematicsDrive::calculate_bellcrank_joint_position(float boom_angle) const
{
	// Calculate bellcrank joint (point C) position from boom angle
	// Point C is at distance crank_joint_to_pivot_length from origin O at angle (boom_angle + crank_joint_to_pivot_angle)
	float effective_boom_angle = boom_angle + _config.crank_joint_to_pivot_angle;
	return matrix::Vector2f{cosf(effective_boom_angle), sinf(effective_boom_angle)} * _config.crank_joint_to_pivot_length;
}

float TiltKinematicsDrive::calculate_angle_OCA(const matrix::Vector2f &point_C, const matrix::Vector2f &point_A) const
{
	// Calculate angle OCA (angle at point C between OC and CA)
	matrix::Vector2f CO_vector = -point_C;  // Vector from C to O (origin)
	matrix::Vector2f CA_vector = point_A - point_C;  // Vector from C to A

	// Calculate angle between vectors using dot product
	CO_vector = CO_vector.normalized();
	CA_vector = CA_vector.normalized();

	float dot_product = CO_vector.dot(CA_vector);
	dot_product = math::constrain(dot_product, -1.0f, 1.0f);

	return acosf(dot_product);
}

bool TiltKinematicsDrive::is_triangle_valid(float a, float b, float c) const
{
	// Check triangle inequality: sum of any two sides > third side
	return (a + b > c) && (a + c > b) && (b + c > a) &&
	       (a > GEOMETRIC_TOLERANCE) && (b > GEOMETRIC_TOLERANCE) && (c > GEOMETRIC_TOLERANCE);
}

// =========================
// VALIDATION AND ANALYSIS
// =========================

bool TiltKinematicsDrive::check_mechanical_limits(const DriveState &state) const
{
	// Check actuator length limits
	if (state.actuator_length < _config.actuator_min_length ||
	    state.actuator_length > _config.actuator_max_length) {
		return false;
	}

	// Check condition number (avoid singularities)
	if (state.condition_number > MAX_CONDITION_NUMBER) {
		return false;
	}

	return true;
}

float TiltKinematicsDrive::compute_condition_number(const DriveState &state) const
{
	// Compute linkage condition number based on Jacobian
	matrix::Matrix<float, 2, 2> jacobian = compute_jacobian(state);

	// Calculate determinant for 2x2 matrix: det = ad - bc
	float det = jacobian(0, 0) * jacobian(1, 1) - jacobian(0, 1) * jacobian(1, 0);

	if (fabsf(det) < GEOMETRIC_TOLERANCE) {
		return MAX_CONDITION_NUMBER;  // Singular
	}

	// Calculate Frobenius norm
	float norm = 0.0f;

	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			norm += jacobian(i, j) * jacobian(i, j);
		}
	}

	norm = sqrtf(norm);

	return norm / fabsf(det);
}

matrix::Matrix<float, 2, 2> TiltKinematicsDrive::compute_jacobian(const DriveState &state) const
{
	matrix::Matrix<float, 2, 2> jacobian;

	// Analytical Jacobian computation for trigonometric solution
	// J = [∂bellcrank_angle/∂actuator_length,  ∂bellcrank_angle/∂boom_angle]
	//     [∂actuator_length/∂bellcrank_angle,  ∂actuator_length/∂boom_angle]

	// For analytical differentiation, we need partial derivatives of the trigonometric solution
	// This is complex, so we'll use numerical differentiation with small perturbation

	const float h = 1e-4f;  // Small perturbation for numerical differentiation

	// Partial derivatives with respect to actuator length
	DriveState state_plus = compute_forward_kinematics(state.actuator_length + h, 0.0f);
	DriveState state_minus = compute_forward_kinematics(state.actuator_length - h, 0.0f);

	if (state_plus.is_valid && state_minus.is_valid) {
		jacobian(0, 0) = (state_plus.bellcrank_angle - state_minus.bellcrank_angle) / (2.0f * h);
		jacobian(1, 0) = (state_plus.actuator_length - state_minus.actuator_length) / (2.0f * h);

	} else {
		jacobian(0, 0) = 0.0f;
		jacobian(1, 0) = 0.0f;
	}

	// Partial derivatives with respect to boom angle
	state_plus = compute_forward_kinematics(state.actuator_length, h);
	state_minus = compute_forward_kinematics(state.actuator_length, -h);

	if (state_plus.is_valid && state_minus.is_valid) {
		jacobian(0, 1) = (state_plus.bellcrank_angle - state_minus.bellcrank_angle) / (2.0f * h);
		jacobian(1, 1) = (state_plus.actuator_length - state_minus.actuator_length) / (2.0f * h);

	} else {
		jacobian(0, 1) = 0.0f;
		jacobian(1, 1) = 0.0f;
	}

	return jacobian;
}

bool TiltKinematicsDrive::validate_configuration() const
{
	// Check for positive link lengths
	if (_config.drive_bellcrank_length <= 0.0f ||
	    _config.crank_joint_to_pivot_length <= 0.0f) {
		PX4_ERR("Invalid drive linkage dimensions: all lengths must be positive");
		return false;
	}

	// Check actuator range
	if (_config.actuator_min_length >= _config.actuator_max_length) {
		PX4_ERR("Invalid AB range: min >= max");
		return false;
	}

	// Test trigonometric solutions at key points
	const float test_lengths[] = {
		_config.actuator_min_length,
		(_config.actuator_min_length + _config.actuator_max_length) * 0.5f,
		_config.actuator_max_length
	};

	for (float test_length : test_lengths) {
		DriveState state = compute_forward_kinematics(test_length, 0.0f);

		if (!state.is_valid) {
			PX4_ERR("Drive forward kinematics failed");
			return false;
		}

		// Test inverse kinematics round-trip
		float inverse_length = compute_inverse_kinematics(state.bellcrank_angle, 0.0f);

		if (inverse_length < 0.0f) {
			PX4_ERR("Drive inverse kinematics failed");
			return false;
		}

		// Check round-trip accuracy
		float length_error = fabsf(inverse_length - test_length);

		if (length_error > 1.0f) {  // 1mm tolerance
			PX4_ERR("Drive round-trip error too large");
			return false;
		}
	}

	PX4_INFO("Bucket drive kinematics validated");

	return true;
}
