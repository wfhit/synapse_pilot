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

#include "tilt_kinematics.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>
#include <cmath>

TiltKinematics::TiltKinematics(ModuleParams *parent) :
	ModuleParams(parent),
	_drive_kinematics(this),
	_tilt_kinematics(this)
{
	update_configuration();
}

void TiltKinematics::update_configuration()
{
	// Update both component configurations first
	_drive_kinematics.update_configuration();
	_tilt_kinematics.update_configuration();

	// Calculate and set triangle angles for both components
	float bellcrank_boom_alignment_offset = get_bellcrank_boom_alignment_offset();
	float coupler_to_pivot_angle = get_coupler_to_pivot_angle();
	float bellcrank_internal_angle = get_bellcrank_internal_angle();

	// Set calculated angles in drive component
	_drive_kinematics.set_triangle_angles(bellcrank_boom_alignment_offset, coupler_to_pivot_angle);

	// Set calculated angles in tilt component
	_tilt_kinematics.set_triangle_angles(bellcrank_internal_angle, bellcrank_boom_alignment_offset, coupler_to_pivot_angle);
}

// =========================
// TRIANGLE ANGLE CALCULATIONS
// =========================

float TiltKinematics::get_bellcrank_boom_alignment_offset() const
{
	// Calculate angle BAC in boom pivot triangle using law of cosines
	float boom_length = _param_boom_length.get();
	float boom_pivot_to_crank = _param_boom_pivot_to_crank_joint_distance.get();
	float bucket_to_crank = _param_bucket_to_crank_joint_distance.get();

	return law_of_cosines_angle(boom_length, boom_pivot_to_crank, bucket_to_crank);
}

float TiltKinematics::get_coupler_to_pivot_angle() const
{
	// Calculate angle BCA in boom pivot triangle using law of cosines
	float boom_length = _param_boom_length.get();
	float boom_pivot_to_crank = _param_boom_pivot_to_crank_joint_distance.get();
	float bucket_to_crank = _param_bucket_to_crank_joint_distance.get();

	return law_of_cosines_angle(bucket_to_crank, boom_pivot_to_crank, boom_length);
}

float TiltKinematics::get_bellcrank_internal_angle() const
{
	// Calculate 2π - angle ABC in bellcrank triangle using law of cosines
	float bellcrank_arm = _param_bellcrank_arm_length.get();
	float bellcrank_to_coupler = _param_bellcrank_to_coupler_distance.get();
	float bellcrank_to_actuator = _param_bellcrank_to_actuator_distance.get();

	float angle_ABC = law_of_cosines_angle(bellcrank_arm, bellcrank_to_actuator, bellcrank_to_coupler);

	if (!std::isnan(angle_ABC)) {
		return 2.0f * static_cast<float>(M_PI) - angle_ABC;
	} else {
		PX4_WARN("Invalid bellcrank triangle geometry for internal angle calculation");
		return 0.0f;
	}
}

// =========================
// HELPER METHODS
// =========================

float TiltKinematics::law_of_cosines_angle(float side_a, float side_b, float side_c) const
{
	// Calculate angle C using law of cosines: cos(C) = (a²+b²-c²)/(2ab)
	if (side_a <= 0.0f || side_b <= 0.0f || side_c <= 0.0f) {
		return NAN;
	}

	// Check triangle inequality
	if ((side_a + side_b <= side_c) || (side_a + side_c <= side_b) || (side_b + side_c <= side_a)) {
		return NAN;
	}

	float cos_C = (side_a * side_a + side_b * side_b - side_c * side_c) / (2.0f * side_a * side_b);

	// Clamp to valid range due to floating point precision issues
	cos_C = fmaxf(-1.0f, fminf(1.0f, cos_C));

	return acosf(cos_C);
}

// =========================
// MAIN KINEMATIC FUNCTIONS
// =========================

TiltKinematics::LinkageState TiltKinematics::compute_forward_kinematics(
	float actuator_length, float boom_angle) const
{
	LinkageState combined_state{};
	combined_state.actuator_length = actuator_length;
	combined_state.is_valid = false;

	// Stage 1: Compute drive kinematics
	TiltKinematicsDrive::DriveState drive_state =
		_drive_kinematics.compute_forward_kinematics(actuator_length, boom_angle);
	if (!drive_state.is_valid) {
		PX4_WARN("Drive kinematics failed for actuator length: %.2f mm", (double)actuator_length);
		return combined_state;
	}

	// Stage 2: Compute tilt kinematics using drive result
	TiltKinematicsTilt::TiltState tilt_state =
		_tilt_kinematics.compute_forward_kinematics(drive_state.bellcrank_angle, boom_angle);
	if (!tilt_state.is_valid) {
		PX4_WARN("Tilt kinematics failed for bellcrank angle: %.3f rad", (double)drive_state.bellcrank_angle);
		return combined_state;
	}

	// Combine results into unified state
	combined_state = combine_states(drive_state, tilt_state);

	return combined_state;
}

TiltKinematics::LinkageState TiltKinematics::compute_inverse_kinematics(
	float bucket_angle, float boom_angle) const
{
	LinkageState combined_state{};
	combined_state.bucket_angle = bucket_angle;
	combined_state.is_valid = false;

	// Stage 2 Inverse: Find required bellcrank angle for desired tilt angle
	float required_bellcrank_tilt = _tilt_kinematics.compute_inverse_kinematics(bucket_angle, boom_angle);
	if (!std::isfinite(required_bellcrank_tilt)) {
		PX4_WARN("Tilt inverse kinematics failed for tilt angle: %.3f rad", (double)bucket_angle);
		return combined_state;
	}

	// Convert Stage 2 bellcrank angle to Stage 1 bellcrank angle
	float required_bellcrank_drive = _tilt_kinematics.remove_stage_coupling(required_bellcrank_tilt);

	// Stage 1 Inverse: Find required actuator length for bellcrank angle
	float required_actuator_length = _drive_kinematics.compute_inverse_kinematics(required_bellcrank_drive, boom_angle);
	if (required_actuator_length < 0.0f) {
		PX4_WARN("Drive inverse kinematics failed for bellcrank angle: %.3f rad", (double)required_bellcrank_drive);
		return combined_state;
	}

	// Compute complete forward kinematics to get all state parameters
	combined_state = compute_forward_kinematics(required_actuator_length, boom_angle);

	return combined_state;
}

TiltKinematics::LinkageState TiltKinematics::combine_states(
	const TiltKinematicsDrive::DriveState& drive_state,
	const TiltKinematicsTilt::TiltState& tilt_state) const
{
	LinkageState combined_state{};

	// Primary outputs
	combined_state.actuator_length = drive_state.actuator_length;
	combined_state.bucket_angle = tilt_state.bucket_angle;
	combined_state.bellcrank_angle_drive = drive_state.bellcrank_angle;
	combined_state.bellcrank_angle_tilt = tilt_state.bellcrank_angle;

	// Validation - both stages must be valid
	combined_state.is_valid = drive_state.is_valid && tilt_state.is_valid;
	combined_state.condition_number = drive_state.condition_number;

	return combined_state;
}

bool TiltKinematics::validate_configuration() const
{
	bool drive_valid = _drive_kinematics.validate_configuration();
	bool tilt_valid = _tilt_kinematics.validate_configuration();

	if (!drive_valid) {
		PX4_ERR("Drive kinematics configuration validation failed");
	}

	if (!tilt_valid) {
		PX4_ERR("Tilt kinematics configuration validation failed");
	}

	if (drive_valid && tilt_valid) {
		PX4_INFO("Combined bucket kinematics configuration validated successfully");
	}

	return drive_valid && tilt_valid;
}

matrix::Matrix<float, 2, 2> TiltKinematics::compute_jacobian(const LinkageState& state) const
{
	// Use drive kinematics Jacobian as the primary source
	// Create temporary drive state from combined state
	TiltKinematicsDrive::DriveState drive_state{};
	drive_state.actuator_length = state.actuator_length;
	drive_state.bellcrank_angle = state.bellcrank_angle_drive;
	drive_state.condition_number = state.condition_number;
	drive_state.is_valid = state.is_valid;

	return _drive_kinematics.compute_jacobian(drive_state);
}

// =========================
// AS5600 ENCODER FUNCTIONS
// =========================

float TiltKinematics::encoder_angle_to_actuator_length(float encoder_angle) const
{
	// Apply offset calibration to get the actual geometric angle (deg)
	float geometric_angle = encoder_angle - encoder_angle_at_min;
	// Convert to radians
	float geometric_angle_rad = math::radians(geometric_angle);

	// Use law of cosines to get actuator length from drive kinematics configuration
	// OA = motor_base distance from boom pivot, OB = crank_joint_to_pivot_length
	float OA = _drive_kinematics.get_configuration().motor_base.norm(); // mm, pivot to actuator base
	float OB = _drive_kinematics.get_configuration().crank_joint_to_pivot_length; // mm, pivot to crank joint
	// AB = sqrt(OA^2 + OB^2 - 2*OA*OB*cos(angle))
	float actuator_length = sqrtf(OA*OA + OB*OB - 2.0f*OA*OB*cosf(geometric_angle_rad));
	return actuator_length;
}

float TiltKinematics::encoder_angle_to_boom_angle(float encoder_angle) const
{
	float actuator_length = encoder_angle_to_actuator_length(encoder_angle);
	// Use drive kinematics to get bellcrank angle, then tilt kinematics to get tilt angle
	// For now, just use drive kinematics as a placeholder
	auto drive_state = _drive_kinematics.compute_forward_kinematics(actuator_length, 0.0f);
	// If you want the actual boom angle, you may need to add more geometry here
	return drive_state.bellcrank_angle;
}

TiltKinematics::LinkageState TiltKinematics::get_kinematic_state_from_encoder(float encoder_angle) const
{
	float actuator_length = encoder_angle_to_actuator_length(encoder_angle);
	// For now, assume boom_angle = 0.0f (should be replaced with actual calculation if needed)
	return compute_forward_kinematics(actuator_length, 0.0f);
}

