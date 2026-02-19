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

#pragma once

// Local includes
#include "tilt_kinematics_drive.hpp"
#include "tilt_kinematics_tilt.hpp"

// System includes
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

// Standard library
#include <cmath>

/**
 * @brief Bucket kinematics coordinator for wheel loader dual-linkage mechanism
 *
 * This class coordinates the kinematic calculations for the wheel loader bucket system,
 * which consists of two mechanically coupled four-bar linkages managed by separate classes:
 *
 * STAGE 1: BUCKET ACTUATION LINKAGE (Primary - handled by TiltKinematicsDrive):
 *    - Coordinate System: Machine body frame (X=Forward, Y=Left, Z=Up)
 *    - Function: Converts actuator length + boom angle → bellcrank angle ∠OCB₁
 *
 * STAGE 2: BUCKET TILT LINKAGE (Secondary - handled by TiltKinematicsTilt):
 *    - Coordinate System: Boom-end frame (X=Boom-forward, Y=Left, Z=Up)
 *    - Function: Converts bellcrank angle ∠OCB₂ → final tilt angle ∠AOC
 *
 * MECHANICAL COUPLING: ∠OCB₁ = ∠OCB₂ + α (handled by tilt class)
 *
 * This coordinator provides the same interface as the original TiltKinematics class
 * but delegates the actual calculations to the specialized drive and tilt classes.
 */
class TiltKinematics : public ModuleParams
{
public:
	/**
	 * @brief Complete linkage state description (maintains backward compatibility)
	 */
	struct LinkageState {
		// Primary outputs
		float actuator_length;     // mm - linear actuator extension
		float bucket_angle;        // rad - final tilt angle (ground-relative)
		float bellcrank_angle_drive;  // rad - Stage 1 bellcrank angle ∠OCB₁
		float bellcrank_angle_tilt;  // rad - Stage 2 bellcrank angle ∠OCB₂

		// Validation
		bool is_valid;            // True if within mechanical limits
		float condition_number;   // Linkage condition (singularity detection)
	};

	explicit TiltKinematics(ModuleParams *parent);

	/**
	 * @brief Forward kinematics: actuator length -> tilt angle
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param actuator_length Actuator extension length (mm)
	 * @param boom_angle Current boom angle (rad)
	 * @return Complete linkage state
	 */
	LinkageState compute_forward_kinematics(float actuator_length, float boom_angle = 0.0f) const;

	/**
	 * @brief Inverse kinematics: tilt angle -> actuator length
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param bucket_angle Desired tilt angle (rad, ground-relative)
	 * @param boom_angle Current boom angle (rad)
	 * @return Required linkage state to achieve tilt angle
	 */
	LinkageState compute_inverse_kinematics(float bucket_angle, float boom_angle = 0.0f) const;

	/**
	 * @brief Validate kinematic configuration
	 * @return True if configuration is mechanically sound
	 */
	bool validate_configuration() const;

	/**
	 * @brief Get Jacobian matrix for sensitivity analysis
	 * @param state Current linkage state
	 * @return 2x2 Jacobian [dangle/dlength, dangle/dboom]
	 */
	matrix::Matrix<float, 2, 2> compute_jacobian(const LinkageState &state) const;

	/**
	 * @brief Update configuration from parameters
	 */
	void update_configuration();

	/**
	 * @brief Get access to drive kinematics component
	 */
	const TiltKinematicsDrive &get_drive_kinematics() const { return _drive_kinematics; }

	/**
	 * @brief Get access to tilt kinematics component
	 */
	const TiltKinematicsTilt &get_tilt_kinematics() const { return _tilt_kinematics; }

	/**
	 * @brief Calculate bellcrank boom alignment offset (angle BAC in boom triangle)
	 */
	float get_bellcrank_boom_alignment_offset() const;

	/**
	 * @brief Calculate coupler to pivot angle (angle BCA in boom triangle)
	 */
	float get_coupler_to_pivot_angle() const;

	/**
	 * @brief Calculate bellcrank internal angle (2π - angle ABC in bellcrank triangle)
	 */
	float get_bellcrank_internal_angle() const;

public:
	// AS5600 calibration/configuration fields (degrees)
	float encoder_angle_at_min = 0.0f;
	float encoder_angle_at_max = 360.0f;
	float encoder_angle_at_zero = 0.0f;

	// Conversion functions
	float encoder_angle_to_actuator_length(float encoder_angle) const;
	float encoder_angle_to_boom_angle(float encoder_angle) const;
	LinkageState get_kinematic_state_from_encoder(float encoder_angle) const;

private:
	TiltKinematicsDrive _drive_kinematics;
	TiltKinematicsTilt _tilt_kinematics;

	DEFINE_PARAMETERS(
		// Boom pivot triangle parameters (coordinates in machine body frame)
		(ParamFloat<px4::params::BCT_BOOM_LENGTH>)
		_param_boom_length,                            // Boom length from pivot to bucket (mm)
		(ParamFloat<px4::params::BCT_BOOM_PIV_CRK>)
		_param_boom_pivot_to_crank_joint_distance, // Distance from boom pivot to crank joint (mm)
		(ParamFloat<px4::params::BCT_BCK_CRK_DIST>)
		_param_bucket_to_crank_joint_distance,    // Distance from bucket base to crank joint (mm)

		// Bellcrank triangle parameters (coordinates in bellcrank frame)
		(ParamFloat<px4::params::BCT_CRK_ARM_LEN>) _param_bellcrank_arm_length,                 // Length of bellcrank arm (mm)
		(ParamFloat<px4::params::BCT_CRK_COUPLER>)
		_param_bellcrank_to_coupler_distance,        // Distance from bellcrank to coupler joint (mm)
		(ParamFloat<px4::params::BCT_CRK_ACT_DIST>)
		_param_bellcrank_to_actuator_distance  // Distance from bellcrank to actuator joint (mm)
	);

	/**
	 * @brief Convert drive state and tilt state to combined linkage state
	 * @param drive_state Drive linkage state
	 * @param tilt_state Tilt linkage state
	 * @return Combined linkage state
	 */
	LinkageState combine_states(const TiltKinematicsDrive::DriveState &drive_state,
				    const TiltKinematicsTilt::TiltState &tilt_state) const;

	/**
	 * @brief Calculate angle using law of cosines: cos(C) = (a²+b²-c²)/(2ab)
	 * @param side_a Length of side a
	 * @param side_b Length of side b
	 * @param side_c Length of side c (opposite to angle C)
	 * @return Angle C in radians, or NaN if triangle impossible
	 */
	float law_of_cosines_angle(float side_a, float side_b, float side_c) const;
};
