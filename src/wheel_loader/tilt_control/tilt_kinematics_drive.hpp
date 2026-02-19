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

#pragma once

// System includes
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

// Standard library
#include <cmath>

/**
 * @brief Bucket drive kinematics solver for wheel loader Stage 1 actuation linkage
 *
 * This class handles the kinematic calculations for Stage 1 of the wheel loader bucket system:
 *
 * STAGE 1: BUCKET ACTUATION LINKAGE (Primary - OABC mechanism):
 *    - Coordinate System: Machine body frame (X=Forward, Y=Left, Z=Up)
 *    - O: Boom pivot point (origin, fixed to machine body)
 *    - A: Linear actuator base attachment
 *    - B: Actuator-bellcrank joint (moving)
 *    - C: Bellcrank-boom attachment (moves with boom)
 *    - Function: Converts actuator length + boom angle → bellcrank angle ∠OCB₁
 *
 * SOLUTION METHOD: Analytical trigonometric approach using:
 *    - Law of cosines for link constraints
 *    - Circle intersections for joint positions
 *    - Direct atan2() calculations for angles
 *    - O(1) constant time complexity
 */
class TiltKinematicsDrive : public ModuleParams
{
public:
	/**
	 * @brief Drive linkage configuration structure
	 */
	struct DriveConfiguration {
		// Attachment points (mm, relative to boom pivot)
		matrix::Vector2f motor_base;                    // Motor base position (point A)
		float crank_joint_to_pivot_length;             // OC length parameter
		float crank_joint_to_pivot_angle;              // Precalculated crank joint to pivot angle

		// Linkage dimensions (mm)
		float drive_bellcrank_length;                  // BC length parameter

		// Angular offsets (rad) - calculated from triangle geometry in parent TiltKinematics
		float bellcrank_boom_alignment_offset;         // Angle BAC in boom pivot triangle
		float coupler_to_pivot_angle;                  // Angle BCA in boom pivot triangle

		// Physical limits
		float actuator_min_length;                     // AB minimum length from quad encoder
		float actuator_max_length;                     // AB maximum length from quad encoder
	};

	/**
	 * @brief Drive linkage state description
	 */
	struct DriveState {
		// Primary outputs
		float actuator_length;     // mm - linear actuator extension
		float bellcrank_angle;     // rad - Stage 1 bellcrank angle ∠OCB₁

		// Validation
		bool is_valid;            // True if within mechanical limits
		float condition_number;   // Linkage condition (singularity detection)
	};

	explicit TiltKinematicsDrive(ModuleParams *parent);

	/**
	 * @brief Forward kinematics: actuator length -> bellcrank angle
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param actuator_length AB length from quad encoder (mm)
	 * @param boom_angle Current boom angle from AS5600 (rad)
	 * @return Drive linkage state
	 */
	DriveState compute_forward_kinematics(float actuator_length, float boom_angle = 0.0f) const;

	/**
	 * @brief Inverse kinematics: bellcrank angle -> actuator length
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param bellcrank_angle Desired bellcrank angle (rad)
	 * @param boom_angle Current boom angle from AS5600 (rad)
	 * @return Required actuator length, or -1 if no valid solution
	 */
	float compute_inverse_kinematics(float bellcrank_angle, float boom_angle = 0.0f) const;

	/**
	 * @brief Validate drive configuration
	 * @return True if configuration is mechanically sound
	 */
	bool validate_configuration() const;

	/**
	 * @brief Get Jacobian matrix for sensitivity analysis
	 * @param state Current drive state
	 * @return 2x2 Jacobian [dbellcrank/dlength, dbellcrank/dboom]
	 */
	matrix::Matrix<float, 2, 2> compute_jacobian(const DriveState &state) const;

	/**
	 * @brief Update configuration from parameters
	 */
	void update_configuration();

	/**
	 * @brief Set calculated angles from parent triangle geometry
	 * @param bellcrank_boom_alignment_offset Angle BAC in boom pivot triangle
	 * @param coupler_to_pivot_angle Angle BCA in boom pivot triangle
	 */
	void set_triangle_angles(float bellcrank_boom_alignment_offset,
				 float coupler_to_pivot_angle);

	/**
	 * @brief Get current configuration
	 */
	const DriveConfiguration &get_configuration() const { return _config; }

private:
	DriveConfiguration _config;

	// Drive kinematic parameters (following 16-char limit)
	DEFINE_PARAMETERS(
		// Stage 1 - Bucket Actuation Linkage (Machine Body Frame)
		(ParamFloat<px4::params::BCT_MOTOR_BASE_X>) _param_motor_base_x,
		(ParamFloat<px4::params::BCT_MOTOR_BASE_Y>) _param_motor_base_y,
		(ParamFloat<px4::params::BCT_CRK_PIV_LEN>) _param_crank_pivot_len,
		(ParamFloat<px4::params::BCT_CRK_PIV_ANG>) _param_crank_pivot_ang,

		// Link dimensions
		(ParamFloat<px4::params::BCT_BELLCRK_LEN>) _param_bellcrank_len,

		// Physical and safety limits from quad encoder
		(ParamFloat<px4::params::BCT_ENC_POS_MIN>) _param_encoder_pos_min,
		(ParamFloat<px4::params::BCT_ENC_POS_MAX>) _param_encoder_pos_max
	);

	// =========================
	// TRIGONOMETRIC SOLUTION METHODS
	// =========================

	/**
	 * @brief Solve Stage 1 linkage using trigonometric approach
	 * Machine body frame: AB length + boom angle → bellcrank angle
	 * @param actuator_length Current AB length from quad encoder (mm)
	 * @param boom_angle Current boom angle from AS5600 (rad)
	 * @param state Output drive state
	 * @return True if valid solution found
	 */
	bool solve_trigonometric(float actuator_length, float boom_angle, DriveState &state) const;

	/**
	 * @brief Inverse solve: Required AB length for bellcrank angle
	 * @param bellcrank_angle Required bellcrank angle (rad)
	 * @param boom_angle Current boom angle from AS5600 (rad)
	 * @return Required AB length, or -1 if no valid solution
	 */
	float solve_inverse_trigonometric(float bellcrank_angle, float boom_angle) const;

	// =========================
	// GEOMETRIC HELPER FUNCTIONS
	// =========================

	/**
	 * @brief Calculate angle using law of cosines: cos(C) = (a²+b²-c²)/(2ab)
	 * @param side_a Length of side a
	 * @param side_b Length of side b
	 * @param side_c Length of side c (opposite to angle C)
	 * @return Angle C in radians, or NaN if triangle impossible
	 */
	float law_of_cosines_angle(float side_a, float side_b, float side_c) const;

	/**
	 * @brief Calculate bellcrank joint (point C) position from boom angle
	 * @param boom_angle Current boom angle from AS5600 (rad)
	 * @return Point C position in machine body frame
	 */
	matrix::Vector2f calculate_bellcrank_joint_position(float boom_angle) const;

	/**
	 * @brief Calculate angle OCA (angle at point C between OC and CA)
	 * @param point_C Bellcrank joint position
	 * @param point_A Motor base position
	 * @return Angle OCA in radians
	 */
	float calculate_angle_OCA(const matrix::Vector2f &point_C, const matrix::Vector2f &point_A) const;

	// =========================
	// VALIDATION AND ANALYSIS
	// =========================

	/**
	 * @brief Check mechanical limits and validate drive state
	 * @param state Drive state to validate
	 * @return True if state is mechanically valid and within limits
	 */
	bool check_mechanical_limits(const DriveState &state) const;



	/**
	 * @brief Compute drive condition number for sensitivity analysis
	 * @param state Current drive state
	 * @return Condition number (higher = closer to singularity)
	 */
	float compute_condition_number(const DriveState &state) const;

	/**
	 * @brief Validate that triangle with given side lengths is possible
	 * @param a Side length a
	 * @param b Side length b
	 * @param c Side length c
	 * @return True if triangle inequality satisfied
	 */
	bool is_triangle_valid(float a, float b, float c) const;

	// Computation constants for trigonometric solutions
	static constexpr float GEOMETRIC_TOLERANCE = 1e-6f;      // Geometric precision
	static constexpr float MAX_CONDITION_NUMBER = 100.0f;     // Singularity threshold
	static constexpr float PI = 3.14159265359f;              // Mathematical constant
};
