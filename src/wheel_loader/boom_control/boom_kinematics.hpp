/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#pragma once

#include <px4_platform_common/module_params.h>
#include <mathlib/mathlib.h>

/**
 * @brief Boom Kinematics Calculator for Wheel Loader
 *
 * @section design Design Overview
 *
 * This class implements the kinematic model for a wheel loader boom mechanism
 * controlled by a linear hydraulic actuator. The system uses triangular geometry
 * to convert between actuator positions and boom angles.
 *
 * @subsection coordinate_system Coordinate System
 *
 * The coordinate system origin is located at the boom pivot point.
 * For detailed visual representation, see: docs/assets/diagrams/boom_kinematics_diagram.svg
 *
 * Key Points:
 * - O (boom_pivot): Origin at boom rotation axis
 * - A (actuator_mount): Hydraulic cylinder mount on chassis
 * - B (actuator_boom_joint): Cylinder attachment on boom
 * - C (bucket_joint): Bucket attachment at boom tip
 * - D (ground_point): Ground reference below bucket
 *
 * Triangular relationships:
 * - Variable triangle O-A-B: Changes with actuator extension
 * - Fixed triangle O-B-C: Rigid boom structure
 * - Reference O-C-D: For height and reach calculations
 *
 * @subsection geometric_model Geometric Model
 *
 * The boom mechanism consists of three key triangles:
 *
 * 1. **Variable Triangle**: (boom_pivot, actuator_mount, actuator_boom_joint)
 *    - Changes as the linear actuator extends/retracts
 *    - Actuator length determines the triangle geometry
 *
 * 2. **Fixed Triangle**: (boom_pivot, actuator_boom_joint, bucket_joint)
 *    - Rigid structure welded to the boom
 *    - Fixed angle between actuator joint and bucket joint
 *
 * 3. **Ground Reference**: (boom_pivot, bucket_joint, ground_point)
 *    - Used for bucket height and reach calculations
 *
 * @subsection kinematic_chain Kinematic Chain
 *
 * The kinematic solution follows this chain:
 * 1. AS5600 encoder angle → Actuator cylinder rotation angle
 * 2. Actuator rotation angle → Actuator length (using law of cosines)
 * 3. Actuator length → Boom angle (through triangle geometry)
 * 4. Boom angle → Bucket position (forward kinematics)
 *
 * Inverse kinematics works in reverse:
 * 1. Desired bucket position → Required boom angle
 * 2. Required boom angle → Required actuator length
 * 3. Required actuator length → AS5600 target angle
 *
 * @subsection sensor_integration Sensor Integration
 *
 * The AS5600 magnetic encoder measures the rotation of the actuator cylinder
 * around its mounting point (actuator_mount). This angle changes as the boom
 * moves up and down, providing feedback for position control.
 *
 * Key relationships:
 * - Encoder measures cylinder rotation relative to minimum position
 * - Calibration maps encoder angles to actuator lengths
 * - Non-linear relationship due to triangular geometry
 *
 * @subsection mechanical_limits Mechanical Limits
 *
 * The system enforces several types of limits:
 * - **Actuator Limits**: Minimum/maximum cylinder extension
 * - **Boom Angle Limits**: Safe operating range for boom
 * - **Geometric Constraints**: Triangle validity checks
 * - **Mechanical Advantage**: Force multiplication calculations
 *
 * @section parameters Key Parameters
 *
 * - `actuator_base_to_pivot_length`: Distance from boom pivot to actuator base mount point
 * - `actuator_base_to_pivot_angle`: Angle of actuator base mount from horizontal
 * - `actuator_joint_to_pivot_length`: Distance from actuator joint attachment to boom pivot
 * - `actuator_joint_to_boom_end_length`: Distance between actuator joint and boom end
 * - `actuator_length_at_zero`: Actuator length when boom is at zero angle
 * - `actuator_joint_to_boom_diff_angle`: Calculated angle between actuator joint and boom centerline
 * - `encoder_angle_at_min/max`: AS5600 calibration points
 *
 * @section usage Usage Example
 *
 * ```cpp
 * BoomKinematics kinematics(this);
 * kinematics.update_configuration();
 *
 * // Get state from encoder
 * float encoder_angle = as5600_reader.get_angle();
 * auto state = kinematics.get_kinematic_state_from_encoder(encoder_angle);
 *
 * // Command boom to desired angle
 * float desired_angle = math::radians(30.0f);
 * float target_length = kinematics.boom_angle_to_actuator_length(desired_angle);
 * float target_encoder = kinematics.actuator_length_to_encoder_angle(target_length);
 * ```
 */
class BoomKinematics : public ModuleParams
{
public:
	struct Configuration {
		// Chassis mounting points (mm from boom_pivot origin)
		float actuator_base_to_pivot_length;   // mm - Distance from boom pivot to actuator base mount point

		// Boom geometry
		float boom_length;                   // mm - Length of boom from pivot to bucket joint
		float actuator_joint_to_pivot_length;      // mm - Distance from actuator joint attachment to boom pivot
		float actuator_joint_to_boom_end_length;     // mm - Length between actuator joint and boom end attachment
		float actuator_length_at_zero;       // mm - Actuator length when boom is at zero angle

		// AS5600 encoder calibration
		float encoder_angle_at_min;          // deg - Encoder reading at minimum extension
		float encoder_angle_at_max;          // deg - Encoder reading at maximum extension
		float encoder_angle_at_zero;         // deg - Encoder reading when boom angle is zero

		// System dimensions
		float pivot_height_from_ground;      // mm - Height of boom pivot from ground level

		// Computed values (cached when parameters change)
		float actuator_base_to_pivot_angle;          // rad - Calculated angle AOB (actuator_base, boom_pivot, actuator_joint)
		float actuator_joint_to_boom_diff_angle;           // rad - Calculated angle between actuator joint and boom centerline

	};

	struct KinematicState {
		float boom_angle;                    // rad - Boom centerline angle from horizontal
		float actuator_length;               // mm - Current cylinder extension
		float bucket_height;                 // mm - Bucket height from ground
		float bucket_reach;                  // mm - Horizontal distance from pivot to bucket
		float mechanical_advantage;          // Force multiplication ratio
		bool is_valid;                      // True if within all limits
	};

	explicit BoomKinematics(ModuleParams *parent);

	/**
	 * @brief Convert actuator length to boom angle
	 * @param actuator_length Cylinder extension in mm
	 * @return Boom angle from horizontal in radians
	 */
	float actuator_length_to_boom_angle(float actuator_length) const;

	/**
	 * @brief Convert boom angle to required actuator length
	 * @param boom_angle Desired boom angle from horizontal in radians
	 * @return Required cylinder extension in mm
	 */
	float boom_angle_to_actuator_length(float boom_angle) const;

	/**
	 * @brief Convert AS5600 encoder angle to actuator length
	 * @param encoder_angle AS5600 angle reading in degrees
	 * @return Cylinder extension in mm
	 *
	 * The AS5600 measures rotation of cylinder around its mount point.
	 * Encoder is calibrated to map angles to min/max cylinder positions.
	 */
	float encoder_angle_to_actuator_length(float encoder_angle) const;

	/**
	 * @brief Convert AS5600 sensor angle directly to actuator length using geometric relationship
	 * @param sensor_angle AS5600 angle reading in degrees
	 * @return Cylinder extension in mm
	 *
	 * This function uses the triangular relationship between the chassis reference point,
	 * pivot point, and boom end to calculate actuator length directly from the sensor angle.
	 * More accurate than encoder calibration for real-time control.
	 */
	float sensor_angle_to_actuator_length(float sensor_angle) const;

	/**
	 * @brief Convert actuator length to AS5600 encoder angle
	 * @param actuator_length Cylinder extension in mm
	 * @return Expected encoder angle in degrees
	 */
	float actuator_length_to_encoder_angle(float actuator_length) const;

	/**
	 * @brief Calculate bucket position from boom angle
	 * @param boom_angle Boom angle in radians
	 * @param x_pos Output: Horizontal position of bucket
	 * @param z_pos Output: Vertical position of bucket
	 */
	void calculate_bucket_position(float boom_angle, float &x_pos, float &z_pos) const;

	/**
	 * @brief Calculate bucket height from ground
	 * @param boom_angle Boom angle in radians
	 * @return Height of bucket from ground in mm
	 */
	float calculate_bucket_height(float boom_angle) const;

	/**
	 * @brief Calculate mechanical advantage at current position
	 * @param boom_angle Current boom angle in radians
	 * @return Mechanical advantage ratio
	 */
	float calculate_mechanical_advantage(float boom_angle) const;

	/**
	 * @brief Get complete kinematic state for given actuator length
	 * @param actuator_length Cylinder extension in mm
	 * @return Complete kinematic state structure
	 */
	KinematicState get_kinematic_state_from_actuator_length(float actuator_length) const;

	/**
	 * @brief Get complete kinematic state from AS5600 encoder reading
	 * @param encoder_angle AS5600 angle in degrees
	 * @return Complete kinematic state structure
	 */
	KinematicState get_kinematic_state_from_encoder(float encoder_angle) const;

	/**
	 * @brief Check if position is within mechanical limits
	 * @param boom_angle Boom angle to check in radians
	 * @return True if position is valid
	 */
	bool is_position_valid(float boom_angle) const;

	/**
	 * @brief Update configuration from parameters
	 */
	void update_configuration();

	/**
	 * @brief Validate kinematic configuration
	 * @return True if configuration is valid
	 */
	bool validate_configuration() const;

	const Configuration &get_configuration() const { return _config; }

private:
	Configuration _config;

	// Kinematic parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BOOM_PIV_BASE>) _param_actuator_base_to_pivot_length,
		(ParamFloat<px4::params::BOOM_PIV_BUK_LEN>) _param_boom_length,
		(ParamFloat<px4::params::BOOM_ACT_PIV_LEN>) _param_actuator_joint_to_pivot_length,
		(ParamFloat<px4::params::BOOM_ACT_JNT_LEN>) _param_actuator_joint_to_boom_end_length,
		(ParamFloat<px4::params::BOOM_ACT_ZERO>) _param_actuator_zero_length,
		(ParamFloat<px4::params::BOOM_PIV_HEIGHT>) _param_pivot_height,
		(ParamFloat<px4::params::BOOM_ENC_OFF>) _param_sensor_boom_offset,
		(ParamFloat<px4::params::BOOM_ENC_MIN_ANG>) _param_encoder_min_angle,
		(ParamFloat<px4::params::BOOM_ENC_MAX_ANG>) _param_encoder_max_angle,
		(ParamFloat<px4::params::BOOM_ENC_ZERO>) _param_encoder_zero_angle
	)

	/**
	 * @brief Calculate angle at boom pivot in actuator triangle
	 * @param actuator_length Cylinder extension in mm
	 * @return Angle at boom pivot in radians
	 */
	float calculate_pivot_angle_from_actuator_length(float actuator_length) const;

	/**
	 * @brief Calculate angle using law of cosines
	 * @param a First side length
	 * @param b Second side length
	 * @param c Third side length (opposite to angle)
	 * @return Angle in radians
	 */
	float law_of_cosines_angle(float a, float b, float c) const;

	/**
	 * @brief Calculate side length using law of cosines
	 * @param a First side length
	 * @param b Second side length
	 * @param gamma_angle Angle between sides a and b in radians
	 * @return Third side length
	 */
	float law_of_cosines_side(float a, float b, float gamma_angle) const;

	/**
	 * @brief Calculate actuator joint to boom differential angle using triangle geometry
	 * @return Angle in radians
	 */
	float calculate_actuator_joint_to_boom_diff_angle() const;

	/**
	 * @brief Calculate actuator base to pivot angle using triangle geometry
	 * @return Angle in radians
	 */
	float calculate_actuator_base_to_pivot_angle() const;
};
