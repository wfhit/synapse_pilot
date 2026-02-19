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

#include "boom_kinematics.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

BoomKinematics::BoomKinematics(ModuleParams *parent) :
	ModuleParams(parent)
{
	// Initialize configuration
	update_configuration();
}

float BoomKinematics::actuator_length_to_boom_angle(float actuator_length) const
{
	// Forward kinematics: Given actuator length, find boom angle
	// Using triangle formed by boom_pivot, actuator_mount, and actuator_boom_joint

	float pivot_to_base_distance = _config.actuator_base_to_pivot_length;  // Distance from pivot to actuator base mount
	float pivot_to_joint_distance = _config.actuator_joint_to_pivot_length;  // Distance from pivot to actuator joint
	float base_to_joint_distance = actuator_length;  // Actuator cylinder length

	// Use law of cosines to find angle at boom pivot
	float pivot_triangle_angle = law_of_cosines_angle(pivot_to_base_distance, pivot_to_joint_distance,
				     base_to_joint_distance);

	// Use cached angle of pivot-to-base line from horizontal
	float base_mount_angle = _config.actuator_base_to_pivot_angle;

	// Calculate angle of pivot-to-actuator-joint line from horizontal
	float joint_angle_from_horizontal = base_mount_angle + pivot_triangle_angle;

	// Calculate boom angle (pivot-to-bucket line from horizontal)
	// Add the fixed angle between actuator joint and bucket joint
	float boom_angle = joint_angle_from_horizontal + _config.actuator_joint_to_boom_diff_angle;

	return boom_angle;
}

float BoomKinematics::boom_angle_to_actuator_length(float boom_angle) const
{
	// Inverse kinematics: Given boom angle, find required actuator length

	// Calculate angle of actuator joint line from horizontal
	float joint_angle_from_horizontal = boom_angle - _config.actuator_joint_to_boom_diff_angle;

	// Use cached angle of actuator base mount line from horizontal
	float base_mount_angle = _config.actuator_base_to_pivot_angle;

	// Calculate angle at boom pivot
	float pivot_triangle_angle = joint_angle_from_horizontal - base_mount_angle;

	float pivot_to_base_distance = _config.actuator_base_to_pivot_length;  // Distance from pivot to base mount
	float pivot_to_joint_distance = _config.actuator_joint_to_pivot_length;  // Distance from pivot to joint

	// Use law of cosines to find actuator length
	float required_actuator_length = law_of_cosines_side(pivot_to_base_distance, pivot_to_joint_distance,
					 pivot_triangle_angle);

	return required_actuator_length;
}

float BoomKinematics::encoder_angle_to_actuator_length(float encoder_angle) const
{
	// Convert AS5600 encoder angle to actuator length
	// The AS5600 returns absolute angle, so relationship with geometric angle OAB is just an offset

	// Apply offset calibration to get the actual geometric angle OAB
	// encoder_angle_at_min corresponds to the encoder reading when actuator is at minimum length
	float geometric_angle_oab = encoder_angle - _config.encoder_angle_at_min;

	// Convert geometric angle OAB to actuator length using law of cosines
	float pivot_to_base_distance = _config.actuator_base_to_pivot_length;  // OA - pivot to actuator base
	float pivot_to_joint_distance = _config.actuator_joint_to_pivot_length;  // OB - pivot to actuator joint

	// Use law of cosines: AB = sqrt(OA² + OB² - 2*OA*OB*cos(angle_OAB))
	float calculated_actuator_length = law_of_cosines_side(pivot_to_base_distance, pivot_to_joint_distance,
					   geometric_angle_oab);

	return calculated_actuator_length;
}

float BoomKinematics::sensor_angle_to_actuator_length(float sensor_angle) const
{
	// Convert AS5600 sensor angle directly to actuator length using geometric relationship
	// This function uses the triangular relationship between chassis reference, pivot, and boom end
	// to calculate actuator length directly from the sensor angle measurement.

	// Get geometric parameters from configuration
	const float chassis_to_pivot_length =
		_config.actuator_base_to_pivot_length;  // Distance from chassis reference to pivot
	const float chassis_to_boom_end_length = _config.boom_length;  // Distance from chassis reference to boom end

	// Get valid angle range
	const float angle_min = _config.encoder_angle_at_min;    // Minimum sensor angle (degrees)
	const float angle_max = _config.encoder_angle_at_max;    // Maximum sensor angle (degrees)

	// Constrain angle to expected operating range
	float constrained_angle = math::constrain(sensor_angle, angle_min, angle_max);
	float constrained_angle_rad = math::radians(constrained_angle);

	// Use law of cosines to calculate actuator length in triangle chassis-pivot-boom_end
	// Given: chassis_to_pivot, chassis_to_boom_end, and angle at pivot
	// We want to find the actuator length (side connecting chassis to boom_end through actuator)
	float actuator_length = law_of_cosines_side(chassis_to_pivot_length, chassis_to_boom_end_length, constrained_angle_rad);

	if (actuator_length < 0.0f || !PX4_ISFINITE(actuator_length)) {
		// Law of cosines failed, use encoder calibration as fallback
		PX4_WARN("Geometric calculation failed, using encoder calibration fallback");
		actuator_length = encoder_angle_to_actuator_length(sensor_angle);
	}

	// Note: Bounds checking is handled by the calling hardware interface
	// which has access to the physical actuator min/max limits

	PX4_DEBUG("Sensor angle %.2f° -> Actuator length %.2f mm (chassis_to_pivot=%.1f, chassis_to_boom_end=%.1f)",
		  (double)sensor_angle, (double)actuator_length,
		  (double)chassis_to_pivot_length, (double)chassis_to_boom_end_length);

	return actuator_length;
}

float BoomKinematics::actuator_length_to_encoder_angle(float actuator_length) const
{
	// Convert actuator length to boom angle using forward kinematics
	float calculated_boom_angle = actuator_length_to_boom_angle(actuator_length);

	// Calculate encoder angle: Direct offset relationship with AS5600
	float calculated_encoder_angle = math::degrees(calculated_boom_angle) + _config.encoder_angle_at_min;

	// Wrap to 0-360 degrees for encoder
	calculated_encoder_angle = fmodf(calculated_encoder_angle + 360.0f, 360.0f);

	return calculated_encoder_angle;
}

void BoomKinematics::calculate_bucket_position(float boom_angle, float &x_pos, float &z_pos) const
{
	// Calculate bucket position from boom angle
	float boom_length = _config.boom_length;

	x_pos = boom_length * cosf(boom_angle);  // Horizontal reach
	z_pos = boom_length * sinf(boom_angle);  // Vertical height from pivot
}

float BoomKinematics::calculate_bucket_height(float boom_angle) const
{
	// Calculate bucket height from ground
	float bucket_height_from_pivot = _config.boom_length * sinf(boom_angle);
	float total_bucket_height = _config.pivot_height_from_ground + bucket_height_from_pivot;

	return total_bucket_height;
}

float BoomKinematics::calculate_mechanical_advantage(float boom_angle) const
{
	// Calculate mechanical advantage at current boom position

	// Calculate moment arms
	float pivot_to_base_distance = _config.actuator_base_to_pivot_length;
	float joint_angle_from_horizontal = boom_angle - _config.actuator_joint_to_boom_diff_angle;
	float base_mount_angle = _config.actuator_base_to_pivot_angle;
	float pivot_triangle_angle = joint_angle_from_horizontal - base_mount_angle;

	// Actuator moment arm (perpendicular distance from pivot to actuator force line)
	float actuator_moment_arm = pivot_to_base_distance * sinf(pivot_triangle_angle);

	// Boom moment arm (perpendicular distance from pivot to load application point)
	float boom_moment_arm = _config.boom_length * sinf(boom_angle);

	// Mechanical advantage (force multiplication ratio)
	float mechanical_advantage_ratio = actuator_moment_arm / boom_moment_arm;

	return fabsf(mechanical_advantage_ratio);
}

BoomKinematics::KinematicState BoomKinematics::get_kinematic_state_from_actuator_length(float actuator_length) const
{
	KinematicState kinematic_state = {};

	// Calculate boom angle
	kinematic_state.boom_angle = actuator_length_to_boom_angle(actuator_length);
	kinematic_state.actuator_length = actuator_length;

	// Calculate bucket position
	float bucket_x_position, bucket_z_position;
	calculate_bucket_position(kinematic_state.boom_angle, bucket_x_position, bucket_z_position);
	kinematic_state.bucket_reach = bucket_x_position;
	kinematic_state.bucket_height = calculate_bucket_height(kinematic_state.boom_angle);

	// Calculate mechanical advantage
	kinematic_state.mechanical_advantage = calculate_mechanical_advantage(kinematic_state.boom_angle);

	// Validate position
	kinematic_state.is_valid = is_position_valid(kinematic_state.boom_angle);

	return kinematic_state;
}

BoomKinematics::KinematicState BoomKinematics::get_kinematic_state_from_encoder(float encoder_angle) const
{
	float calculated_actuator_length = encoder_angle_to_actuator_length(encoder_angle);
	return get_kinematic_state_from_actuator_length(calculated_actuator_length);
}

bool BoomKinematics::is_position_valid(float boom_angle) const
{
	// Check for basic triangle validity
	float required_actuator_length = boom_angle_to_actuator_length(boom_angle);

	// Check for physical constraints (triangle inequality)
	float pivot_to_base_distance = _config.actuator_base_to_pivot_length;
	float pivot_to_joint_distance = _config.actuator_joint_to_pivot_length;
	float base_to_joint_distance = required_actuator_length;

	if (base_to_joint_distance > (pivot_to_base_distance + pivot_to_joint_distance) ||
	    base_to_joint_distance < fabsf(pivot_to_base_distance - pivot_to_joint_distance)) {
		return false;
	}

	return true;
}

void BoomKinematics::update_configuration()
{
	// Load configuration parameters using DEFINE_PARAMETERS approach
	_config.boom_length = _param_boom_length.get();
	_config.actuator_joint_to_pivot_length = _param_actuator_joint_to_pivot_length.get();
	_config.actuator_base_to_pivot_length = _param_actuator_base_to_pivot_length.get();
	_config.actuator_joint_to_boom_end_length = _param_actuator_joint_to_boom_end_length.get();
	_config.actuator_length_at_zero = _param_actuator_zero_length.get();
	_config.pivot_height_from_ground = _param_pivot_height.get();
	_config.encoder_angle_at_min = _param_encoder_min_angle.get();
	_config.encoder_angle_at_max = _param_encoder_max_angle.get();
	_config.encoder_angle_at_zero = _param_encoder_zero_angle.get();

	// Calculate computed values
	_config.actuator_joint_to_boom_diff_angle = calculate_actuator_joint_to_boom_diff_angle();

	// Calculate actuator_base_to_pivot_angle using pivot angle at zero length plus boom differential angle
	_config.actuator_base_to_pivot_angle = calculate_actuator_base_to_pivot_angle();
}

float BoomKinematics::calculate_actuator_base_to_pivot_angle() const
{
	// Calculate actuator_base_to_pivot_angle as:
	// pivot angle when actuator_length_at_zero + actuator_joint_to_boom_diff_angle

	// First calculate the pivot angle in the actuator triangle when actuator is at zero length
	float pivot_angle_at_zero_length = calculate_pivot_angle_from_actuator_length(_config.actuator_length_at_zero);

	// Add the actuator joint to boom differential angle
	float calculated_base_to_pivot_angle = pivot_angle_at_zero_length + _config.actuator_joint_to_boom_diff_angle;

	return calculated_base_to_pivot_angle;  // Return in radians
}

bool BoomKinematics::validate_configuration() const
{
	// Check for reasonable values
	if (_config.boom_length <= 0.0f ||
	    _config.actuator_joint_to_pivot_length <= 0.0f ||
	    _config.actuator_base_to_pivot_length <= 0.0f ||
	    _config.actuator_joint_to_boom_end_length <= 0.0f ||
	    _config.actuator_length_at_zero <= 0.0f ||
	    _config.pivot_height_from_ground <= 0.0f) {
		PX4_ERR("Invalid kinematic configuration - check parameters");
		return false;
	}

	// Check triangle inequality for boom structure (pivot, actuator joint, bucket)
	float boom_length = _config.boom_length;
	float pivot_to_joint_distance = _config.actuator_joint_to_pivot_length;
	float joint_to_bucket_distance = _config.actuator_joint_to_boom_end_length;

	// Triangle inequality: sum of any two sides must be greater than third side
	if ((pivot_to_joint_distance + boom_length <= joint_to_bucket_distance) ||
	    (pivot_to_joint_distance + joint_to_bucket_distance <= boom_length) ||
	    (boom_length + joint_to_bucket_distance <= pivot_to_joint_distance)) {
		PX4_ERR("Invalid boom geometry - triangle inequality violated");
		return false;
	}

	// Check encoder calibration
	if (_config.encoder_angle_at_max <= _config.encoder_angle_at_min) {
		PX4_WARN("Invalid encoder calibration - max angle must be greater than min angle");
		return false;
	}

	return true;
}

float BoomKinematics::calculate_pivot_angle_from_actuator_length(float actuator_length) const
{
	// Calculate angle at boom pivot in actuator triangle
	float pivot_to_base_distance = _config.actuator_base_to_pivot_length;
	float pivot_to_joint_distance = _config.actuator_joint_to_pivot_length;
	float base_to_joint_distance = actuator_length;

	return law_of_cosines_angle(pivot_to_base_distance, pivot_to_joint_distance, base_to_joint_distance);
}

float BoomKinematics::calculate_actuator_joint_to_boom_diff_angle() const
{
	// Calculate angle between actuator joint and boom centerline
	// This is a fixed geometric relationship based on boom design

	// Using the triangle formed by boom_pivot, actuator_joint, and bucket_joint
	float pivot_to_joint_distance = _config.actuator_joint_to_pivot_length;
	float pivot_to_bucket_distance = _config.boom_length;
	float joint_to_bucket_distance = _config.actuator_joint_to_boom_end_length;

	// Use law of cosines to find angle at boom pivot between boom centerline and actuator joint
	return law_of_cosines_angle(pivot_to_bucket_distance, pivot_to_joint_distance, joint_to_bucket_distance);
}

float BoomKinematics::law_of_cosines_side(float side_a, float side_b, float included_angle) const
{
	// Calculate the third side of a triangle given two sides and included angle
	// c² = a² + b² - 2ab*cos(C)
	float side_c_squared = side_a * side_a + side_b * side_b - 2.0f * side_a * side_b * cosf(included_angle);
	return sqrtf(side_c_squared);
}

float BoomKinematics::law_of_cosines_angle(float side_a, float side_b, float opposite_side) const
{
	// Calculate angle C opposite to side c
	// cos(C) = (a² + b² - c²) / (2ab)
	float cosine_value = (side_a * side_a + side_b * side_b - opposite_side * opposite_side) / (2.0f * side_a * side_b);
	return acosf(math::constrain(cosine_value, -1.0f, 1.0f));
}
