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

#include "wheel_loader_manual_bucket_mode.hpp"
#include <px4_platform_common/log.h>

WheelLoaderManualBucketMode::WheelLoaderManualBucketMode(ModuleParams *parent) :
	OperationModeBase(parent, "WheelLoaderManualBucket")
{
	// Load vehicle parameters
	loadVehicleParameters();
}

bool WheelLoaderManualBucketMode::activate()
{
	PX4_INFO("Activating Wheel Loader Manual Bucket Mode");

	// Check if required data is available
	if (!_vehicle_local_position_sub.copy(&_vehicle_local_position)) {
		PX4_WARN("Vehicle local position not available");
		return false;
	}

	if (!_vehicle_attitude_sub.copy(&_vehicle_attitude)) {
		PX4_WARN("Vehicle attitude not available");
		return false;
	}

	if (!_manual_control_setpoint_sub.copy(&_manual_control_setpoint)) {
		PX4_WARN("Manual control setpoint not available");
		return false;
	}

	// Update current state
	updateCurrentState();

	// Initialize target bucket pose from current state
	initializeBucketPose();

	set_active(true);

	// Publish initial control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = false;
	config.flag_control_rates_enabled = true;
	publish_control_config(config);

	PX4_INFO("Manual bucket mode activated - bucket at X:%.2f Z:%.2f heading:%.2f tilt:%.2f",
		 (double)_target_bucket_pose.x, (double)_target_bucket_pose.z,
		 (double)_target_bucket_pose.heading, (double)_target_bucket_pose.tilt);

	return true;
}

void WheelLoaderManualBucketMode::deactivate()
{
	PX4_INFO("Deactivating Wheel Loader Manual Bucket Mode");
	set_active(false);
}

void WheelLoaderManualBucketMode::update(float dt)
{
	// Update sensor data
	updateCurrentState();

	// Process RC inputs and compute bucket velocities
	processRCInputs(dt);

	// Integrate velocities to update target bucket pose
	updateBucketPose(dt);

	// Apply limits to bucket pose
	applyBucketLimits();

	// Compute inverse kinematics to get chassis + boom + tilt targets
	if (!computeInverseKinematics()) {
		PX4_DEBUG("IK failed - holding position");
	}

	// Publish control setpoints
	publishSetpoints();

	// Publish control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = false;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = false;
	config.flag_control_attitude_enabled = false;
	config.flag_control_rates_enabled = true;
	publish_control_config(config);
}

bool WheelLoaderManualBucketMode::is_valid() const
{
	bool position_valid = _vehicle_local_position.xy_valid && _vehicle_local_position.z_valid &&
			      (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 500_ms);

	bool attitude_valid = (hrt_elapsed_time(&_vehicle_attitude.timestamp) < 500_ms);

	bool rc_valid = (hrt_elapsed_time(&_manual_control_setpoint.timestamp) < 500_ms);

	return position_valid && attitude_valid && rc_valid;
}

void WheelLoaderManualBucketMode::processRCInputs(float dt)
{
	// Update RC data
	_manual_control_setpoint_sub.update(&_manual_control_setpoint);

	// RC Channel mapping (normalized -1 to +1):
	// x (roll):     Ch 1 - Bucket forward/backward velocity
	// y (pitch):    Ch 2 - Bucket height velocity
	// z (throttle): Ch 3 - Bucket heading rate
	// r (yaw):      Ch 4 - Bucket tilt rate

	float rc_x = applyDeadband(_manual_control_setpoint.roll, _control_params.rc_deadband);
	float rc_y = applyDeadband(_manual_control_setpoint.pitch, _control_params.rc_deadband);
	float rc_z = applyDeadband(_manual_control_setpoint.throttle, _control_params.rc_deadband);
	float rc_r = applyDeadband(_manual_control_setpoint.yaw, _control_params.rc_deadband);

	// Scale to velocities
	_bucket_vel_x = rc_x * _control_params.max_bucket_vel_x;
	_bucket_vel_z = rc_y * _control_params.max_bucket_vel_z;
	_bucket_rate_heading = rc_z * _control_params.max_bucket_rate_heading;
	_bucket_rate_tilt = rc_r * _control_params.max_bucket_rate_tilt;

	PX4_DEBUG("RC: x=%.2f y=%.2f z=%.2f r=%.2f -> vel_x=%.2f vel_z=%.2f rate_h=%.2f rate_t=%.2f",
		  (double)rc_x, (double)rc_y, (double)rc_z, (double)rc_r,
		  (double)_bucket_vel_x, (double)_bucket_vel_z,
		  (double)_bucket_rate_heading, (double)_bucket_rate_tilt);
}

float WheelLoaderManualBucketMode::applyDeadband(float input, float deadband)
{
	if (fabsf(input) < deadband) {
		return 0.0f;
	}

	// Scale from [deadband, 1.0] to [0, 1.0]
	float sign = (input > 0.0f) ? 1.0f : -1.0f;
	return sign * (fabsf(input) - deadband) / (1.0f - deadband);
}

void WheelLoaderManualBucketMode::updateBucketPose(float dt)
{
	// Integrate velocities to get target bucket pose
	_target_bucket_pose.x += _bucket_vel_x * dt;
	_target_bucket_pose.z += _bucket_vel_z * dt;
	_target_bucket_pose.heading += _bucket_rate_heading * dt;
	_target_bucket_pose.tilt += _bucket_rate_tilt * dt;

	// Wrap heading to [-pi, pi]
	_target_bucket_pose.heading = matrix::wrap_pi(_target_bucket_pose.heading);
}

void WheelLoaderManualBucketMode::applyBucketLimits()
{
	// Clamp bucket X position
	_target_bucket_pose.x = math::constrain(_target_bucket_pose.x,
						_vehicle_params.bucket_x_min,
						_vehicle_params.bucket_x_max);

	// Clamp bucket Z height
	_target_bucket_pose.z = math::constrain(_target_bucket_pose.z,
						_vehicle_params.bucket_z_min,
						_vehicle_params.bucket_z_max);

	// Clamp tilt angle
	_target_bucket_pose.tilt = math::constrain(_target_bucket_pose.tilt,
			_vehicle_params.tilt_min,
			_vehicle_params.tilt_max);
}

bool WheelLoaderManualBucketMode::computeInverseKinematics()
{
	// Simplified inverse kinematics
	// Assumes bucket is at end of boom, boom pivots from chassis

	// Target bucket position in body frame
	float bucket_x_body = _target_bucket_pose.x;
	float bucket_z = _target_bucket_pose.z;

	// Boom base is offset from chassis center
	float boom_base_x = _vehicle_params.chassis_to_boom_offset;

	// Vector from boom base to bucket
	float dx = bucket_x_body - boom_base_x;
	float dz = bucket_z;

	// Boom length (hypotenuse)
	float boom_extension = sqrtf(dx * dx + dz * dz);

	// Check if reachable
	if (boom_extension > _vehicle_params.boom_max) {
		PX4_DEBUG("Bucket position unreachable: boom_extension=%.2f > max=%.2f",
			  (double)boom_extension, (double)_vehicle_params.boom_max);

		// Clamp to max reach
		float scale = _vehicle_params.boom_max / boom_extension;
		dx *= scale;
		dz *= scale;
		boom_extension = _vehicle_params.boom_max;
	}

	if (boom_extension < _vehicle_params.boom_min) {
		boom_extension = _vehicle_params.boom_min;
	}

	// Boom angle from horizontal
	float boom_angle = atan2f(dz, dx);

	// Output boom position (could be extension or angle depending on actuator)
	_target_boom_position = boom_extension;

	// Tilt angle is directly from target
	_target_tilt_angle = _target_bucket_pose.tilt;

	// Chassis target: position bucket heading in world frame
	// Transform bucket position from body to world frame
	float cos_h = cosf(_current_chassis_state.heading);
	float sin_h = sinf(_current_chassis_state.heading);

	float bucket_x_world = _current_chassis_state.x + bucket_x_body * cos_h;
	float bucket_y_world = _current_chassis_state.y + bucket_x_body * sin_h;

	// Chassis should move to place bucket at desired world position
	// For now, keep chassis at current position and let bucket move in body frame
	_target_chassis_state.x = _current_chassis_state.x;
	_target_chassis_state.y = _current_chassis_state.y;
	_target_chassis_state.heading = _current_chassis_state.heading + _target_bucket_pose.heading;

	PX4_DEBUG("IK: bucket(%.2f, %.2f) -> boom=%.2f tilt=%.2f chassis_heading=%.2f",
		  (double)bucket_x_body, (double)bucket_z,
		  (double)_target_boom_position, (double)_target_tilt_angle,
		  (double)_target_chassis_state.heading);

	return true;
}

void WheelLoaderManualBucketMode::publishSetpoints()
{
	hrt_abstime now = hrt_absolute_time();

	// Compute chassis velocity to reach target position
	float chassis_error_x = _target_chassis_state.x - _current_chassis_state.x;
	float chassis_error_y = _target_chassis_state.y - _current_chassis_state.y;
	float heading_error = matrix::wrap_pi(_target_chassis_state.heading - _current_chassis_state.heading);

	float chassis_vel_x = _control_params.chassis_position_gain * chassis_error_x;
	float chassis_vel_y = _control_params.chassis_position_gain * chassis_error_y;
	float chassis_yaw_rate = _control_params.chassis_position_gain * heading_error;

	// Limit velocities
	chassis_vel_x = math::constrain(chassis_vel_x, -_control_params.chassis_max_velocity,
					_control_params.chassis_max_velocity);
	chassis_yaw_rate = math::constrain(chassis_yaw_rate, -0.5f, 0.5f);

	// Publish chassis setpoint
	chassis_setpoint_s chassis_sp{};
	chassis_sp.timestamp = now;
	chassis_sp.velocity_x = chassis_vel_x;
	chassis_sp.velocity_y = chassis_vel_y;
	chassis_sp.yaw_rate = chassis_yaw_rate;
	chassis_sp.articulation_rate = 0.f;
	chassis_sp.position_valid = true;
	chassis_sp.velocity_valid = true;
	chassis_sp.articulation_valid = false;
	_chassis_setpoint_pub.publish(chassis_sp);

	// Publish boom setpoint
	boom_setpoint_s boom_sp{};
	boom_sp.timestamp = now;
	boom_sp.position = _target_boom_position;
	boom_sp.velocity = 0.f;  // Position control mode
	boom_sp.acceleration = 0.f;
	boom_sp.control_mode = boom_setpoint_s::MODE_POSITION;
	boom_sp.max_velocity = _control_params.max_bucket_vel_z;
	boom_sp.max_acceleration = 0.5f;
	boom_sp.setpoint_valid = true;
	boom_sp.trajectory_complete = false;
	_boom_setpoint_pub.publish(boom_sp);

	// Publish tilt setpoint
	tilt_setpoint_s tilt_sp{};
	tilt_sp.timestamp = now;
	tilt_sp.angle = _target_tilt_angle;
	tilt_sp.angular_velocity = 0.f;  // Position control mode
	tilt_sp.angular_acceleration = 0.f;
	tilt_sp.control_mode = tilt_setpoint_s::MODE_POSITION;
	tilt_sp.max_velocity = _control_params.max_bucket_rate_tilt;
	tilt_sp.max_acceleration = 1.0f;
	tilt_sp.setpoint_valid = true;
	tilt_sp.trajectory_complete = false;
	_tilt_setpoint_pub.publish(tilt_sp);
}

void WheelLoaderManualBucketMode::updateCurrentState()
{
	// Update subscriptions
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_attitude_sub.update(&_vehicle_attitude);

	// Update chassis state
	_current_chassis_state.x = _vehicle_local_position.x;
	_current_chassis_state.y = _vehicle_local_position.y;

	// Get heading from attitude
	matrix::Quatf q(_vehicle_attitude.q);
	matrix::Eulerf euler(q);
	_current_chassis_state.heading = euler.psi();

	// TODO: Get actual boom and tilt positions from sensors
	// For now, use last commanded values
	_current_boom_position = _target_boom_position;
	_current_tilt_angle = _target_tilt_angle;

	// TODO: Compute current bucket pose from forward kinematics
	// For now, keep it synchronized with target
	_current_bucket_pose = _target_bucket_pose;
}

void WheelLoaderManualBucketMode::loadVehicleParameters()
{
	// TODO: Load from YAML or PX4 parameters
	// Using defaults for now

	_vehicle_params.boom_length = 3.5f;
	_vehicle_params.boom_min = 0.5f;
	_vehicle_params.boom_max = 3.0f;
	_vehicle_params.tilt_min = -1.57f;
	_vehicle_params.tilt_max = 1.57f;
	_vehicle_params.bucket_x_min = -1.0f;
	_vehicle_params.bucket_x_max = 4.0f;
	_vehicle_params.bucket_z_min = 0.0f;
	_vehicle_params.bucket_z_max = 3.5f;
	_vehicle_params.chassis_to_boom_offset = 1.5f;

	_control_params.rc_deadband = 0.05f;
	_control_params.max_bucket_vel_x = 0.5f;
	_control_params.max_bucket_vel_z = 0.3f;
	_control_params.max_bucket_rate_heading = 0.3f;
	_control_params.max_bucket_rate_tilt = 0.5f;
	_control_params.chassis_position_gain = 0.5f;
	_control_params.chassis_max_velocity = 0.3f;
}

void WheelLoaderManualBucketMode::initializeBucketPose()
{
	// Initialize from current state (or default if sensors not available)
	// TODO: Use forward kinematics from current boom/tilt to compute bucket pose

	// Default initial pose: bucket in front of chassis
	_target_bucket_pose.x = 2.0f;  // 2m in front
	_target_bucket_pose.z = 1.0f;  // 1m height
	_target_bucket_pose.heading = 0.0f;  // Aligned with chassis
	_target_bucket_pose.tilt = 0.0f;  // Level

	_current_bucket_pose = _target_bucket_pose;

	PX4_INFO("Initialized bucket pose: X=%.2f Z=%.2f heading=%.2f tilt=%.2f",
		 (double)_target_bucket_pose.x, (double)_target_bucket_pose.z,
		 (double)_target_bucket_pose.heading, (double)_target_bucket_pose.tilt);
}
