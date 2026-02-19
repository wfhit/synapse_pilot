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

#include "traj_follower_mode.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

TrajFollowerMode::TrajFollowerMode(ModuleParams *parent) :
	OperationModeBase(parent, "WheelLoaderTrajFollower")
{
	// Load vehicle parameters from YAML
	loadVehicleParameters();

	// Configure MPC controller
	_chassis_mpc.setParameters(10, 5, UPDATE_DT);
	_chassis_mpc.setWeights(_mpc_weights);
	_chassis_mpc.setVehicleParams(_vehicle_params);

	// Configure S-curve planners
	_boom_planner.setLimits(_boom_limits);
	_boom_planner.setProfileType(ProfileType::QUINTIC);

	_tilt_planner.setLimits(_tilt_limits);
	_tilt_planner.setProfileType(ProfileType::QUINTIC);
}

bool TrajFollowerMode::activate()
{
	PX4_INFO("Activating Wheel Loader Trajectory Follower Mode");

	// Check if required data is available
	if (!_vehicle_local_position_sub.copy(&_vehicle_local_position)) {
		PX4_WARN("Vehicle local position not available");
		return false;
	}

	if (!_vehicle_attitude_sub.copy(&_vehicle_attitude)) {
		PX4_WARN("Vehicle attitude not available");
		return false;
	}

	// Update current state
	updateCurrentState();

	set_active(true);

	// Publish initial control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = true;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = true;
	config.flag_control_attitude_enabled = true;
	config.flag_control_rates_enabled = true;
	publish_control_config(config);

	return true;
}

void TrajFollowerMode::deactivate()
{
	PX4_INFO("Deactivating Wheel Loader Trajectory Follower Mode");

	_has_active_trajectory = false;
	_trajectory_complete = false;

	set_active(false);
}

void TrajFollowerMode::update(float dt)
{
	// Update subscriptions
	updateCurrentState();

	// Check for new trajectory
	if (_trajectory_sub.updated()) {
		processNewTrajectory();
	}

	// If no active trajectory, hold position
	if (!_has_active_trajectory) {
		// TODO: Implement position hold
		return;
	}

	// Check if trajectory is complete
	float elapsed_time = (hrt_absolute_time() - _trajectory_start_time) / 1e6f;
	float total_duration = (_trajectory_timestamps[_num_decoded_points - 1] - _trajectory_timestamps[0]) / 1e6f;

	if (elapsed_time >= total_duration) {
		_trajectory_complete = true;

		if (_trajectory.hold_last_point) {
			PX4_DEBUG("Trajectory complete, holding last point");
			// Continue publishing last setpoint

		} else {
			PX4_INFO("Trajectory complete, stopping");
			_has_active_trajectory = false;
			return;
		}
	}

	// Step 5: At 50Hz, sample from the pre-computed trajectories
	computeChassisControl();
	computeBoomControl();
	computeTiltControl();
	publishSetpoints();

	// Publish control config
	operation_control_config_s config{};
	config.flag_control_position_enabled = true;
	config.flag_control_velocity_enabled = true;
	config.flag_control_acceleration_enabled = true;
	config.flag_control_attitude_enabled = true;
	config.flag_control_rates_enabled = true;
	publish_control_config(config);
}

bool TrajFollowerMode::is_valid() const
{
	bool position_valid = _vehicle_local_position.xy_valid && _vehicle_local_position.z_valid &&
			      (hrt_elapsed_time(&_vehicle_local_position.timestamp) < 500000);

	bool attitude_valid = (hrt_elapsed_time(&_vehicle_attitude.timestamp) < 500000);

	return position_valid && attitude_valid;
}

void TrajFollowerMode::processNewTrajectory()
{
	// Save blending data before overwriting _trajectory
	if (_has_active_trajectory && _trajectory.smooth_transition) {
		_prev_num_points = math::min(_num_decoded_points, static_cast<uint8_t>(MAX_PREV_POINTS));
		memcpy(_prev_chassis_trajectory, _chassis_trajectory, sizeof(ChassisState) * _prev_num_points);
		memcpy(_prev_boom_trajectory, _boom_trajectory, sizeof(float) * _prev_num_points);
		memcpy(_prev_tilt_trajectory, _tilt_trajectory, sizeof(float) * _prev_num_points);
	}

	// Read directly into member to avoid ~1.8kB stack allocation
	if (!_trajectory_sub.copy(&_trajectory)) {
		return;
	}

	PX4_INFO("Received new trajectory: %d points, type %d", _trajectory.num_points, _trajectory.trajectory_type);

	// Validate trajectory
	if (_trajectory.num_points == 0 || _trajectory.num_points > MAX_TRAJ_POINTS) {
		PX4_WARN("Invalid trajectory point count: %d", _trajectory.num_points);
		return;
	}

	// Step 1: Decode entire trajectory into 3 separate trajectories
	decodeEntireTrajectory();

	// Step 2: Fuse with old trajectory if smooth transition requested
	if (_trajectory.smooth_transition && _prev_num_points > 0) {
		fuseTrajectories();
	}

	// Step 3: Update MPC with complete chassis trajectory
	// _chassis_mpc.setTrajectory(_chassis_trajectory, _trajectory_timestamps, _num_decoded_points);

	// Step 4: Update S-curve planners with complete boom/tilt trajectories
	// _boom_planner.setTrajectory(_boom_trajectory, _trajectory_timestamps, _num_decoded_points);
	// _tilt_planner.setTrajectory(_tilt_trajectory, _trajectory_timestamps, _num_decoded_points);

	// Reset tracking state
	_current_trajectory_point = 0;
	_trajectory_start_time = hrt_absolute_time();
	_point_start_time = _trajectory_start_time;
	_has_active_trajectory = true;
	_trajectory_complete = false;

	PX4_INFO("Trajectory decoded and loaded: %d points", _num_decoded_points);
}

bool TrajFollowerMode::decodeTrajectoryPoint(uint8_t point_index,
		ChassisState &chassis_target, float &boom_target, float &tilt_target)
{
	if (point_index >= _trajectory.num_points) {
		return false;
	}

	switch (_trajectory.trajectory_type) {
	case vla_trajectory_s::TRAJ_TYPE_BUCKET_6DOF: {
			// Type 0: Bucket 6DOF pose
			// Need to compute inverse kinematics to get chassis/boom/tilt from bucket pose
			// This is a simplified placeholder

			float bucket_x = _trajectory.bucket_x[point_index];
			float bucket_y = _trajectory.bucket_y[point_index];
			float bucket_z = _trajectory.bucket_z[point_index];

			// Transform to local frame
			transformToLocalFrame(bucket_x, bucket_y, chassis_target.heading, _trajectory.frame_id);

			// Simplified IK: assume bucket is directly above chassis
			chassis_target.x = bucket_x;
			chassis_target.y = bucket_y;
			boom_target = bucket_z;  // Simplified: use z as boom height

			// Extract tilt from quaternion (simplified)
			matrix::Quatf q(_trajectory.bucket_qw[point_index], _trajectory.bucket_qx[point_index],
					_trajectory.bucket_qy[point_index], _trajectory.bucket_qz[point_index]);
			matrix::Eulerf euler(q);
			tilt_target = euler.phi();  // Roll as tilt

			chassis_target.articulation = 0.f;
			break;
		}

	case vla_trajectory_s::TRAJ_TYPE_CHASSIS_BUCKET_FULL: {
			// Type 1: Chassis position + bucket heading/altitude/tilt + articulation
			chassis_target.x = _trajectory.chassis_x[point_index];
			chassis_target.y = _trajectory.chassis_y[point_index];
			chassis_target.heading = 0.f;  // Not specified in this type
			chassis_target.articulation = _trajectory.articulation_angle[point_index];

			transformToLocalFrame(chassis_target.x, chassis_target.y, chassis_target.heading, _trajectory.frame_id);

			boom_target = _trajectory.bucket_altitude[point_index];
			tilt_target = _trajectory.bucket_tilt[point_index];
			break;
		}

	case vla_trajectory_s::TRAJ_TYPE_CHASSIS_BUCKET_POSE: {
			// Type 2: Chassis pose + bucket altitude/tilt
			chassis_target.x = _trajectory.chassis_x[point_index];
			chassis_target.y = _trajectory.chassis_y[point_index];
			chassis_target.heading = _trajectory.chassis_heading[point_index];
			chassis_target.articulation = 0.f;  // Not used in this type

			transformToLocalFrame(chassis_target.x, chassis_target.y, chassis_target.heading, _trajectory.frame_id);

			boom_target = _trajectory.bucket_altitude[point_index];
			tilt_target = _trajectory.bucket_tilt[point_index];
			break;
		}

	default:
		PX4_WARN("Unknown trajectory type: %d", _trajectory.trajectory_type);
		return false;
	}

	return true;
}

void TrajFollowerMode::decodeEntireTrajectory()
{
	_num_decoded_points = _trajectory.num_points;

	for (uint8_t i = 0; i < _num_decoded_points; i++) {
		ChassisState chassis_target;
		float boom_target, tilt_target;

		if (decodeTrajectoryPoint(i, chassis_target, boom_target, tilt_target)) {
			_chassis_trajectory[i] = chassis_target;
			_boom_trajectory[i] = boom_target;
			_tilt_trajectory[i] = tilt_target;
			_trajectory_timestamps[i] = _trajectory.point_timestamps[i];

		} else {
			PX4_WARN("Failed to decode trajectory point %d", i);
			_num_decoded_points = i;
			break;
		}
	}

	PX4_DEBUG("Decoded %d trajectory points", _num_decoded_points);
}

void TrajFollowerMode::fuseTrajectories()
{
	if (_prev_num_points == 0) {
		return;
	}

	// Blend from current state to first point over blend_duration
	float blend_factor = 0.5f; // TODO: Load from parameters

	// Only blend the first point
	if (_num_decoded_points > 0) {
		_chassis_trajectory[0].x = _current_chassis_state.x * (1.0f - blend_factor) +
					   _chassis_trajectory[0].x * blend_factor;
		_chassis_trajectory[0].y = _current_chassis_state.y * (1.0f - blend_factor) +
					   _chassis_trajectory[0].y * blend_factor;
		_chassis_trajectory[0].heading = _current_chassis_state.heading * (1.0f - blend_factor) +
						 _chassis_trajectory[0].heading * blend_factor;
		_chassis_trajectory[0].velocity = _current_chassis_state.velocity * (1.0f - blend_factor) +
						  _chassis_trajectory[0].velocity * blend_factor;

		_boom_trajectory[0] = _current_boom_state.position * (1.0f - blend_factor) +
				      _boom_trajectory[0] * blend_factor;

		_tilt_trajectory[0] = _current_tilt_state.position * (1.0f - blend_factor) +
				      _tilt_trajectory[0] * blend_factor;

		PX4_DEBUG("Fused trajectory with current state (blend_factor: %.2f)", (double)blend_factor);
	}
}

void TrajFollowerMode::transformToLocalFrame(float &x, float &y, float &heading, uint8_t frame_id)
{
	if (frame_id == vla_trajectory_s::FRAME_LOCAL) {
		// Already in local frame
		return;

	} else if (frame_id == vla_trajectory_s::FRAME_BODY) {
		// Transform from body to local frame
		float cos_h = cosf(_current_chassis_state.heading);
		float sin_h = sinf(_current_chassis_state.heading);

		float x_local = _current_chassis_state.x + x * cos_h - y * sin_h;
		float y_local = _current_chassis_state.y + x * sin_h + y * cos_h;
		float heading_local = _current_chassis_state.heading + heading;

		x = x_local;
		y = y_local;
		heading = heading_local;

	} else if (frame_id == vla_trajectory_s::FRAME_GLOBAL) {
		// For now, treat global same as local
		// In production, would need proper global->local transform
		PX4_WARN("Global frame not fully implemented, treating as local");
	}
}

void TrajFollowerMode::updateCurrentState()
{
	// Update vehicle position
	_vehicle_local_position_sub.update(&_vehicle_local_position);
	_vehicle_attitude_sub.update(&_vehicle_attitude);

	// Update chassis state
	_current_chassis_state.x = _vehicle_local_position.x;
	_current_chassis_state.y = _vehicle_local_position.y;
	_current_chassis_state.velocity = sqrtf(_vehicle_local_position.vx * _vehicle_local_position.vx +
						_vehicle_local_position.vy * _vehicle_local_position.vy);

	// Get heading from attitude
	matrix::Quatf q(_vehicle_attitude.q);
	matrix::Eulerf euler(q);
	_current_chassis_state.heading = euler.psi();
	// _current_chassis_state.yaw_rate = _vehicle_attitude.deltaphis[2]; // deltaphis doesn't exist
	_current_chassis_state.yaw_rate = 0.f; // TODO: Use vehicle_angular_velocity

	// TODO: Get articulation angle from actual sensor
	// For now, use last commanded value or zero
	_current_chassis_state.articulation = 0.f;

	// TODO: Get boom and tilt positions from actual sensors
	// Placeholders:
	_current_boom_state.position = 0.f;
	_current_boom_state.velocity = 0.f;
	_current_boom_state.acceleration = 0.f;

	_current_tilt_state.position = 0.f;
	_current_tilt_state.velocity = 0.f;
	_current_tilt_state.acceleration = 0.f;
}

void TrajFollowerMode::computeChassisControl()
{
	// Get current absolute time since trajectory start
	// uint64_t current_time = hrt_absolute_time();
	// float elapsed_time = (current_time - _trajectory_start_time) / 1e6f;

	// MPC updates its control at 50Hz based on current state and pre-loaded trajectory
	// _chassis_mpc.computeControl(_current_chassis_state, current_time, _chassis_control);
	// TODO: Implement chassis MPC control computation
}

void TrajFollowerMode::computeBoomControl()
{
	// Get current absolute time for trajectory sampling
	uint64_t current_time = hrt_absolute_time();

	// S-curve planner samples from pre-computed trajectory
	_boom_planner.getTrajectoryPoint(current_time, _boom_setpoint);
}

void TrajFollowerMode::computeTiltControl()
{
	// Get current absolute time for trajectory sampling
	uint64_t current_time = hrt_absolute_time();

	// S-curve planner samples from pre-computed trajectory
	_tilt_planner.getTrajectoryPoint(current_time, _tilt_setpoint);
}

void TrajFollowerMode::synchronizeControllers()
{
	// Get durations for each controller
	_chassis_time_to_target = 1.0f;  // TODO: Estimate from MPC
	_boom_time_to_target = _boom_planner.getDuration();
	_tilt_time_to_target = _tilt_planner.getDuration();

	// Use maximum time for synchronization
	_synchronized_time = fmaxf(_chassis_time_to_target, fmaxf(_boom_time_to_target, _tilt_time_to_target));

	// Re-plan slower trajectories to match synchronized time
	if (_boom_time_to_target < _synchronized_time) {
		ChassisState dummy;
		float boom_target, tilt_target;
		decodeTrajectoryPoint(_current_trajectory_point, dummy, boom_target, tilt_target);
		_boom_planner.planTrajectory(_current_boom_state, boom_target, _synchronized_time);
	}

	if (_tilt_time_to_target < _synchronized_time) {
		ChassisState dummy;
		float boom_target, tilt_target;
		decodeTrajectoryPoint(_current_trajectory_point, dummy, boom_target, tilt_target);
		_tilt_planner.planTrajectory(_current_tilt_state, tilt_target, _synchronized_time);
	}

	PX4_DEBUG("Synchronized time: %.2f s (chassis: %.2f, boom: %.2f, tilt: %.2f)",
		  (double)_synchronized_time, (double)_chassis_time_to_target,
		  (double)_boom_time_to_target, (double)_tilt_time_to_target);
}

void TrajFollowerMode::publishSetpoints()
{
	hrt_abstime now = hrt_absolute_time();

	// Publish chassis setpoint
	chassis_setpoint_s chassis_sp{};
	chassis_sp.timestamp = now;
	chassis_sp.velocity_x = _chassis_control.velocity;
	chassis_sp.velocity_y = 0.f;
	chassis_sp.yaw_rate = _chassis_control.yaw_rate;
	chassis_sp.articulation_rate = _chassis_control.articulation_rate;
	chassis_sp.position_valid = true;
	chassis_sp.velocity_valid = true;
	chassis_sp.articulation_valid = true;
	_chassis_setpoint_pub.publish(chassis_sp);

	// Publish boom setpoint
	boom_setpoint_s boom_sp{};
	boom_sp.timestamp = now;
	boom_sp.position = _boom_setpoint.position;
	boom_sp.velocity = _boom_setpoint.velocity;
	boom_sp.acceleration = _boom_setpoint.acceleration;
	boom_sp.control_mode = boom_setpoint_s::MODE_POSITION;
	boom_sp.max_velocity = _boom_limits.max_velocity;
	boom_sp.max_acceleration = _boom_limits.max_acceleration;
	boom_sp.setpoint_valid = true;
	boom_sp.trajectory_complete = _boom_planner.isComplete((hrt_absolute_time() - _point_start_time) / 1e6f);
	_boom_setpoint_pub.publish(boom_sp);

	// Publish tilt setpoint
	tilt_setpoint_s tilt_sp{};
	tilt_sp.timestamp = now;
	tilt_sp.angle = _tilt_setpoint.position;
	tilt_sp.angular_velocity = _tilt_setpoint.velocity;
	tilt_sp.angular_acceleration = _tilt_setpoint.acceleration;
	tilt_sp.control_mode = tilt_setpoint_s::MODE_POSITION;
	tilt_sp.max_velocity = _tilt_limits.max_velocity;
	tilt_sp.max_acceleration = _tilt_limits.max_acceleration;
	tilt_sp.setpoint_valid = true;
	tilt_sp.trajectory_complete = _tilt_planner.isComplete((hrt_absolute_time() - _point_start_time) / 1e6f);
	_tilt_setpoint_pub.publish(tilt_sp);
}

bool TrajFollowerMode::shouldAdvanceToNextPoint()
{
	if (_current_trajectory_point >= _trajectory.num_points) {
		return false;
	}

	// Check if we've reached the target time for current point
	uint64_t target_timestamp = _trajectory.point_timestamps[_current_trajectory_point];
	uint64_t current_time = hrt_absolute_time();

	// If using absolute timestamps
	if (target_timestamp > _trajectory.timestamp) {
		return current_time >= target_timestamp;
	}

	// Otherwise use synchronized time
	float elapsed = (current_time - _point_start_time) / 1e6f;
	return elapsed >= _synchronized_time;
}

void TrajFollowerMode::loadVehicleParameters()
{
	// TODO: Load from YAML file (vehicle_params.yaml)
	// For now, use defaults from VehicleParams struct

	// MPC weights (load from YAML in production)
	_mpc_weights.position_x = 10.0f;
	_mpc_weights.position_y = 10.0f;
	_mpc_weights.heading = 5.0f;
	_mpc_weights.articulation = 3.0f;
	_mpc_weights.velocity = 1.0f;
	_mpc_weights.yaw_rate = 1.0f;
	_mpc_weights.articulation_rate = 1.5f;
	_mpc_weights.velocity_change = 0.5f;
	_mpc_weights.yaw_rate_change = 0.5f;
	_mpc_weights.terminal_position = 20.0f;
	_mpc_weights.terminal_heading = 10.0f;
	_mpc_weights.terminal_articulation = 5.0f;

	// Vehicle parameters
	_vehicle_params.front_length = 2.5f;
	_vehicle_params.rear_length = 2.0f;
	_vehicle_params.max_velocity = 2.0f;
	_vehicle_params.max_yaw_rate = 0.5f;
	_vehicle_params.max_articulation = 0.785f;
	_vehicle_params.max_articulation_rate = 0.3f;

	// Boom limits
	_boom_limits.max_velocity = 0.5f;
	_boom_limits.max_acceleration = 0.3f;
	_boom_limits.max_jerk = 1.0f;

	// Tilt limits
	_tilt_limits.max_velocity = 0.8f;
	_tilt_limits.max_acceleration = 0.5f;
	_tilt_limits.max_jerk = 2.0f;
}

void TrajFollowerMode::setupInitialBlending()
{
	_is_blending = true;
	_blend_start_time = hrt_absolute_time();
	_blend_duration = 1.0f;  // TODO: Load from parameters

	PX4_INFO("Setting up smooth blend transition, duration: %.2f s", (double)_blend_duration);
}
