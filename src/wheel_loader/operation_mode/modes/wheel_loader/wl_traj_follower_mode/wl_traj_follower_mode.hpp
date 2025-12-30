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

/**
 * @file wheel_loader_traj_follower_mode.hpp
 *
 * Wheel Loader Trajectory Follower Mode
 * - Subscribes to WheelLoaderTrajectory messages
 * - Uses MPC for chassis control
 * - Uses S-curve planner for boom and tilt control
 * - Time-synchronized execution
 *
 * @author Your Name
 */

#pragma once

#include "../../../operation_mode_base.hpp"
#include "chassis_mpc_controller.hpp"
#include "s_curve_planner.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/wheel_loader/trajectory.h>
#include <uORB/topics/wheel_loader/chassis_setpoint.h>
#include <uORB/topics/wheel_loader/boom_setpoint.h>
#include <uORB/topics/wheel_loader/tilt_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/matrix/math.hpp>

class WheelLoaderTrajFollowerMode : public OperationModeBase
{
public:
	WheelLoaderTrajFollowerMode(ModuleParams *parent);
	~WheelLoaderTrajFollowerMode() override = default;

	bool activate() override;
	void deactivate() override;
	void update(float dt) override;
	bool is_valid() const override;

private:
	/**
	 * Process new trajectory message
	 */
	void processNewTrajectory();

	/**
	 * Decode entire trajectory into separate chassis, boom, and tilt trajectories
	 */
	void decodeEntireTrajectory();

	/**
	 * Decode trajectory point based on type
	 */
	bool decodeTrajectoryPoint(uint8_t point_index, ChassisState &chassis_target,
				   float &boom_target, float &tilt_target);

	/**
	 * Fuse new trajectory with old trajectory for smooth blending
	 */
	void fuseTrajectories();

	/**
	 * Transform coordinates between frames
	 */
	void transformToLocalFrame(float &x, float &y, float &heading, uint8_t frame_id);

	/**
	 * Update current state from sensors
	 */
	void updateCurrentState();

	/**
	 * Compute chassis control using MPC
	 */
	void computeChassisControl();

	/**
	 * Compute boom control using S-curve planner
	 */
	void computeBoomControl();

	/**
	 * Compute tilt control using S-curve planner
	 */
	void computeTiltControl();

	/**
	 * Synchronize timing across all controllers
	 */
	void synchronizeControllers();

	/**
	 * Publish control setpoints
	 */
	void publishSetpoints();

	/**
	 * Check if we should advance to next trajectory point
	 */
	bool shouldAdvanceToNextPoint();

	/**
	 * Load vehicle parameters from YAML
	 */
	void loadVehicleParameters();

	/**
	 * Blend from current state to first trajectory point
	 */
	void setupInitialBlending();

	// Subscriptions
	uORB::Subscription _wheel_loader_trajectory_sub{ORB_ID(wheel_loader_trajectory)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	// Publications
	uORB::Publication<wheel_loader_chassis_setpoint_s> _chassis_setpoint_pub{ORB_ID(wheel_loader_chassis_setpoint)};
	uORB::Publication<wheel_loader_boom_setpoint_s> _boom_setpoint_pub{ORB_ID(wheel_loader_boom_setpoint)};
	uORB::Publication<wheel_loader_tilt_setpoint_s> _tilt_setpoint_pub{ORB_ID(wheel_loader_tilt_setpoint)};

	// Controllers
	ChassisMPCController _chassis_mpc;
	SCurvePlanner _boom_planner;
	SCurvePlanner _tilt_planner;

	// Current state
	ChassisState _current_chassis_state;
	TrajectoryPoint _current_boom_state;
	TrajectoryPoint _current_tilt_state;

	vehicle_local_position_s _vehicle_local_position{};
	vehicle_attitude_s _vehicle_attitude{};
	wheel_loader_trajectory_s _trajectory{};

	// Decoded trajectories
	static constexpr uint8_t MAX_TRAJ_POINTS = 16;
	ChassisState _chassis_trajectory[MAX_TRAJ_POINTS];
	float _boom_trajectory[MAX_TRAJ_POINTS];
	float _tilt_trajectory[MAX_TRAJ_POINTS];
	uint64_t _trajectory_timestamps[MAX_TRAJ_POINTS];
	uint8_t _num_decoded_points{0};

	// Previous trajectory for blending
	ChassisState _prev_chassis_trajectory[MAX_TRAJ_POINTS];
	float _prev_boom_trajectory[MAX_TRAJ_POINTS];
	float _prev_tilt_trajectory[MAX_TRAJ_POINTS];
	uint8_t _prev_num_points{0};

	// Trajectory tracking
	uint8_t _current_trajectory_point{0};
	hrt_abstime _trajectory_start_time{0};
	hrt_abstime _point_start_time{0};
	bool _has_active_trajectory{false};
	bool _trajectory_complete{false};

	// Control outputs
	ChassisControl _chassis_control;
	TrajectoryPoint _boom_setpoint;
	TrajectoryPoint _tilt_setpoint;

	// Timing and synchronization
	float _chassis_time_to_target{0.f};
	float _boom_time_to_target{0.f};
	float _tilt_time_to_target{0.f};
	float _synchronized_time{0.f};

	// Blending state
	bool _is_blending{false};
	hrt_abstime _blend_start_time{0};
	float _blend_duration{1.0f};

	// Vehicle parameters
	VehicleParams _vehicle_params;
	MotionLimits _boom_limits;
	MotionLimits _tilt_limits;

	// MPC weights
	MPCWeights _mpc_weights;

	// Update rate
	static constexpr float UPDATE_RATE = 50.0f;  // 50 Hz
	static constexpr float UPDATE_DT = 1.0f / UPDATE_RATE;
};
