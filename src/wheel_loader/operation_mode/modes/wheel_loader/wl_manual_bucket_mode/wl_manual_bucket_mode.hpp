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
 * @file wheel_loader_manual_bucket_mode.hpp
 *
 * Wheel Loader Manual Bucket Control Mode
 * - RC channels directly control bucket velocity (forward/back, height, heading, tilt)
 * - Chassis automatically positions via inverse kinematics
 * - Runs at 50Hz
 *
 * @author PX4 Development Team
 */

#pragma once

#include "../../../operation_mode_base.hpp"

#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/chassis_setpoint.h>
#include <uORB/topics/boom_setpoint.h>
#include <uORB/topics/tilt_setpoint.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <matrix/matrix/math.hpp>
#include <mathlib/mathlib.h>

class WheelLoaderManualBucketMode : public OperationModeBase
{
public:
	WheelLoaderManualBucketMode(ModuleParams *parent);
	~WheelLoaderManualBucketMode() override = default;

	bool activate() override;
	void deactivate() override;
	void update(float dt) override;
	bool is_valid() const override;

private:
	/**
	 * Bucket pose in body frame
	 */
	struct BucketPose {
		float x;           // Forward/backward position relative to chassis [m]
		float z;           // Height above ground [m]
		float heading;     // Heading angle [rad]
		float tilt;        // Tilt angle [rad]
	};

	/**
	 * Chassis state
	 */
	struct ChassisState {
		float x;           // Position X [m]
		float y;           // Position Y [m]
		float heading;     // Heading [rad]
	};

	/**
	 * Read RC inputs and convert to bucket velocities
	 */
	void processRCInputs(float dt);

	/**
	 * Apply deadband to RC input
	 */
	float applyDeadband(float input, float deadband);

	/**
	 * Update target bucket pose by integrating velocities
	 */
	void updateBucketPose(float dt);

	/**
	 * Apply position and angle limits to bucket pose
	 */
	void applyBucketLimits();

	/**
	 * Compute inverse kinematics: bucket pose -> chassis + boom + tilt
	 */
	bool computeInverseKinematics();

	/**
	 * Publish control setpoints
	 */
	void publishSetpoints();

	/**
	 * Update current sensor state
	 */
	void updateCurrentState();

	/**
	 * Load vehicle parameters
	 */
	void loadVehicleParameters();

	/**
	 * Initialize bucket pose from current state
	 */
	void initializeBucketPose();

	// Subscriptions
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};

	// Publications
	uORB::Publication<chassis_setpoint_s> _chassis_setpoint_pub{ORB_ID(chassis_setpoint)};
	uORB::Publication<boom_setpoint_s> _boom_setpoint_pub{ORB_ID(boom_setpoint)};
	uORB::Publication<tilt_setpoint_s> _tilt_setpoint_pub{ORB_ID(tilt_setpoint)};

	// RC input data
	manual_control_setpoint_s _manual_control_setpoint{};
	vehicle_local_position_s _vehicle_local_position{};
	vehicle_attitude_s _vehicle_attitude{};

	// Bucket control state
	BucketPose _target_bucket_pose{};  // Target bucket pose from RC integration
	BucketPose _current_bucket_pose{}; // Current bucket pose (from sensors)

	// Bucket velocities from RC
	float _bucket_vel_x{0.f};      // Forward/backward velocity [m/s]
	float _bucket_vel_z{0.f};      // Height velocity [m/s]
	float _bucket_rate_heading{0.f}; // Heading rate [rad/s]
	float _bucket_rate_tilt{0.f};  // Tilt rate [rad/s]

	// IK outputs
	ChassisState _target_chassis_state{};
	float _target_boom_position{0.f};
	float _target_tilt_angle{0.f};

	// Current chassis state
	ChassisState _current_chassis_state{};
	float _current_boom_position{0.f};
	float _current_tilt_angle{0.f};

	// Vehicle parameters
	struct VehicleParams {
		float boom_length{3.5f};           // Boom length [m]
		float boom_min{0.0f};              // Minimum boom position [m]
		float boom_max{3.0f};              // Maximum boom position [m]
		float tilt_min{-1.57f};            // Minimum tilt angle [rad]
		float tilt_max{1.57f};             // Maximum tilt angle [rad]
		float bucket_x_min{-2.0f};         // Min bucket X relative to chassis [m]
		float bucket_x_max{5.0f};          // Max bucket X relative to chassis [m]
		float bucket_z_min{0.0f};          // Min bucket height [m]
		float bucket_z_max{4.0f};          // Max bucket height [m]
		float chassis_to_boom_offset{1.5f}; // Distance from chassis center to boom base [m]
	} _vehicle_params;

	// Control parameters
	struct ControlParams {
		float rc_deadband{0.05f};          // RC deadband (0-1)
		float max_bucket_vel_x{0.5f};      // Max bucket forward/back velocity [m/s]
		float max_bucket_vel_z{0.3f};      // Max bucket height velocity [m/s]
		float max_bucket_rate_heading{0.3f}; // Max bucket heading rate [rad/s]
		float max_bucket_rate_tilt{0.5f}; // Max bucket tilt rate [rad/s]
		float chassis_position_gain{1.0f}; // Chassis position controller gain
		float chassis_max_velocity{0.5f};  // Max chassis velocity [m/s]
	} _control_params;

	// Update rate
	static constexpr float UPDATE_RATE = 50.0f;  // 50 Hz
	static constexpr float UPDATE_DT = 1.0f / UPDATE_RATE;
};
