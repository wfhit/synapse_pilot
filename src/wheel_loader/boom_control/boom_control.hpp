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

// System includes first
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// Library includes
#include <perf/perf_counter.h>

// uORB includes (use lowercase topic names)
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/boom_control_setpoint.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/parameter_update.h>

// Using declarations
using namespace time_literals;

// Component includes (required for composition)
#include "boom_kinematics.hpp"
#include "boom_hardware_interface.hpp"
#include "boom_motion_controller.hpp"
#include "boom_state_manager.hpp"

/**
 * @brief Main boom control module for wheel loader lifting system
 *
 * Coordinates boom actuator components through a well-defined architecture:
 * - Hardware interface management (BoomHardwareInterface)
 * - Kinematic calculations (BoomKinematics)
 * - Motion planning and control (BoomMotionController)
 * - State management and safety (BoomStateManager)
 *
 * The module uses composition over inheritance for better maintainability
 * and follows RAII principles for resource management. All components are
 * constructed as member objects to ensure proper lifetime management.
 *
 * @note This implementation follows PX4 coding standards including:
 * - Tab-based indentation (4-space display width)
 * - Composition over inheritance architecture
 * - Proper uORB message handling
 * - Parameter validation and bounds checking
 */
class BoomControl final : public ModuleBase<BoomControl>,
	public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	static constexpr uint32_t CONTROL_INTERVAL_US = 20000; // 50 Hz

	BoomControl();
	~BoomControl();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * @brief Main control pipeline stages
	 *
	 * The control loop executes these stages sequentially at 50Hz:
	 * 1. update_sensors() - Read encoder and H-bridge status via hardware interface
	 * 2. process_commands() - Handle incoming boom commands via uORB
	 * 3. update_motion_planning() - Generate smooth trajectories
	 * 4. execute_control() - Compute motor commands using PID control
	 * 5. publish_telemetry() - Send status updates via uORB
	 */
	void update_sensors();
	void process_commands();
	void update_motion_planning();
	void execute_control();
	void publish_telemetry();

	/**
	 * @brief System management and safety functions
	 *
	 * These functions handle parameter updates, emergency conditions,
	 * and overall system health monitoring for safe boom operation.
	 */
	void update_parameters();
	void handle_emergency_stop();
	bool check_system_health();

	// Core components (using composition for better maintainability and testing)
	BoomKinematics _kinematics;                    // Forward/inverse kinematics calculations
	BoomHardwareInterface _hardware_interface;     // Unified sensor and actuator interface
	BoomMotionController _motion_controller;       // Trajectory generation and PID control
	BoomStateManager _state_manager;               // State machine and safety management

	// Current system state (updated each control cycle)
	float _current_boom_angle{0.0f};               // Current boom angle (rad)
	float _current_actuator_length{0.0f};          // Current hydraulic cylinder length (mm)
	float _target_boom_angle{0.0f};                // Commanded target boom angle (rad)
	hrt_abstime _last_command_time{0};             // Timestamp of last received command

	// uORB interface (using modern uORB::Publication/Subscription classes)
		// Subscriptions
	uORB::Subscription _boom_control_setpoint_sub{ORB_ID(boom_control_setpoint)}; // Control setpoint input
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};   // Parameter updates
	uORB::Publication<boom_status_s> _boom_status_pub{ORB_ID(boom_status)}; // Outgoing status telemetry

	// Performance monitoring
	perf_counter_t _cycle_perf;
	perf_counter_t _control_latency_perf;

	// Module parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BOOM_EN>) _param_enabled,
		(ParamFloat<px4::params::BOOM_RATE>) _param_update_rate,
		// IK parameters
		(ParamFloat<px4::params::BOOM_LEN>) _param_boom_length,
		(ParamFloat<px4::params::BOOM_PIVOT_H>) _param_boom_pivot_height,
		(ParamFloat<px4::params::BOOM_ANG_MIN>) _param_boom_angle_min,
		(ParamFloat<px4::params::BOOM_ANG_MAX>) _param_boom_angle_max
	)

	/**
	 * @brief Compute boom angle from bucket height using inverse kinematics
	 * @param bucket_height Target bucket height from ground [m]
	 * @return Boom angle [rad], clamped to joint limits
	 */
	float computeBoomAngleFromHeight(float bucket_height);
};
