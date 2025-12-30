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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/wheel_loader/command_status.h>
#include <uORB/topics/wheel_loader/arm_cmd.h>
#include <uORB/topics/wheel_loader/arm_status.h>
#include <uORB/topics/wheel_loader/operation_mode_cmd.h>
#include <uORB/topics/wheel_loader/safety_ack_cmd.h>
#include <uORB/topics/wheel_loader/safety_status.h>
#include <uORB/topics/wheel_loader/safety_input.h>
#include <uORB/topics/wheel_loader/health_monitor_status.h>

/**
 * @brief Operator Interface
 *
 * Receives commands from MAVLink and RC, processes and forwards them to appropriate modules.
 * - Processes MAVLink COMMAND_LONG for arm/disarm, mode changes, task execution
 * - Processes RC switches for arm, mode, emergency stop, deadman
 * - Validates commands and enforces safety requirements
 * - Handles command source priority (RC safety overrides)
 * - Monitors MAVLink and RC heartbeat/connection
 */
class OperatorInterface : public ModuleBase<OperatorInterface>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	OperatorInterface();
	~OperatorInterface() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

private:
	void Run() override;

	/**
	 * Process MAVLink commands
	 */
	void process_mavlink_commands();

	/**
	 * Process RC inputs
	 */
	void process_rc_inputs();

	/**
	 * Send arm command
	 */
	void send_arm_command(bool arm, uint8_t source);

	/**
	 * Send operation mode command
	 */
	void send_mode_command(uint8_t mode, uint8_t source);

	/**
	 * Send task executor command
	 */
	void send_task_command(uint8_t task_id, uint8_t action, uint8_t source);

	/**
	 * Send safety acknowledge command
	 */
	void send_safety_ack_command();

	/**
	 * Send emergency stop via safety input
	 */
	void send_emergency_stop(bool stop);

	/**
	 * Update deadman switch status in safety input
	 */
	void update_deadman_status(bool active);

	/**
	 * Send MAVLink command acknowledgment
	 */
	void send_command_ack(uint16_t command, uint8_t result);

	/**
	 * Validate if command is allowed in current state
	 */
	bool validate_command(uint8_t command_type);

	/**
	 * Map RC mode switch to operation mode
	 */
	uint8_t map_rc_mode(uint8_t rc_value);

	/**
	 * Map RC task trigger to task ID
	 */
	uint8_t map_rc_task(uint8_t rc_value);

	static constexpr uint32_t SCHEDULE_INTERVAL_US = 50_ms; // 20 Hz

	// Subscriptions
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _arm_status_sub{ORB_ID(arm_status)};
	uORB::Subscription _safety_status_sub{ORB_ID(safety_status)};
	uORB::Subscription _health_status_sub{ORB_ID(health_monitor_status)};

	// Publications
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};
	uORB::Publication<command_status_s> _command_status_pub{ORB_ID(command_status)};
	uORB::Publication<arm_cmd_s> _arm_cmd_pub{ORB_ID(arm_cmd)};
	uORB::Publication<operation_mode_cmd_s> _operation_mode_cmd_pub{ORB_ID(operation_mode_cmd)};
	// TODO: Removed task_executor - redesign needed
	// uORB::Publication<task_executor_cmd_s> _task_executor_cmd_pub{ORB_ID(task_executor_cmd)};
	uORB::Publication<safety_ack_cmd_s> _safety_ack_cmd_pub{ORB_ID(safety_ack_cmd)};
	uORB::Publication<safety_input_s> _safety_input_pub{ORB_ID(safety_input)};

	// Status
	command_status_s _command_status{};
	hrt_abstime _last_run_timestamp{0};

	// MAVLink tracking
	uint32_t _mavlink_cmd_count{0};
	uint32_t _mavlink_cmd_accepted{0};
	uint32_t _mavlink_cmd_rejected{0};

	// RC tracking
	bool _prev_rc_arm_switch{false};
	uint8_t _prev_rc_mode_switch{0};
	uint8_t _prev_rc_task_trigger{0};
	bool _prev_rc_emergency_stop{false};
	bool _prev_rc_deadman{false};
	uint32_t _rc_cmd_count{0};
	uint32_t _rc_mode_changes{0};
	uint32_t _rc_task_triggers{0};

	// State tracking
	bool _armed{false};
	bool _safety_faults_present{false};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::CMD_REQUIRE_DEADMAN>) _param_require_deadman,
		(ParamBool<px4::params::CMD_RC_PRIORITY>) _param_rc_priority,
		(ParamInt<px4::params::CMD_ARM_RC_CHAN>) _param_arm_rc_chan,
		(ParamInt<px4::params::CMD_MODE_RC_CHAN>) _param_mode_rc_chan,
		(ParamInt<px4::params::CMD_ESTOP_RC_CHAN>) _param_estop_rc_chan,
		(ParamInt<px4::params::CMD_DEADMAN_RC_CHAN>) _param_deadman_rc_chan,
		(ParamInt<px4::params::CMD_TASK_RC_CHAN>) _param_task_rc_chan,
		(ParamInt<px4::params::CMD_RC_THRESHOLD>) _param_rc_threshold
	)

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
};
