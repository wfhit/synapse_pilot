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

#include "operator_interface.hpp"

#include <mathlib/mathlib.h>

OperatorInterface::OperatorInterface() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

OperatorInterface::~OperatorInterface()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool OperatorInterface::init()
{
	ScheduleOnInterval(SCHEDULE_INTERVAL_US);
	return true;
}

void OperatorInterface::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	const hrt_abstime now = hrt_absolute_time();
	_last_run_timestamp = now;

	// Update arming state
	arm_status_s arm_status;

	if (_arm_status_sub.copy(&arm_status)) {
		_armed = arm_status.armed;
	}

	// Update safety faults state
	safety_status_s safety_status;

	if (_safety_status_sub.copy(&safety_status)) {
		_safety_faults_present = !safety_status.safety_ok;
	}

	// Get connection status from health monitor
	health_monitor_status_s health_status;

	if (_health_status_sub.copy(&health_status)) {
		_command_status.mavlink_connected = health_status.mavlink_connected;
		_command_status.rc_connected = health_status.rc_connected;
		_command_status.mavlink_last_heartbeat = health_status.mavlink_last_heartbeat;
		_command_status.rc_last_update = health_status.rc_last_update;
	}

	// Process MAVLink commands
	process_mavlink_commands();

	// Process RC inputs
	process_rc_inputs();

	// Update command status
	_command_status.timestamp = now;
	_command_status.mavlink_commands_received = _mavlink_cmd_count;
	_command_status.mavlink_commands_accepted = _mavlink_cmd_accepted;
	_command_status.mavlink_commands_rejected = _mavlink_cmd_rejected;
	_command_status.rc_commands_sent = _rc_cmd_count;
	_command_status.rc_mode_changes = _rc_mode_changes;
	_command_status.rc_task_triggers = _rc_task_triggers;

	// Determine command authority
	if (_command_status.mavlink_connected && _command_status.rc_connected) {
		_command_status.command_authority = command_status_s::AUTHORITY_BOTH;

	} else if (_command_status.mavlink_connected) {
		_command_status.command_authority = command_status_s::AUTHORITY_MAVLINK;

	} else if (_command_status.rc_connected) {
		_command_status.command_authority = command_status_s::AUTHORITY_RC;

	} else {
		_command_status.command_authority = command_status_s::AUTHORITY_NONE;
	}

	// Check if commands are allowed
	_command_status.command_allowed = true;

	if (_param_require_deadman.get() && !_command_status.deadman_active) {
		_command_status.command_allowed = false;
	}

	if (_command_status.emergency_stop_active) {
		_command_status.command_allowed = false;
	}

	_command_status_pub.publish(_command_status);

	perf_end(_loop_perf);
}

void OperatorInterface::process_mavlink_commands()
{
	vehicle_command_s cmd;

	while (_vehicle_command_sub.update(&cmd)) {
		_mavlink_cmd_count++;

		bool handled = false;
		uint8_t result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;

		// Check if RC has priority and overrides MAVLink for safety-critical commands
		bool rc_has_priority = _param_rc_priority.get() && _command_status.rc_connected;

		switch (cmd.command) {
		case vehicle_command_s::VEHICLE_CMD_COMPONENT_ARM_DISARM: {
				if (rc_has_priority && _command_status.rc_emergency_stop) {
					result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED;
					PX4_WARN("Arm command rejected: RC emergency stop active");

				} else {
					bool arm = (cmd.param1 > 0.5f);
					send_arm_command(arm, arm_cmd_s::SOURCE_GCS);
					_command_status.last_command_type = arm ? command_status_s::LAST_CMD_ARM : command_status_s::LAST_CMD_DISARM;
					_command_status.last_command_timestamp = hrt_absolute_time();
					result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					_mavlink_cmd_accepted++;
				}

				handled = true;
				break;
			}

		case vehicle_command_s::VEHICLE_CMD_DO_SET_MODE: {
				// param1 = mode (custom mode for PX4)
				uint8_t mode = static_cast<uint8_t>(cmd.param1);

				if (mode <= 7) {  // Valid mode range
					send_mode_command(mode, operation_mode_cmd_s::SOURCE_GCS);
					_command_status.last_command_type = command_status_s::LAST_CMD_MODE_CHANGE;
					_command_status.last_command_timestamp = hrt_absolute_time();
					result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					_mavlink_cmd_accepted++;

				} else {
					result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
					PX4_WARN("Invalid mode: %d", mode);
				}

				handled = true;
				break;
			}

		// case vehicle_command_s::VEHICLE_CMD_DO_SET_MISSION_CURRENT: {
		// 		// TODO: Task execution redesign needed
		// 		result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		// 		break;
		// 	}

		case vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR: {
				// Use param1 as safety acknowledge command
				if (cmd.param1 > 0.5f) {
					send_safety_ack_command();
					_command_status.last_command_type = command_status_s::LAST_CMD_SAFETY_ACK;
					_command_status.last_command_timestamp = hrt_absolute_time();
					result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
					_mavlink_cmd_accepted++;

				} else {
					result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
				}

				handled = true;
				break;
			}

		case vehicle_command_s::VEHICLE_CMD_PREFLIGHT_CALIBRATION: {
				// Switch to calibration mode for calibration operations
				send_mode_command(operation_mode_cmd_s::MODE_WL_CALIBRATION, operation_mode_cmd_s::SOURCE_GCS);
				_command_status.last_command_type = command_status_s::LAST_CMD_MODE_CHANGE;
				_command_status.last_command_timestamp = hrt_absolute_time();
				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
				_mavlink_cmd_accepted++;
				handled = true;
				break;
			}

		default:
			break;
		}

		if (handled) {
			send_command_ack(cmd.command, result);

			if (result != vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
				_mavlink_cmd_rejected++;
			}
		}
	}
}

void OperatorInterface::process_rc_inputs()
{
	input_rc_s rc;

	if (!_input_rc_sub.copy(&rc)) {
		return;
	}

	if (!_command_status.rc_connected) {
		return;
	}

	const int rc_threshold = _param_rc_threshold.get();  // RC switch threshold (default 1500)

	// Read RC channels
	int arm_chan = _param_arm_rc_chan.get();
	int mode_chan = _param_mode_rc_chan.get();
	int estop_chan = _param_estop_rc_chan.get();
	int deadman_chan = _param_deadman_rc_chan.get();
	int task_chan = _param_task_rc_chan.get();

	bool rc_arm = false;
	bool rc_estop = false;
	bool rc_deadman = false;
	uint8_t rc_mode = 0;
	uint8_t rc_task = 0;

	// Parse RC channels (channels are 1-indexed, array is 0-indexed)
	if (arm_chan > 0 && arm_chan <= 18) {
		rc_arm = rc.values[arm_chan - 1] > rc_threshold;
	}

	if (estop_chan > 0 && estop_chan <= 18) {
		rc_estop = rc.values[estop_chan - 1] < rc_threshold;  // Active low
	}

	if (deadman_chan > 0 && deadman_chan <= 18) {
		rc_deadman = rc.values[deadman_chan - 1] > rc_threshold;
	}

	if (mode_chan > 0 && mode_chan <= 18) {
		// Map 1000-2000 to 0-7 (8 modes)
		int mode_value = math::constrain((int)rc.values[mode_chan - 1], 1000, 2000);
		rc_mode = (mode_value - 1000) / 125;  // 8 steps of 125us
		rc_mode = math::min(rc_mode, (uint8_t)7);
	}

	if (task_chan > 0 && task_chan <= 18) {
		// Map 1000-2000 to 0-7 (8 tasks)
		int task_value = math::constrain((int)rc.values[task_chan - 1], 1000, 2000);
		rc_task = (task_value - 1000) / 125;
		rc_task = math::min(rc_task, (uint8_t)7);
	}

	// Update command status
	_command_status.rc_arm_switch = rc_arm;
	_command_status.rc_emergency_stop = rc_estop;
	_command_status.rc_deadman_switch = rc_deadman;
	_command_status.rc_mode_switch = rc_mode;
	_command_status.rc_task_trigger = rc_task;
	_command_status.deadman_required = _param_require_deadman.get();
	_command_status.deadman_active = rc_deadman;
	_command_status.emergency_stop_active = rc_estop;

	// Process emergency stop (highest priority)
	if (rc_estop != _prev_rc_emergency_stop) {
		send_emergency_stop(rc_estop);
		_command_status.last_command_type = command_status_s::LAST_CMD_EMERGENCY_STOP;
		_command_status.last_command_timestamp = hrt_absolute_time();
		_prev_rc_emergency_stop = rc_estop;
		_rc_cmd_count++;
	}

	// Process deadman switch
	if (rc_deadman != _prev_rc_deadman) {
		update_deadman_status(rc_deadman);
		_prev_rc_deadman = rc_deadman;
	}

	// Check if commands are allowed
	if (!_command_status.command_allowed) {
		return;
	}

	// Process arm switch
	if (rc_arm != _prev_rc_arm_switch) {
		send_arm_command(rc_arm, arm_cmd_s::SOURCE_RC);
		_command_status.last_command_type = rc_arm ? command_status_s::LAST_CMD_ARM : command_status_s::LAST_CMD_DISARM;
		_command_status.last_command_timestamp = hrt_absolute_time();
		_prev_rc_arm_switch = rc_arm;
		_rc_cmd_count++;
	}

	// Process mode switch
	if (rc_mode != _prev_rc_mode_switch) {
		uint8_t operation_mode = map_rc_mode(rc_mode);
		send_mode_command(operation_mode, operation_mode_cmd_s::SOURCE_RC);
		_command_status.last_command_type = command_status_s::LAST_CMD_MODE_CHANGE;
		_command_status.last_command_timestamp = hrt_absolute_time();
		_prev_rc_mode_switch = rc_mode;
		_rc_cmd_count++;
		_rc_mode_changes++;
	}

	// Process task trigger (on rising edge)
	if (rc_task != _prev_rc_task_trigger && rc_task > 0) {
		uint8_t operation_mode = map_rc_task(rc_task);

		if (operation_mode != 0xFF) {
			// Send mode command to switch to task-specific operation mode
			send_mode_command(operation_mode, operation_mode_cmd_s::SOURCE_RC);
			_command_status.last_command_type = command_status_s::LAST_CMD_TASK_START;
			_command_status.last_command_timestamp = hrt_absolute_time();
			_rc_cmd_count++;
			_rc_task_triggers++;
		}

		_prev_rc_task_trigger = rc_task;

	} else if (rc_task == 0 && _prev_rc_task_trigger != 0) {
		_prev_rc_task_trigger = 0;
	}
}

void OperatorInterface::send_arm_command(bool arm, uint8_t source)
{
	arm_cmd_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.command = arm ? arm_cmd_s::ARM : arm_cmd_s::DISARM;
	cmd.source = source;
	_arm_cmd_pub.publish(cmd);

	PX4_INFO("%s command from %s", arm ? "Arm" : "Disarm", source == arm_cmd_s::SOURCE_RC ? "RC" : "MAVLink");
}

void OperatorInterface::send_mode_command(uint8_t mode, uint8_t source)
{
	operation_mode_cmd_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.target_mode = mode;
	cmd.mode_source = source;
	_operation_mode_cmd_pub.publish(cmd);

	PX4_INFO("Mode %d command from %s", mode, source == operation_mode_cmd_s::SOURCE_RC ? "RC" : "MAVLink");
}

void OperatorInterface::send_task_command(uint8_t task_id, uint8_t action, uint8_t source)
{
	// Tasks are now handled through operation modes
	// Convert task_id to appropriate operation mode
	uint8_t operation_mode = 0xFF;

	switch (task_id) {
	case 1: // Load material task
	case 2: // Dump material task
	case 3: // Travel to point task
		operation_mode = operation_mode_cmd_s::MODE_WL_TRAJ_FOLLOWER;
		break;

	case 4: // Emergency stop
		operation_mode = operation_mode_cmd_s::MODE_WL_SAFETY_STOP;
		break;

	case 5: // Steering calibration
	case 6: // Encoder calibration
	case 7: // Actuator calibration
		operation_mode = operation_mode_cmd_s::MODE_WL_CALIBRATION;
		break;

	default:
		PX4_WARN("Unknown task_id: %d", task_id);
		return;
	}

	if (action == 1) { // START action
		send_mode_command(operation_mode, source);
		PX4_INFO("Task %d started via mode %d", task_id, operation_mode);

	} else { // STOP action - return to hold mode
		send_mode_command(operation_mode_cmd_s::MODE_WL_HOLD, source);
		PX4_INFO("Task %d stopped, returning to hold mode", task_id);
	}
}

void OperatorInterface::send_safety_ack_command()
{
	safety_ack_cmd_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.acknowledge_type = safety_ack_cmd_s::ACK_ALL;
	_safety_ack_cmd_pub.publish(cmd);

	PX4_INFO("Safety acknowledge command");
}

void OperatorInterface::send_emergency_stop(bool stop)
{
	safety_input_s input{};
	input.timestamp = hrt_absolute_time();
	input.emergency_stop_pressed = stop;
	input.deadman_active = _command_status.deadman_active;
	_safety_input_pub.publish(input);

	PX4_WARN("Emergency stop: %s", stop ? "ACTIVE" : "cleared");
}

void OperatorInterface::update_deadman_status(bool active)
{
	safety_input_s input{};
	input.timestamp = hrt_absolute_time();
	input.emergency_stop_pressed = _command_status.emergency_stop_active;
	input.deadman_active = active;
	_safety_input_pub.publish(input);
}

void OperatorInterface::send_command_ack(uint16_t command, uint8_t result)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = hrt_absolute_time();
	ack.command = command;
	ack.result = result;
	_vehicle_command_ack_pub.publish(ack);
}

bool OperatorInterface::validate_command(uint8_t command_type)
{
	// Basic validation - can be extended
	return _command_status.command_allowed;
}

uint8_t OperatorInterface::map_rc_mode(uint8_t rc_value)
{
	// Map RC switch position (0-7) to operation mode
	switch (rc_value) {
	case 0: return operation_mode_cmd_s::MODE_WL_MANUAL_DIRECT;

	case 1: return operation_mode_cmd_s::MODE_WL_MANUAL_BUCKET;

	case 2: return operation_mode_cmd_s::MODE_WL_TRAJ_FOLLOWER;

	case 3: return operation_mode_cmd_s::MODE_WL_HOLD;

	case 4: return operation_mode_cmd_s::MODE_WL_SAFETY_STOP;

	case 5: return operation_mode_cmd_s::MODE_WL_LOITER;

	default: return operation_mode_cmd_s::MODE_WL_HOLD;
	}
}

uint8_t OperatorInterface::map_rc_task(uint8_t rc_value)
{
	// Map RC trigger position (1-7) to operation mode
	// Tasks are now integrated into operation modes
	switch (rc_value) {
	case 1: // Load material
	case 2: // Dump material
	case 3: // Travel to point
		return operation_mode_cmd_s::MODE_WL_TRAJ_FOLLOWER;

	case 4: // Emergency stop
		return operation_mode_cmd_s::MODE_WL_SAFETY_STOP;

	case 5: // Steering calibration
	case 6: // Encoder calibration
	case 7: // Actuator calibration
		return operation_mode_cmd_s::MODE_WL_CALIBRATION;

	default:
		return 0xFF; // Invalid
	}
}

int OperatorInterface::task_spawn(int argc, char *argv[])
{
	OperatorInterface *instance = new OperatorInterface();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int OperatorInterface::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int OperatorInterface::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Operator interface for wheel loader autonomous system.

Receives commands from MAVLink and RC, processes and forwards them to appropriate modules:
- MAVLink: arm/disarm, mode changes, task execution, calibration
- RC: switches for arm, mode, emergency stop, deadman, task trigger
- Validates commands and enforces safety requirements
- Handles command source priority (RC safety overrides)
- Monitors MAVLink and RC heartbeat/connection

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("operator_interface", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int operator_interface_main(int argc, char *argv[])
{
	return OperatorInterface::main(argc, argv);
}
