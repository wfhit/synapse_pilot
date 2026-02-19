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

#include "operation_mode_manager.hpp"
#include <px4_platform_common/log.h>
#include <drivers/drv_hrt.h>

OperationModeManager::OperationModeManager() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	// Initialize with wheel loader trajectory follower mode as default
	_current_mode = &_traj_follower_mode;
	_current_mode_id = operation_mode_cmd_s::MODE_WL_TRAJ_FOLLOWER;
}

OperationModeManager::~OperationModeManager()
{
}

bool OperationModeManager::init()
{
	// Start the work queue
	ScheduleNow();

	return true;
}

void OperationModeManager::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Check for parameter updates
	parameters_update();

	// Check for mode switch commands
	check_mode_switch();

	// Calculate dt
	const hrt_abstime now = hrt_absolute_time();
	float dt = (_last_update_time > 0) ? ((now - _last_update_time) / 1e6f) : 0.02f; // Default to 20ms
	_last_update_time = now;

	// Clamp dt to reasonable values
	dt = math::constrain(dt, 0.001f, 0.1f);

	// Update active mode
	update_active_mode(dt);

	// Publish status
	publish_status();

	// Schedule next iteration
	ScheduleDelayed(SCHEDULE_INTERVAL);
}

void OperationModeManager::check_mode_switch()
{
	operation_mode_cmd_s cmd;

	// Process all pending commands
	while (_operation_mode_cmd_sub.update(&cmd)) {
		// Check if mode switch is needed
		if (cmd.target_mode != _current_mode_id) {
			PX4_INFO("Mode switch request: %d -> %d", _current_mode_id, cmd.target_mode);

			if (switch_to_mode(cmd.target_mode)) {
				PX4_INFO("Mode switch successful to %s", _current_mode->get_name());

			} else {
				PX4_WARN("Mode switch failed, staying in %s", _current_mode->get_name());
			}
		}
	}
}

bool OperationModeManager::switch_to_mode(uint8_t target_mode)
{
	_transition_in_progress = true;

	OperationModeBase *new_mode = nullptr;

	// Select the target mode
	switch (target_mode) {
	case operation_mode_cmd_s::MODE_WL_TRAJ_FOLLOWER:
		new_mode = &_traj_follower_mode;
		break;

	case operation_mode_cmd_s::MODE_WL_MANUAL_BUCKET:
		new_mode = &_manual_bucket_mode;
		break;

	case operation_mode_cmd_s::MODE_WL_MANUAL_DIRECT:
		new_mode = &_manual_direct_mode;
		break;

	case operation_mode_cmd_s::MODE_WL_HOLD:
		new_mode = &_hold_mode;
		break;

	case operation_mode_cmd_s::MODE_WL_LOITER:
		new_mode = &_loiter_mode;
		break;

	case operation_mode_cmd_s::MODE_WL_SAFETY_STOP:
		new_mode = &_safety_stop_mode;
		break;

	default:
		PX4_ERR("Unknown mode: %d", target_mode);
		_transition_in_progress = false;
		return false;
	}

	// Check if new mode is valid
	if (!new_mode->is_valid()) {
		PX4_WARN("Target mode %s not valid", new_mode->get_name());
		_transition_in_progress = false;
		return false;
	}

	// Deactivate current mode
	if (_current_mode && _current_mode->is_active()) {
		_current_mode->deactivate();
	}

	// Activate new mode
	if (new_mode->activate()) {
		_current_mode = new_mode;
		_current_mode_id = target_mode;
		_mode_switch_count++;
		_transition_in_progress = false;
		return true;

	} else {
		PX4_ERR("Failed to activate mode %s", new_mode->get_name());

		// Try to reactivate previous mode as fallback
		if (_current_mode && !_current_mode->is_active()) {
			_current_mode->activate();
		}

		_transition_in_progress = false;
		return false;
	}
}

void OperationModeManager::update_active_mode(float dt)
{
	if (_current_mode && _current_mode->is_active()) {
		_current_mode->update(dt);
	}
}

void OperationModeManager::publish_status()
{
	operation_mode_status_s status{};

	status.timestamp = hrt_absolute_time();
	status.current_mode = _current_mode_id;
	status.transition_in_progress = _transition_in_progress;
	status.mode_ready = (_current_mode != nullptr) && _current_mode->is_valid();
	status.mode_switch_count = _mode_switch_count;

	_operation_mode_status_pub.publish(status);
}

void OperationModeManager::parameters_update()
{
	updateParams();
}

int OperationModeManager::task_spawn(int argc, char *argv[])
{
	OperationModeManager *instance = new OperationModeManager();

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

int OperationModeManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int OperationModeManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Operation Mode Manager - manages independent operation modes for vehicle control

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("operation_mode", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int operation_mode_main(int argc, char *argv[])
{
	return OperationModeManager::main(argc, argv);
}
