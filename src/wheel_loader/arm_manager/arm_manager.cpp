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

#include "arm_manager.hpp"

#include <mathlib/mathlib.h>
#include <uORB/topics/operation_mode_cmd.h>

ArmManager::ArmManager() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_arm_status.last_disarm_reason = arm_status_s::DISARM_REASON_INIT;
}

ArmManager::~ArmManager()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool ArmManager::init()
{
	ScheduleOnInterval(SCHEDULE_INTERVAL_US);
	return true;
}

void ArmManager::Run()
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

	// Process any arm/disarm commands
	process_arm_commands();

	// Check for auto-disarm conditions
	if (_armed && _param_auto_disarm.get()) {
		check_auto_disarm();
	}

	// Update and publish arm status
	update_arm_status();
	_arm_status.timestamp = now;
	_arm_status_pub.publish(_arm_status);

	// Publish actuator_armed
	publish_actuator_armed();

	perf_end(_loop_perf);
}

void ArmManager::process_arm_commands()
{
	arm_cmd_s cmd;

	while (_arm_cmd_sub.update(&cmd)) {
		if (cmd.command == arm_cmd_s::ARM) {
			if (_armed) {
				PX4_DEBUG("Already armed");
				continue;
			}

			if (check_can_arm(cmd.force)) {
				arm(cmd.source);
				PX4_INFO("Armed (source: %d)", cmd.source);

			} else {
				_arm_status.arm_reject_count++;
				PX4_WARN("Arm rejected - safety checks failed");
			}

		} else if (cmd.command == arm_cmd_s::DISARM) {
			if (!_armed) {
				PX4_DEBUG("Already disarmed");
				continue;
			}

			disarm(arm_status_s::DISARM_REASON_MANUAL, cmd.source);
			PX4_INFO("Disarmed manually (source: %d)", cmd.source);
		}
	}
}

bool ArmManager::check_can_arm(bool force)
{
	bool can_arm = true;

	// Reset prevention flags
	_arm_status.health_check_failed = false;
	_arm_status.mode_not_allowed = false;
	_arm_status.actuators_not_ready = false;
	_arm_status.manual_control_lost = false;

	// Check health monitor status
	health_monitor_status_s health_status;

	if (_health_monitor_status_sub.copy(&health_status)) {
		if (_param_require_health.get() && !health_status.system_healthy) {
			_arm_status.health_check_failed = true;
			can_arm = false;
			PX4_DEBUG("Arm blocked: system not healthy");
		}

	} else if (_param_require_health.get()) {
		_arm_status.health_check_failed = true;
		can_arm = false;
		PX4_DEBUG("Arm blocked: no health status");
	}

	// Check operation mode - don't allow arming in SAFETY_STOP
	operation_mode_status_s mode_status;

	if (_operation_mode_status_sub.copy(&mode_status)) {
		if (mode_status.current_mode == operation_mode_cmd_s::MODE_WL_SAFETY_STOP) {
			_arm_status.mode_not_allowed = true;
			can_arm = false;
			PX4_DEBUG("Arm blocked: in SAFETY_STOP mode");
		}
	}

	// Check actuator status - ensure all valid
	actuator_status_s actuator_status;

	if (_actuator_status_sub.copy(&actuator_status)) {
		if (!actuator_status.chassis_left_valid ||
		    !actuator_status.chassis_right_valid ||
		    !actuator_status.boom_valid ||
		    !actuator_status.tilt_valid ||
		    !actuator_status.articulation_valid) {
			_arm_status.actuators_not_ready = true;
			can_arm = false;
			PX4_DEBUG("Arm blocked: actuators not ready");
		}

	} else {
		_arm_status.actuators_not_ready = true;
		can_arm = false;
		PX4_DEBUG("Arm blocked: no actuator status");
	}

	// Check manual control (if timeout parameter is set)
	if (_param_manual_timeout.get() > 0) {
		manual_control_setpoint_s manual;

		if (_manual_control_sub.copy(&manual)) {
			const hrt_abstime now = hrt_absolute_time();
			const uint32_t timeout_ms = (now - manual.timestamp) / 1000;

			if (timeout_ms > (uint32_t)_param_manual_timeout.get()) {
				_arm_status.manual_control_lost = true;
				can_arm = false;
				PX4_DEBUG("Arm blocked: manual control timeout");
			}

		} else {
			_arm_status.manual_control_lost = true;
			can_arm = false;
			PX4_DEBUG("Arm blocked: no manual control");
		}
	}

	// Force flag can bypass some checks (but not critical safety)
	if (force && !can_arm) {
		// Still require actuators to be valid
		if (!_arm_status.actuators_not_ready) {
			PX4_WARN("Forcing arm despite safety checks");
			can_arm = true;
		}
	}

	_arm_status.ready_to_arm = can_arm;
	return can_arm;
}

void ArmManager::arm(uint8_t source)
{
	const hrt_abstime now = hrt_absolute_time();

	_armed = true;
	_last_arm_time = now;
	_last_activity_time = now;
	_idle_start_time = 0;
	_was_idle = false;

	PX4_INFO("System ARMED");
}

void ArmManager::disarm(uint8_t reason, uint8_t source)
{
	const hrt_abstime now = hrt_absolute_time();

	_armed = false;
	_last_disarm_time = now;
	_arm_status.last_disarm_reason = reason;

	const char *reason_str = "unknown";

	switch (reason) {
	case arm_status_s::DISARM_REASON_MANUAL: reason_str = "manual"; break;

	case arm_status_s::DISARM_REASON_TIMEOUT: reason_str = "idle timeout"; break;

	case arm_status_s::DISARM_REASON_HEALTH: reason_str = "health failure"; break;

	case arm_status_s::DISARM_REASON_EMERGENCY: reason_str = "emergency"; break;
	}

	PX4_WARN("System DISARMED (%s)", reason_str);
}

void ArmManager::check_auto_disarm()
{
	// Check for health failures
	health_monitor_status_s health_status;

	if (_health_monitor_status_sub.copy(&health_status)) {
		if (!health_status.system_healthy) {
			disarm(arm_status_s::DISARM_REASON_HEALTH, arm_cmd_s::SOURCE_AUTO);
			return;
		}

		// Emergency conditions
		if (!health_status.vehicle_stable) {
			disarm(arm_status_s::DISARM_REASON_EMERGENCY, arm_cmd_s::SOURCE_AUTO);
			return;
		}
	}

	// Check for idle timeout
	if (check_system_idle()) {
		const hrt_abstime now = hrt_absolute_time();

		if (_idle_start_time == 0) {
			_idle_start_time = now;
		}

		const float idle_duration = (now - _idle_start_time) / 1e6f;

		if (idle_duration > _param_idle_timeout.get()) {
			disarm(arm_status_s::DISARM_REASON_TIMEOUT, arm_cmd_s::SOURCE_AUTO);
		}

	} else {
		// Reset idle timer if system becomes active
		_idle_start_time = 0;
	}
}

bool ArmManager::check_system_idle()
{
	const hrt_abstime now = hrt_absolute_time();
	const float idle_vel_threshold = _param_idle_velocity.get();
	bool is_idle = true;

	// Check vehicle velocity
	vehicle_local_position_s local_pos;

	if (_vehicle_local_position_sub.copy(&local_pos)) {
		const float velocity = sqrtf(
					       local_pos.vx * local_pos.vx +
					       local_pos.vy * local_pos.vy
				       );

		if (velocity > idle_vel_threshold) {
			is_idle = false;
			_last_activity_time = now;
		}
	}

	// Check operation mode (HOLD and LOITER are considered idle)
	operation_mode_status_s mode_status;

	if (_operation_mode_status_sub.copy(&mode_status)) {
		if (mode_status.current_mode != operation_mode_cmd_s::MODE_WL_HOLD &&
		    mode_status.current_mode != operation_mode_cmd_s::MODE_WL_LOITER) {
			is_idle = false;
			_last_activity_time = now;
		}
	}

	// Check manual control input
	manual_control_setpoint_s manual;

	if (_manual_control_sub.copy(&manual)) {
		// Check if any stick is deflected
		const float stick_threshold = 0.05f;

		if (fabsf(manual.roll) > stick_threshold ||
		    fabsf(manual.pitch) > stick_threshold ||
		    fabsf(manual.yaw) > stick_threshold ||
		    fabsf(manual.throttle - 0.5f) > stick_threshold) {
			is_idle = false;
			_last_activity_time = now;
		}
	}

	return is_idle;
}

void ArmManager::publish_actuator_armed()
{
	actuator_armed_s actuator_armed{};
	actuator_armed.timestamp = hrt_absolute_time();
	actuator_armed.armed = _armed;
	actuator_armed.ready_to_arm = _arm_status.ready_to_arm;
	actuator_armed.lockdown = false;
	// actuator_armed.manual_lockdown = false; // Field doesn't exist
	// actuator_armed.force_failsafe = !_armed; // Field doesn't exist
	actuator_armed.prearmed = false;

	_actuator_armed_pub.publish(actuator_armed);
}

void ArmManager::update_arm_status()
{
	const hrt_abstime now = hrt_absolute_time();

	_arm_status.armed = _armed;

	// Update timing
	if (_last_arm_time > 0) {
		_arm_status.time_since_armed = (now - _last_arm_time) / 1e6f;

	} else {
		_arm_status.time_since_armed = -1.0f;
	}

	if (_last_disarm_time > 0) {
		_arm_status.time_since_disarmed = (now - _last_disarm_time) / 1e6f;

	} else {
		_arm_status.time_since_disarmed = -1.0f;
	}

	// Update idle status
	_arm_status.system_idle = check_system_idle();

	if (_arm_status.system_idle && _idle_start_time > 0) {
		_arm_status.idle_time = (now - _idle_start_time) / 1e6f;

	} else {
		_arm_status.idle_time = 0.0f;
	}

	// Update ready_to_arm flag if not armed
	if (!_armed) {
		check_can_arm(false);
	}
}

int ArmManager::task_spawn(int argc, char *argv[])
{
	ArmManager *instance = new ArmManager();

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

int ArmManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ArmManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Arm manager for wheel loader system. Manages arming/disarming with safety checks.
- Processes arm/disarm commands from multiple sources (GCS, RC, onboard)
- Checks system health before allowing arming
- Auto-disarms on idle timeout or health failures
- Publishes actuator_armed for actuator enabling

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("arm_manager", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int arm_manager_main(int argc, char *argv[])
{
	return ArmManager::main(argc, argv);
}
