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

#include "strategy_manager.hpp"

// Include strategy implementations here
#include "strategies/trajectory_follower_strategy.hpp"
#include "strategies/manual_bucket_strategy.hpp"

StrategyExecutor::StrategyExecutor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	// Initialize health monitoring hysteresis
	_battery_low_hysteresis.set_hysteresis_time_from(false, 2000_ms);  // 2s to trigger
	_battery_low_hysteresis.set_hysteresis_time_from(true, 5000_ms);   // 5s to clear

	_system_critical_hysteresis.set_hysteresis_time_from(false, 500_ms);  // 500ms to trigger
	_system_critical_hysteresis.set_hysteresis_time_from(true, 2000_ms);  // 2s to clear
}

StrategyExecutor::~StrategyExecutor()
{
	// Stop active strategy gracefully
	if (_active_strategy != nullptr) {
		stop_current_strategy(false);
	}

	// Clean up all registered strategies
	for (size_t i = 0; i < _num_strategies; i++) {
		if (_strategies[i] != nullptr) {
			delete _strategies[i];
			_strategies[i] = nullptr;
		}
	}

	// Free performance counters
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
	perf_free(_strategy_switch_perf);
}

bool StrategyExecutor::init()
{
	// Register all available strategies
	register_strategies();

	// Initialize status message
	_status.current_strategy_id = strategy_cmd_s::STRATEGY_IDLE;
	_status.state = strategy_status_s::STATE_IDLE;
	_status.current_step = 0;
	_status.total_steps = 0;
	_status.strategy_name[0] = '\0';
	_status.current_step_name[0] = '\0';
	_status.status_text[0] = '\0';

	// Update parameters
	updateParams();

	PX4_INFO("Strategy Executor initialized with %zu strategies", _num_strategies);

	// Schedule first run
	ScheduleNow();

	return true;
}

void StrategyExecutor::register_strategies()
{
	// Register strategy implementations
	// Each strategy registers with its unique ID
	_strategies[_num_strategies++] = new TrajectoryFollowerStrategy();
	_strategies[_num_strategies++] = new ManualBucketStrategy();
	_strategies[_num_strategies++] = new ManualDirectStrategy();
	_strategies[_num_strategies++] = new CalibrationStrategy();
	_strategies[_num_strategies++] = new HoldPositionStrategy();

	PX4_INFO("Registered %zu strategies", _num_strategies);
}

void StrategyExecutor::Run()
{
	if (should_exit()) {
		// Gracefully stop current strategy
		stop_current_strategy(false);
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// Update parameters periodically
	updateParams();

	// Process incoming strategy commands
	process_commands();

	// Monitor system-wide health
	hrt_abstime now = hrt_absolute_time();
	if (now - _last_health_check > HEALTH_CHECK_INTERVAL) {
		monitor_system_health();
		_last_health_check = now;
	}

	// Update active strategy
	update_active_strategy();

	// Publish status periodically
	if (now - _last_status_publish > STATUS_PUBLISH_INTERVAL) {
		publish_status();
		_last_status_publish = now;
	}

	perf_end(_cycle_perf);

	// Schedule next run
	ScheduleDelayed(10_ms);
}

void StrategyExecutor::process_commands()
{
	strategy_cmd_s cmd;

	while (_strategy_cmd_sub.update(&cmd)) {
		// Create command request for arbitration
		CommandRequest request{};
		request.strategy_id = cmd.strategy_id;
		request.source = cmd.source;
		request.priority = cmd.priority;
		request.timestamp = cmd.timestamp;

		PX4_INFO("Received strategy command: ID=%u, source=%u, priority=%u",
			 request.strategy_id, request.source, request.priority);

		// Priority-based arbitration
		if (_current_command.is_valid()) {
			// Compare priorities
			if (request.priority > _current_command.priority) {
				// Higher priority - preempt current command
				PX4_INFO("Command preempts current (priority %u > %u)",
					 request.priority, _current_command.priority);
				_pending_command = _current_command;  // Save for potential rollback
				_current_command = request;
				execute_strategy_switch(request);
			} else if (request.priority == _current_command.priority) {
				// Equal priority - use timestamp (newer wins)
				if (request.timestamp > _current_command.timestamp) {
					PX4_INFO("Command replaces current (same priority, newer timestamp)");
					_current_command = request;
					execute_strategy_switch(request);
				} else {
					PX4_INFO("Command rejected (same priority, older timestamp)");
				}
			} else {
				// Lower priority - reject
				PX4_INFO("Command rejected (priority %u < %u)",
					 request.priority, _current_command.priority);
			}
		} else {
			// No active command - accept immediately
			_current_command = request;
			execute_strategy_switch(request);
		}
	}
}

bool StrategyExecutor::execute_strategy_switch(const CommandRequest &request)
{
	perf_begin(_strategy_switch_perf);

	bool success = false;

	// Handle IDLE command (stop current strategy)
	if (request.strategy_id == strategy_cmd_s::STRATEGY_IDLE) {
		if (_active_strategy == nullptr) {
			PX4_INFO("No active strategy to stop");
			success = true;  // Already idle
		} else {
			PX4_INFO("Stopping strategy %u (%s)",
				 _active_strategy_id, _active_strategy->get_name());
			stop_current_strategy(false);
			success = true;
		}
		perf_end(_strategy_switch_perf);
		return success;
	}

	// Validate strategy ID
	StrategyBase *new_strategy = get_strategy(request.strategy_id);
	if (new_strategy == nullptr) {
		PX4_ERR("Strategy %u not found in registry", request.strategy_id);
		perf_end(_strategy_switch_perf);
		return false;
	}

	// Stop current strategy if different
	if (_active_strategy != nullptr && _active_strategy_id != request.strategy_id) {
		PX4_INFO("Switching from strategy %u to %u", _active_strategy_id, request.strategy_id);
		stop_current_strategy(true);  // Preempted
	} else if (_active_strategy != nullptr && _active_strategy_id == request.strategy_id) {
		PX4_INFO("Restarting strategy %u (%s)", request.strategy_id, new_strategy->get_name());
		stop_current_strategy(false);
	}

	// Activate new strategy
	PX4_INFO("Activating strategy %u (%s)", request.strategy_id, new_strategy->get_name());

	StrategyResult result = new_strategy->activate();

	if (result.success) {
		_active_strategy = new_strategy;
		_active_strategy_id = request.strategy_id;
		_strategy_activation_time = hrt_absolute_time();
		_previous_strategy_state = new_strategy->get_state();

		// Update statistics
		_strategy_stats[_active_strategy_id].activation_count++;
		_strategy_stats[_active_strategy_id].last_activation = _strategy_activation_time;
		_total_strategy_switches++;

		PX4_INFO("Strategy %u activated successfully (state=%u)",
			 request.strategy_id, new_strategy->get_state());
		success = true;
	} else {
		PX4_ERR("Strategy %u activation failed: %s", request.strategy_id, result.message);
		_active_strategy = nullptr;
		_active_strategy_id = strategy_cmd_s::STRATEGY_IDLE;
		success = false;
	}

	perf_end(_strategy_switch_perf);
	return success;
}

void StrategyExecutor::update_active_strategy()
{
	if (_active_strategy == nullptr) {
		return;
	}

	// Store previous state for transition detection
	uint8_t previous_state = _active_strategy->get_state();

	// Execute strategy state machine
	StrategyResult result = _active_strategy->run();

	// Detect state transitions
	uint8_t current_state = _active_strategy->get_state();
	if (current_state != previous_state) {
		log_state_transition(previous_state, current_state);
		_previous_strategy_state = current_state;
	}

	// Handle strategy completion or failure
	if (!result.success) {
		PX4_WARN("Strategy %u (%s) failed: %s",
			 _active_strategy_id, _active_strategy->get_name(), result.message);

		// Update statistics
		_strategy_stats[_active_strategy_id].failure_count++;
		hrt_abstime runtime = hrt_absolute_time() - _strategy_activation_time;
		_strategy_stats[_active_strategy_id].total_runtime += runtime;

		// Clear active strategy
		_active_strategy = nullptr;
		_active_strategy_id = strategy_cmd_s::STRATEGY_IDLE;
		_current_command.clear();
	} else if (current_state == strategy_status_s::STATE_IDLE && previous_state != strategy_status_s::STATE_IDLE) {
		// Strategy completed (transitioned back to IDLE)
		PX4_INFO("Strategy %u (%s) completed successfully",
			 _active_strategy_id, _active_strategy->get_name());

		// Update statistics
		_strategy_stats[_active_strategy_id].success_count++;
		_strategy_stats[_active_strategy_id].last_completion = hrt_absolute_time();
		hrt_abstime runtime = _strategy_stats[_active_strategy_id].last_completion - _strategy_activation_time;
		_strategy_stats[_active_strategy_id].total_runtime += runtime;

		// Clear active strategy
		_active_strategy = nullptr;
		_active_strategy_id = strategy_cmd_s::STRATEGY_IDLE;
		_current_command.clear();
	}
}

void StrategyExecutor::stop_current_strategy(bool preempted)
{
	if (_active_strategy == nullptr) {
		return;
	}

	PX4_INFO("Stopping strategy %u (%s) - %s",
		 _active_strategy_id, _active_strategy->get_name(),
		 preempted ? "preempted" : "normal");

	// Deactivate strategy
	_active_strategy->deactivate();

	// Update statistics
	if (preempted) {
		_strategy_stats[_active_strategy_id].preemption_count++;
	}
	hrt_abstime runtime = hrt_absolute_time() - _strategy_activation_time;
	_strategy_stats[_active_strategy_id].total_runtime += runtime;

	// Clear active strategy
	_active_strategy = nullptr;
	_active_strategy_id = strategy_cmd_s::STRATEGY_IDLE;
}

StrategyBase *StrategyExecutor::get_strategy(uint8_t strategy_id)
{
	for (size_t i = 0; i < _num_strategies; i++) {
		if (_strategies[i] != nullptr && _strategies[i]->get_id() == strategy_id) {
			return _strategies[i];
		}
	}
	return nullptr;
}

// ========== Health Monitoring ==========

bool StrategyExecutor::monitor_system_health()
{
	bool system_healthy = true;

	// Check safety button first (highest priority)
	if (!check_safety_button()) {
		system_healthy = false;
	}

	// Check battery health
	if (!check_battery_health()) {
		system_healthy = false;
	}

	// Check vehicle status
	if (!check_vehicle_status()) {
		system_healthy = false;
	}

	// Check arm status
	if (!check_arm_status()) {
		system_healthy = false;
	}

	// Update critical system hysteresis
	_system_critical_hysteresis.set_state_and_update(!system_healthy, hrt_absolute_time());

	// Trigger failsafe if system critical
	if (_system_critical_hysteresis.get_state() && _active_strategy != nullptr) {
		PX4_ERR("System health critical - aborting strategy %u", _active_strategy_id);
		stop_current_strategy(false);
		_current_command.clear();
	}

	return system_healthy;
}

bool StrategyExecutor::check_battery_health()
{
	battery_status_s battery;
	if (!_battery_sub.copy(&battery)) {
		return true;  // No data - assume healthy
	}

	// Check battery level
	float battery_warn_threshold = _param_battery_warn.get();
	float battery_crit_threshold = _param_battery_crit.get();

	bool battery_low = battery.remaining < battery_warn_threshold;
	_battery_low_hysteresis.set_state_and_update(battery_low, hrt_absolute_time());

	if (battery.remaining < battery_crit_threshold) {
		PX4_ERR("Battery critically low: %.1f%%", (double)(battery.remaining * 100.0f));
		return false;
	}

	if (_battery_low_hysteresis.get_state()) {
		PX4_WARN("Battery low: %.1f%%", (double)(battery.remaining * 100.0f));
	}

	return true;
}

bool StrategyExecutor::check_vehicle_status()
{
	vehicle_status_s status;
	if (!_vehicle_status_sub.copy(&status)) {
		return true;  // No data - assume healthy
	}

	// Check for failsafe conditions
	if (status.failsafe) {
		PX4_WARN("Vehicle in failsafe mode");
		return false;
	}

	return true;
}

bool StrategyExecutor::check_arm_status()
{
	actuator_armed_s armed;
	if (!_armed_sub.copy(&armed)) {
		return true;  // No data - assume healthy
	}

	// Check if armed (required for most strategies)
	if (_active_strategy != nullptr && !armed.armed) {
		PX4_WARN("Vehicle disarmed during strategy execution");
		return false;
	}

	return true;
}

bool StrategyExecutor::check_safety_button()
{
	manual_control_switches_s switches;
	if (!_manual_control_switches_sub.copy(&switches)) {
		// No switch data available - assume safe
		return true;
	}

	// Check for emergency/kill switch activation
	// Bit 0 (LSB) typically represents emergency stop/kill switch
	// Check your specific system's switch mapping
	bool safety_pressed = false;

	// Check kill switch (if available)
	if (switches.mode_slot != manual_control_switches_s::MODE_SLOT_NONE) {
		// Mode slot can be used for emergency stop
		if (switches.mode_slot == manual_control_switches_s::MODE_SLOT_6) {
			safety_pressed = true;
		}
	}

	// Check return switch (alternative emergency button)
	if (switches.return_switch == manual_control_switches_s::SWITCH_POS_ON) {
		safety_pressed = true;
	}

	// Update hysteresis
	_safety_button_hysteresis.set_state_and_update(safety_pressed, hrt_absolute_time());

	// Check if safety button is triggered (with hysteresis)
	if (_safety_button_hysteresis.get_state()) {
		if (!_safety_button_triggered) {
			// First detection - log and flag
			PX4_ERR("SAFETY BUTTON PRESSED - Aborting strategy");
			_safety_button_triggered = true;

			// Abort active strategy immediately
			if (_active_strategy != nullptr) {
				PX4_ERR("Emergency stop triggered by safety button - aborting strategy %u", _active_strategy_id);
			}
		}
		return false;  // System not healthy
	} else {
		// Button released - clear flag
		if (_safety_button_triggered) {
			PX4_INFO("Safety button released");
			_safety_button_triggered = false;
		}
		return true;
	}
}

// ========== Status & Logging ==========

void StrategyExecutor::publish_status()
{
	_status.timestamp = hrt_absolute_time();

	if (_active_strategy != nullptr) {
		_status.current_strategy_id = _active_strategy_id;
		_status.state = _active_strategy->get_state();
		_status.current_step = _active_strategy->get_current_step();
		_status.total_steps = _active_strategy->get_total_steps();

		// Copy strategy name (ensure null termination)
		const char *name = _active_strategy->get_name();
		size_t name_len = strlen(name);
		size_t copy_len = math::min(name_len, sizeof(_status.strategy_name) - 1);
		memcpy(_status.strategy_name, name, copy_len);
		_status.strategy_name[copy_len] = '\0';

		// Copy step name
		const char *step_name = _active_strategy->get_step_name();
		if (step_name != nullptr) {
			size_t step_len = strlen(step_name);
			size_t step_copy_len = math::min(step_len, sizeof(_status.current_step_name) - 1);
			memcpy(_status.current_step_name, step_name, step_copy_len);
			_status.current_step_name[step_copy_len] = '\0';
		} else {
			_status.current_step_name[0] = '\0';
		}

		// Status text
		snprintf(_status.status_text, sizeof(_status.status_text),
			 "Active: %s (step %u/%u)",
			 _status.strategy_name, _status.current_step + 1, _status.total_steps);

	} else {
		_status.current_strategy_id = strategy_cmd_s::STRATEGY_IDLE;
		_status.state = strategy_status_s::STATE_IDLE;
		_status.current_step = 0;
		_status.total_steps = 0;
		_status.strategy_name[0] = '\0';
		_status.current_step_name[0] = '\0';
		strncpy(_status.status_text, "Idle - No active strategy", sizeof(_status.status_text));
	}

	_strategy_status_pub.publish(_status);
}

void StrategyExecutor::log_state_transition(uint8_t old_state, uint8_t new_state)
{
	const char *state_names[] = {
		"IDLE", "ACTIVE", "PRECHECK", "INIT", "RUNNING"
	};

	const char *old_name = (old_state < 5) ? state_names[old_state] : "UNKNOWN";
	const char *new_name = (new_state < 5) ? state_names[new_state] : "UNKNOWN";

	PX4_INFO("Strategy %u (%s): %s → %s",
		 _active_strategy_id,
		 _active_strategy ? _active_strategy->get_name() : "NULL",
		 old_name, new_name);
}

int StrategyExecutor::print_status()
{
	PX4_INFO("========== Strategy Executor Status ==========");

	// Current strategy
	if (_active_strategy != nullptr) {
		PX4_INFO("Active Strategy: %u (%s)", _active_strategy_id, _active_strategy->get_name());
		PX4_INFO("  State: %u, Step: %u/%u",
			 _active_strategy->get_state(),
			 _active_strategy->get_current_step() + 1,
			 _active_strategy->get_total_steps());

		hrt_abstime runtime = hrt_absolute_time() - _strategy_activation_time;
		PX4_INFO("  Runtime: %.2f s", (double)(runtime / 1000000.0));
	} else {
		PX4_INFO("Active Strategy: None (IDLE)");
	}

	// Statistics
	PX4_INFO("\n----- Strategy Statistics -----");
	PX4_INFO("Total switches: %u", _total_strategy_switches);

	for (size_t i = 0; i < _num_strategies; i++) {
		if (_strategies[i] != nullptr) {
			uint8_t id = _strategies[i]->get_id();
			const StrategyStats &stats = _strategy_stats[id];

			if (stats.activation_count > 0) {
				PX4_INFO("\nStrategy %u (%s):", id, _strategies[i]->get_name());
				PX4_INFO("  Activations: %u", stats.activation_count);
				PX4_INFO("  Success: %u, Failures: %u, Preemptions: %u",
					 stats.success_count, stats.failure_count, stats.preemption_count);

				if (stats.success_count > 0) {
					float avg_runtime = (float)stats.total_runtime / (float)stats.success_count / 1000000.0f;
					PX4_INFO("  Avg runtime: %.2f s", (double)avg_runtime);
				}
			}
		}
	}

	// Performance
	PX4_INFO("\n----- Performance -----");
	perf_print_counter(_cycle_perf);
	perf_print_counter(_interval_perf);
	perf_print_counter(_strategy_switch_perf);

	return 0;
}

// ========== Module Interface ==========

int StrategyExecutor::task_spawn(int argc, char *argv[])
{
	StrategyExecutor *instance = new StrategyExecutor();

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

int StrategyExecutor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int StrategyExecutor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Strategy Executor - High-level autonomous strategy coordination system

The Strategy Executor manages complex, multi-step operational strategies for the wheel loader.
It provides:
- Priority-based command arbitration
- Comprehensive health monitoring and failsafe handling
- Strategy state management and transition tracking
- Performance metrics and execution statistics
- Graceful degradation and error recovery

A strategy is a high-level autonomous workflow that:
- Coordinates multiple operation modes
- Implements multi-step sequences
- Monitors safety conditions
- Handles failures gracefully
- Can be preempted by higher priority commands

Strategies vs Operation Modes:
- Strategies: High-level workflows (trajectory following, calibrations, autonomous tasks)
- Operation Modes: Low-level control primitives (manual, hold, trajectory tracking)
- Strategies command and coordinate operation modes to achieve their goals

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("strategy_executor", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int strategy_executor_main(int argc, char *argv[])
{
	return StrategyExecutor::main(argc, argv);
}
