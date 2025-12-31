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
 * @file strategy_manager.hpp
 *
 * Strategy Executor - High-level autonomous strategy coordination system
 *
 * Architecture:
 * - Manages multiple concurrent operational strategies with priority-based arbitration
 * - Separates strategy logic (what to do) from execution safety (health monitoring)
 * - Provides comprehensive health monitoring and fault detection framework
 * - Supports graceful degradation and error recovery
 * - Enables strategy preemption with state preservation
 *
 * Key Improvements:
 * - Priority-based command arbitration with source tracking
 * - Comprehensive health monitoring framework (battery, sensors, actuators, localization)
 * - State persistence and recovery mechanisms
 * - Enhanced logging with structured diagnostics
 * - Performance metrics and execution statistics
 * - Configurable failsafe thresholds via parameters
 * - Support for strategy chaining and complex workflows
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/strategy_cmd.h>
#include <uORB/topics/strategy_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_switches.h>

#include "strategy_base.hpp"

#include <lib/hysteresis/hysteresis.h>

/**
 * Command request tracking for priority arbitration
 */
struct CommandRequest {
	uint8_t strategy_id{strategy_cmd_s::STRATEGY_IDLE};
	uint8_t source{strategy_cmd_s::SOURCE_INTERNAL};
	uint8_t priority{0};
	hrt_abstime timestamp{0};

	bool is_valid() const { return timestamp > 0; }
	void clear() { timestamp = 0; }
};

/**
 * Strategy execution statistics
 */
struct StrategyStats {
	uint32_t activation_count{0};
	uint32_t success_count{0};
	uint32_t failure_count{0};
	uint32_t preemption_count{0};
	hrt_abstime total_runtime{0};
	hrt_abstime last_activation{0};
	hrt_abstime last_completion{0};

	void reset() {
		activation_count = 0;
		success_count = 0;
		failure_count = 0;
		preemption_count = 0;
		total_runtime = 0;
		last_activation = 0;
		last_completion = 0;
	}
};

/**
 * @class StrategyExecutor
 * @brief Manages high-level operational strategies with comprehensive health monitoring
 */
class StrategyExecutor : public ModuleBase<StrategyExecutor>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	StrategyExecutor();
	~StrategyExecutor() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	// ========== Strategy Management ==========

	/**
	 * Register all available strategies
	 */
	void register_strategies();

	/**
	 * Process incoming strategy commands with priority arbitration
	 */
	void process_commands();

	/**
	 * Execute strategy switch with validation and state preservation
	 * @return true if switch successful
	 */
	bool execute_strategy_switch(const CommandRequest &request);

	/**
	 * Update currently active strategy
	 */
	void update_active_strategy();

	/**
	 * Gracefully stop current strategy
	 * @param preempted true if stopped due to preemption
	 */
	void stop_current_strategy(bool preempted = false);

	/**
	 * Get strategy instance by ID
	 */
	StrategyBase *get_strategy(uint8_t strategy_id);

	// ========== Health Monitoring ==========

	/**
	 * Monitor system-wide health and trigger failsafes if needed
	 * @return true if system healthy
	 */
	bool monitor_system_health();

	/**
	 * Check battery health status
	 */
	bool check_battery_health();

	/**
	 * Check vehicle status
	 */
	bool check_vehicle_status();

	/**
	 * Check arm/safety status
	 */
	bool check_arm_status();

	/**
	 * Check safety/emergency button status
	 */
	bool check_safety_button();

	// ========== Status & Logging ==========

	/**
	 * Publish comprehensive strategy status
	 */
	void publish_status();

	/**
	 * Update strategy execution statistics
	 */
	void update_statistics(StrategyResult result);

	/**
	 * Log state transition
	 */
	void log_state_transition(uint8_t old_state, uint8_t new_state);

	// ========== uORB Subscriptions ==========
	uORB::SubscriptionCallbackWorkItem _strategy_cmd_sub{this, ORB_ID(strategy_cmd)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _manual_control_switches_sub{ORB_ID(manual_control_switches)};

	// ========== uORB Publications ==========
	uORB::Publication<strategy_status_s> _strategy_status_pub{ORB_ID(strategy_status)};

	// ========== Strategy Registry ==========
	static constexpr size_t MAX_STRATEGIES = 16;
	StrategyBase *_strategies[MAX_STRATEGIES]{};
	size_t _num_strategies{0};

	// ========== Active Strategy State ==========
	StrategyBase *_active_strategy{nullptr};
	uint8_t _active_strategy_id{strategy_cmd_s::STRATEGY_IDLE};
	uint8_t _previous_strategy_state{strategy_status_s::STATE_IDLE};
	hrt_abstime _strategy_activation_time{0};

	// ========== Command Arbitration ==========
	CommandRequest _current_command{};
	CommandRequest _pending_command{};

	// ========== Health Monitoring ==========
	systemlib::Hysteresis _battery_low_hysteresis{false};
	systemlib::Hysteresis _system_critical_hysteresis{false};
	systemlib::Hysteresis _safety_button_hysteresis{false};
	hrt_abstime _last_health_check{0};
	bool _safety_button_triggered{false};
	static constexpr hrt_abstime HEALTH_CHECK_INTERVAL = 100000;

	// ========== Statistics ==========
	StrategyStats _strategy_stats[MAX_STRATEGIES]{};
	uint32_t _total_strategy_switches{0};

	// ========== Status Publication ==========
	strategy_status_s _status{};
	hrt_abstime _last_status_publish{0};
	static constexpr hrt_abstime STATUS_PUBLISH_INTERVAL = 100000;

	// ========== Performance Monitoring ==========
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};
	perf_counter_t _strategy_switch_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": switch")};

	// ========== Module Parameters ==========
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SE_BATT_WARN>) _param_battery_warn,
		(ParamFloat<px4::params::SE_BATT_CRIT>) _param_battery_crit,
		(ParamInt<px4::params::SE_MAX_RUNTIME>) _param_max_runtime,
		(ParamBool<px4::params::SE_AUTO_RECOVERY>) _param_auto_recovery
	)
};
