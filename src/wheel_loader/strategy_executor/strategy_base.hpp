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
 * @file strategy_base.hpp
 *
 * Base class for all autonomous strategies
 *
 * Design Philosophy:
 * - Strategies define WHAT to do (high-level goals and sequences)
 * - Operation modes define HOW to do it (low-level control primitives)
 * - Strategies orchestrate multiple modes to achieve complex objectives
 * - Clean separation between strategy logic and safety/health monitoring
 *
 * Strategy Lifecycle:
 *   IDLE ← activate() → ACTIVE → PRECHECK → INIT → RUNNING → [IDLE | continue]
 *         ← deactivate() ←
 *
 * Key Improvements:
 * - Return-based error reporting (StrategyResult) instead of state mutation
 * - Separate health monitoring from strategy execution logic
 * - Support for pause/resume and state preservation
 * - Configurable timeouts and retry logic
 * - Extensible step-based execution framework
 */

#pragma once

#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/strategy_status.h>
#include <uORB/topics/operation_mode_cmd.h>
#include "failsafe_base.hpp"

/**
 * Strategy execution result
 */
struct StrategyResult {
	bool success{true};
	const char *message{nullptr};

	StrategyResult() = default;
	StrategyResult(bool s, const char *msg = nullptr) : success(s), message(msg) {}

	// Helper constructors
	static StrategyResult Success() { return {true, nullptr}; }
	static StrategyResult Failure(const char *msg) { return {false, msg}; }
};

/**
 * @class StrategyBase
 * @brief Base class for all high-level autonomous strategies
 *
 * Strategies coordinate multiple operation modes to achieve complex objectives.
 * They are responsible for:
 * - Pre-flight checks and validation
 * - Multi-step sequence execution
 * - Mode coordination and command generation
 * - Progress monitoring
 * - Graceful error handling
 *
 * Strategies are NOT responsible for:
 * - Low-level control (that's operation modes)
 * - Direct actuator commands (goes through modes)
 * - Real-time safety monitoring (that's the executor + failsafe)
 */
class StrategyBase
{
public:
	StrategyBase(const char *name, uint8_t id) :
		_name(name),
		_id(id),
		_state(strategy_status_s::STATE_IDLE),
		_current_step(0),
		_total_steps(0),
		_failsafe(nullptr)
	{}

	virtual ~StrategyBase()
	{
		if (_failsafe != nullptr) {
			delete _failsafe;
			_failsafe = nullptr;
		}
	}

	// ========== Lifecycle Management ==========

	/**
	 * Activate strategy
	 * Called when strategy is selected for execution
	 * @return StrategyResult indicating success/failure
	 */
	virtual StrategyResult activate()
	{
		if (_state != strategy_status_s::STATE_IDLE) {
			return StrategyResult::Failure("Strategy not in IDLE state");
		}

		// Create failsafe monitor if needed
		if (_failsafe == nullptr) {
			_failsafe = create_failsafe();

			if (_failsafe == nullptr) {
				return StrategyResult::Failure("Failed to create failsafe");
			}
		}

		// Reset state
		_start_time = hrt_absolute_time();
		_step_start_time = _start_time;
		_current_step = 0;
		_is_paused = false;

		// Transition to ACTIVE
		_state = strategy_status_s::STATE_ACTIVE;

		return StrategyResult::Success();
	}

	/**
	 * Deactivate strategy
	 * Called when strategy is stopped or completed
	 */
	virtual void deactivate()
	{
		// Cleanup and return to IDLE
		_state = strategy_status_s::STATE_IDLE;
		_current_step = 0;

		// Clear failsafe timeout
		if (_failsafe != nullptr) {
			_failsafe->clear_timeout();
		}

		// Virtual hook for derived classes
		on_deactivate();
	}

	/**
	 * Pause strategy execution
	 * @return StrategyResult indicating success/failure
	 */
	virtual StrategyResult pause()
	{
		if (_state != strategy_status_s::STATE_RUNNING) {
			return StrategyResult::Failure("Can only pause RUNNING strategy");
		}

		_is_paused = true;
		_pause_time = hrt_absolute_time();

		return StrategyResult::Success();
	}

	/**
	 * Resume paused strategy
	 * @return StrategyResult indicating success/failure
	 */
	virtual StrategyResult resume()
	{
		if (!_is_paused) {
			return StrategyResult::Failure("Strategy not paused");
		}

		_is_paused = false;

		// Adjust timing to account for pause duration
		hrt_abstime pause_duration = hrt_absolute_time() - _pause_time;
		_start_time += pause_duration;
		_step_start_time += pause_duration;

		return StrategyResult::Success();
	}

	// ========== State Machine Execution ==========

	/**
	 * Run strategy state machine
	 * Called every cycle by the executor
	 * @return StrategyResult indicating execution status
	 */
	virtual StrategyResult run()
	{
		// Don't execute if paused
		if (_is_paused) {
			return StrategyResult::Success();
		}

		// Execute based on current state
		switch (_state) {
		case strategy_status_s::STATE_ACTIVE:
			// Transition to precheck
			_state = strategy_status_s::STATE_PRECHECK;
			return StrategyResult::Success();

		case strategy_status_s::STATE_PRECHECK:
			return run_precheck();

		case strategy_status_s::STATE_INIT:
			return run_init();

		case strategy_status_s::STATE_RUNNING:
			return run_update();

		case strategy_status_s::STATE_IDLE:
			// Idle - no execution
			return StrategyResult::Success();

		default:
			return StrategyResult::Failure("Unknown state");
		}
	}

	// ========== Information Accessors ==========

	uint8_t get_id() const { return _id; }
	const char *get_name() const { return _name; }
	uint8_t get_state() const { return _state; }
	uint8_t get_current_step() const { return _current_step; }
	uint8_t get_total_steps() const { return _total_steps; }
	bool is_paused() const { return _is_paused; }

	/**
	 * Get current step name for status reporting
	 */
	virtual const char *get_step_name() const { return ""; }

	/**
	 * Get estimated time remaining (microseconds)
	 * @return 0 if unknown
	 */
	virtual hrt_abstime get_time_remaining() const { return 0; }

protected:
	// ========== Virtual Hooks for Derived Classes ==========

	/**
	 * Perform pre-flight checks before initialization
	 * @return StrategyResult - success transitions to INIT, failure to IDLE
	 */
	virtual StrategyResult precheck() = 0;

	/**
	 * Initialize strategy (setup, mode activation, etc.)
	 * @return StrategyResult - success transitions to RUNNING, failure to IDLE
	 */
	virtual StrategyResult init() = 0;

	/**
	 * Update strategy execution (called every cycle in RUNNING state)
	 * @return StrategyResult - failure transitions to IDLE
	 */
	virtual StrategyResult update() = 0;

	/**
	 * Create strategy-specific failsafe monitor
	 * Must be implemented by each strategy
	 */
	virtual FailsafeBase *create_failsafe() = 0;

	/**
	 * Called when strategy is deactivated (cleanup hook)
	 */
	virtual void on_deactivate() {}

	// ========== Helper Methods ==========

	/**
	 * Publish mode command
	 */
	void command_mode(uint8_t mode, uint8_t priority = 100)
	{
		operation_mode_cmd_s cmd{};
		cmd.timestamp = hrt_absolute_time();
		cmd.target_mode = mode;
		cmd.priority = priority;
		_mode_cmd_pub.publish(cmd);
	}

	/**
	 * Advance to next step
	 */
	void next_step()
	{
		_current_step++;
		_step_start_time = hrt_absolute_time();
	}

	/**
	 * Get time elapsed in current step (microseconds)
	 */
	hrt_abstime get_step_elapsed() const
	{
		return hrt_absolute_time() - _step_start_time;
	}

	/**
	 * Get total strategy runtime (microseconds)
	 */
	hrt_abstime get_total_runtime() const
	{
		return hrt_absolute_time() - _start_time;
	}

	// ========== Member Variables ==========
	const char *_name;
	uint8_t _id;
	uint8_t _state;
	uint8_t _current_step;
	uint8_t _total_steps;

	// Timing
	hrt_abstime _start_time{0};
	hrt_abstime _step_start_time{0};
	hrt_abstime _pause_time{0};
	bool _is_paused{false};

	// Failsafe monitor
	FailsafeBase *_failsafe;

	// Mode command publisher
	uORB::Publication<operation_mode_cmd_s> _mode_cmd_pub{ORB_ID(operation_mode_cmd)};

private:
	// ========== State Machine Runners ==========

	StrategyResult run_precheck()
	{
		StrategyResult result = precheck();

		if (result.success) {
			_state = strategy_status_s::STATE_INIT;

		} else {
			_state = strategy_status_s::STATE_IDLE;
		}

		return result;
	}

	StrategyResult run_init()
	{
		StrategyResult result = init();

		if (result.success) {
			_state = strategy_status_s::STATE_RUNNING;

			// Start failsafe timeout
			if (_failsafe != nullptr) {
				_failsafe->reset_timeout();
			}

		} else {
			_state = strategy_status_s::STATE_IDLE;
		}

		return result;
	}

	StrategyResult run_update()
	{
		StrategyResult result = update();

		if (!result.success) {
			_state = strategy_status_s::STATE_IDLE;
		}

		return result;
	}
};
