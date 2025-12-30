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

#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/manual_control_setpoint.h>

#include <uORB/topics/arm_cmd.h>
#include <uORB/topics/arm_status.h>
#include <uORB/topics/operation_mode_status.h>
#include <uORB/topics/health_monitor_status.h>
#include <uORB/topics/strategy_status.h>
#include <uORB/topics/actuator_status.h>

/**
 * @brief Arm Manager
 *
 * Manages system arming/disarming with safety checks.
 * - Enables actuators when armed
 * - Auto-disarms on idle timeout or health failures
 * - Accepts arm/disarm commands from multiple sources
 */
class ArmManager : public ModuleBase<ArmManager>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	ArmManager();
	~ArmManager() override;

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
	 * Process arm/disarm commands
	 */
	void process_arm_commands();

	/**
	 * Check if system can be armed
	 */
	bool check_can_arm(bool force);

	/**
	 * Execute arm operation
	 */
	void arm(uint8_t source);

	/**
	 * Execute disarm operation
	 */
	void disarm(uint8_t reason, uint8_t source);

	/**
	 * Check for auto-disarm conditions
	 */
	void check_auto_disarm();

	/**
	 * Check if system is idle
	 */
	bool check_system_idle();

	/**
	 * Publish actuator_armed message
	 */
	void publish_actuator_armed();

	/**
	 * Update arm status
	 */
	void update_arm_status();

	// uORB subscriptions
	uORB::SubscriptionCallbackWorkItem _arm_cmd_sub{this, ORB_ID(arm_cmd)};
	uORB::Subscription _operation_mode_status_sub{ORB_ID(operation_mode_status)};
	uORB::Subscription _health_monitor_status_sub{ORB_ID(health_monitor_status)};
	uORB::Subscription _strategy_status_sub{ORB_ID(strategy_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _actuator_status_sub{ORB_ID(actuator_status)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};

	// Publications
	uORB::Publication<arm_status_s> _arm_status_pub{ORB_ID(arm_status)};
	uORB::Publication<actuator_armed_s> _actuator_armed_pub{ORB_ID(actuator_armed)};

	// State
	arm_status_s _arm_status{};
	bool _armed{false};

	// Timing
	hrt_abstime _last_run_timestamp{0};
	hrt_abstime _last_arm_time{0};
	hrt_abstime _last_disarm_time{0};
	hrt_abstime _last_activity_time{0};
	hrt_abstime _idle_start_time{0};

	// Activity tracking
	bool _was_idle{false};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::ARM_IDLE_TIMEOUT>) _param_idle_timeout,
		(ParamFloat<px4::params::ARM_IDLE_VEL>) _param_idle_velocity,
		(ParamBool<px4::params::ARM_REQ_HEALTH>) _param_require_health,
		(ParamBool<px4::params::ARM_AUTO_DISARM>) _param_auto_disarm,
		(ParamInt<px4::params::ARM_MAN_TIMEOUT>) _param_manual_timeout
	)

	static constexpr uint32_t SCHEDULE_INTERVAL_US = 50000;	// 20 Hz (50ms)
};
