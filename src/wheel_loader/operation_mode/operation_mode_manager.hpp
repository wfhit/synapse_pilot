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
 * @file operation_mode_manager.hpp
 *
 * Manager for operation modes - handles mode switching and coordination
 *
 * @author Your Name
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/wheel_loader/operation_mode_cmd.h>
#include <uORB/topics/wheel_loader/operation_mode_status.h>
#include <uORB/topics/wheel_loader/operation_control_config.h>
#include <uORB/topics/vehicle_status.h>

#include "operation_mode_base.hpp"
#include "modes/wheel_loader/wheel_loader_traj_follower_mode/wheel_loader_traj_follower_mode.hpp"
#include "modes/wheel_loader/wheel_loader_manual_bucket_mode/wheel_loader_manual_bucket_mode.hpp"
#include "modes/wheel_loader/wheel_loader_manual_direct_mode/wheel_loader_manual_direct_mode.hpp"
#include "modes/wheel_loader/wheel_loader_hold_mode/wheel_loader_hold_mode.hpp"
#include "modes/wheel_loader/wheel_loader_loiter_mode/wheel_loader_loiter_mode.hpp"
#include "modes/wheel_loader/wheel_loader_safety_stop_mode/wheel_loader_safety_stop_mode.hpp"

class OperationModeManager : public ModuleBase<OperationModeManager>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	OperationModeManager();
	~OperationModeManager() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	bool init();

private:
	/**
	 * Check for mode switch commands
	 */
	void check_mode_switch();

	/**
	 * Switch to a new mode
	 * @param target_mode the mode to switch to
	 * @return true if switch successful
	 */
	bool switch_to_mode(uint8_t target_mode);

	/**
	 * Update the currently active mode
	 * @param dt time since last update
	 */
	void update_active_mode(float dt);

	/**
	 * Publish mode status
	 */
	void publish_status();

	/**
	 * Handle parameters update
	 */
	void parameters_update();

	// Subscriptions
	uORB::SubscriptionCallbackWorkItem _operation_mode_cmd_sub{this, ORB_ID(operation_mode_cmd)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// Publications
	uORB::Publication<operation_mode_status_s> _operation_mode_status_pub{ORB_ID(operation_mode_status)};

	// Mode instances
	WheelLoaderTrajFollowerMode _wheel_loader_traj_follower_mode{this};
	WheelLoaderManualBucketMode _wheel_loader_manual_bucket_mode{this};
	WheelLoaderManualDirectMode _wheel_loader_manual_direct_mode{this};

	// Current active mode pointer
	OperationModeBase *_current_mode{nullptr};

	// State tracking
	uint8_t _current_mode_id{operation_mode_cmd_s::MODE_WL_TRAJ_FOLLOWER};
	bool _transition_in_progress{false};
	uint32_t _mode_switch_count{0};
	hrt_abstime _last_update_time{0};

	// Parameters
	// DEFINE_PARAMETERS(
	// 	(ParamFloat<px4::params::OP_MODE_RATE>) _param_update_rate
	// )

	static constexpr uint32_t SCHEDULE_INTERVAL{50000}; // 20 Hz (50ms)
};
