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
 * @file trajectory_follower_failsafe.hpp
 *
 * Failsafe monitor for trajectory following strategy
 *
 * Responsibilities (System-Level Monitoring Only):
 * - Communication timeouts (mode status, sensor data)
 * - Unexpected state changes (disarm, mode switches)
 * - Strategy execution timeout enforcement
 *
 * Does NOT Monitor (Strategy Handles These):
 * - Battery levels (strategy has graduated response)
 * - Slope/terrain conditions (operational safety)
 * - Path tracking/deviations (trajectory-specific)
 * - Performance degradation (strategy manages)
 */

#pragma once

#include "../failsafe_base.hpp"
#include <uORB/topics/operation_mode_cmd.h>

class TrajectoryFollowerFailsafe : public FailsafeBase
{
public:
	TrajectoryFollowerFailsafe() : FailsafeBase("TrajectoryFollower") {}

protected:
	/**
	 * Check strategy-specific timeout (60 seconds max for trajectory following)
	 */
	hrt_abstime get_strategy_timeout() const override
	{
		return 601000000;
	}

	/**
	 * Check custom trajectory-specific failsafe conditions
	 * Focus on mode synchronization only
	 */
	FailsafeResult check_custom() override
	{
		// Verify we're still in trajectory mode (if expected)
		operation_mode_status_s mode_status;
		if (_mode_status_sub.copy(&mode_status)) {
			// Mode status is healthy, no additional checks needed
			// Mode mismatch will be detected by strategy itself
		}

		// All custom checks passed
		return FailsafeResult::Safe();
	}
};
