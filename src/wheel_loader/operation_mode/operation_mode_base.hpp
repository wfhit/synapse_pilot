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
 * @file operation_mode_base.hpp
 *
 * Base class for operation modes in the operation_mode system
 *
 * @author Your Name
 */

#pragma once

#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/operation_control_config.h>
#include <matrix/matrix/math.hpp>

class OperationModeBase : public ModuleParams
{
public:
	OperationModeBase(ModuleParams *parent, const char *name);
	virtual ~OperationModeBase() = default;

	/**
	 * Activate this operation mode
	 * Called when switching into this mode
	 * @return true if activation successful, false otherwise
	 */
	virtual bool activate() = 0;

	/**
	 * Deactivate this operation mode
	 * Called when switching out of this mode
	 */
	virtual void deactivate() = 0;

	/**
	 * Update the operation mode control loop
	 * @param dt time since last update in seconds
	 */
	virtual void update(float dt) = 0;

	/**
	 * Check if the mode is valid and can operate
	 * @return true if mode has valid inputs and can run
	 */
	virtual bool is_valid() const = 0;

	/**
	 * Get the mode name
	 * @return const char* mode name
	 */
	const char *get_name() const { return _mode_name; }

	/**
	 * Check if mode is currently active
	 * @return true if mode is active
	 */
	bool is_active() const { return _is_active; }

protected:
	/**
	 * Publish the operation control configuration
	 * Derived classes should call this to update control config
	 */
	void publish_control_config(const operation_control_config_s &config);

	/**
	 * Set mode active state
	 */
	void set_active(bool active) { _is_active = active; }

	const char *_mode_name;
	bool _is_active{false};

	uORB::Publication<operation_control_config_s> _operation_control_config_pub{ORB_ID(operation_control_config)};
};
