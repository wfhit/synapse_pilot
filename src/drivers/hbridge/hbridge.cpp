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

#include "hbridge.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_arch/io_timer.h>
#include <px4_arch/micro_hal.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_motor_pwm.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/hbridge_setpoint.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/sensor_limit_switch.h>
#include <board_config.h>
#include <drivers/hbridge/hbridge_config.h>

// External declaration of board configuration
#ifdef BOARD_HAS_HBRIDGE_CONFIG
extern hbridge_config_t hbridge_configs[];
extern hbridge_manager_config_t hbridge_manager_config;
#endif

// Static member initialization
HBridge *HBridge::_instances[MAX_INSTANCES] = {};
px4::atomic<uint8_t> HBridge::_num_instances{0};
HBridge *HBridge::_manager_instance = nullptr;
px4::atomic<uint32_t> HBridge::_initialized_pwm_channels{0};

HBridge::HBridge(uint8_t instance) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_instance(instance),
	_parameter_update_sub(ORB_ID(parameter_update)),
	_command_sub(ORB_ID::hbridge_setpoint),
	_limit_sensor_sub(ORB_ID::sensor_limit_switch),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_command_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": command"))
{
	// Initialize state
	_current_duty_cycle = 0.0f;
	_forward_limit_active = false;
	_reverse_limit_active = false;
	_initialized = false;

	// Set manager instance if this is the manager
	if (_instance == MANAGER_INSTANCE) {
		_manager_instance = this;

	} else {
		// Register regular instances
		if (_instance < MAX_INSTANCES) {
			_instances[_instance] = this;
			_num_instances.fetch_add(1);
		}
	}
}

HBridge::~HBridge()
{
	// Stop the work queue
	ScheduleClear();

	// Manager instance cleanup
	if (_instance == MANAGER_INSTANCE) {
		// Disable H-bridge before stopping instances
		output_enable(false);
		stop_all_instances();
		_manager_instance = nullptr;

	} else {
		// Stop motor before cleanup for regular instances
		_current_duty_cycle = 0.0f;
		output_pwm();
	}

	// Unadvertise publication
	if (_pub_handle != nullptr) {
		orb_unadvertise(_pub_handle);
	}

	// Unregister this instance
	if (_instance < MAX_INSTANCES && _instances[_instance] == this) {
		// Remove this channel from the initialized PWM channels mask
		if (_board_config != nullptr) {
			uint32_t current_channels = _initialized_pwm_channels.load();
			uint32_t this_channel_mask = (1 << _board_config->pwm_ch);
			uint32_t new_channels = current_channels & ~this_channel_mask;
			_initialized_pwm_channels.store(new_channels);
			PX4_DEBUG("Removed PWM channel %d from initialized mask (remaining: 0x%08lx)",
				  _board_config->pwm_ch, (unsigned long)new_channels);
		}

		_instances[_instance] = nullptr;
		_num_instances.fetch_sub(1);
	}

	// Free performance counters
	perf_free(_loop_perf);
	perf_free(_command_perf);
}

const hbridge_config_t *HBridge::get_board_config(uint8_t instance)
{
#ifdef BOARD_HAS_HBRIDGE_CONFIG

	if (instance < HBRIDGE_MAX_INSTANCES && hbridge_configs[instance].enabled) {
		return &hbridge_configs[instance];
	}

#endif
	return nullptr;
}

const hbridge_manager_config_t *HBridge::get_manager_config()
{
#ifdef BOARD_HAS_HBRIDGE_CONFIG
	return &hbridge_manager_config;
#endif
	return nullptr;
}

bool HBridge::init()
{
	// Configure enable GPIO and PWM system (manager instance only)
	if (is_manager_instance()) {
		const hbridge_manager_config_t *manager_config = get_manager_config();

		if (manager_config != nullptr && manager_config->enable_gpio != 0) {
			px4_arch_configgpio(manager_config->enable_gpio);
			px4_arch_gpiowrite(manager_config->enable_gpio, 0); // Start disabled
			PX4_INFO("Manager instance initialized with enable GPIO: 0x%08lx", manager_config->enable_gpio);

		} else {
			PX4_ERR("No shared enable GPIO configured");
			return false;
		}

		// CRITICAL FIX: Set initialized flag for manager instance
		_initialized = true;
		PX4_INFO("HBridge manager instance initialized");
		return true;
	}

	output_enable(false); // Ensure output is disabled initially

	// Get board configuration for this instance
	_board_config = get_board_config(_instance);

	if (_board_config == nullptr) {
		PX4_ERR("No board configuration for instance %d", _instance);
		return false;
	}

	// Load parameters
	updateParams();

	// Apply board-level direction reverse default (overrides param default at first boot)
	if (_board_config->dir_reverse) {
		if (_instance == 0) {
			_param_dir_reverse_0.set(1);
			_param_dir_reverse_0.commit_no_notification();
		} else if (_instance == 1) {
			_param_dir_reverse_1.set(1);
			_param_dir_reverse_1.commit_no_notification();
		}

		updateParams();
	}

	// Configure hardware
	if (!configure_hardware()) {
		PX4_ERR("Failed to configure hardware for instance %d", _instance);
		return false;
	}

	// Initialize uORB publication
	hbridge_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.instance = _instance;

	int instance_copy = _instance;
	_pub_handle = orb_advertise_multi(ORB_ID(hbridge_status), &status, &instance_copy);

	_initialized = true;

	// Start work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	PX4_INFO("HBridge instance %d (%s) initialized",
		 _instance, _board_config->name);

	output_enable(true);

	return true;
}

bool HBridge::configure_hardware()
{
	if (_board_config == nullptr) {
		return false;
	}

	// Configure direction GPIO
	if (_board_config->dir_gpio != 0) {
		px4_arch_configgpio(_board_config->dir_gpio);
		px4_arch_gpiowrite(_board_config->dir_gpio, 0); // Default to reverse
	}

	// Read-modify-write approach for PWM channel initialization
	// Get current initialized channels and add this channel to the mask
	uint32_t current_channels = _initialized_pwm_channels.load();
	uint32_t this_channel_mask = (1 << _board_config->pwm_ch);
	uint32_t new_channels = current_channels | this_channel_mask;

	// Only call up_motor_pwm_init if this channel isn't already initialized
	if ((current_channels & this_channel_mask) == 0) {
		int ret = up_motor_pwm_init(new_channels);

		if (ret < 0) {
			PX4_ERR("Motor PWM init failed for channel %d: %d", _board_config->pwm_ch, ret);
			return false;
		}

		// Update the initialized channels mask atomically
		_initialized_pwm_channels.store(new_channels);

		up_motor_pwm_set_rate(MOTOR_PWM_FREQ_25KHZ);
		up_motor_pwm_arm(true, new_channels);
		PX4_INFO("Motor PWM channel %d initialized (total channels: 0x%08lx)", _board_config->pwm_ch,
			 (unsigned long)new_channels);

	} else {
		PX4_INFO("Motor PWM channel %d already initialized", _board_config->pwm_ch);
	}

	return true;
}

void HBridge::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	// Manager instance doesn't need to run
	if (_instance == MANAGER_INSTANCE) {
		return;
	}

	perf_begin(_loop_perf);

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Process commands
	process_commands();

	// Process limit sensors
	process_limit_sensors();

	// Apply safety limits
	apply_safety_limits();

	// Apply final output
	output_pwm();

	// Publish status
	publish_status();

	perf_end(_loop_perf);
}

void HBridge::process_commands()
{
	perf_begin(_command_perf);

	// Skip uORB command processing if in manual mode
	if (_manual_mode) {
		perf_end(_command_perf);
		return;
	}

	hbridge_setpoint_s cmd;

	if (_command_sub[_instance].updated() &&
	    _command_sub[_instance].copy(&cmd)) {
		// Process duty cycle command for this instance
		if (cmd.enable) {
			float duty_cycle = math::constrain(cmd.duty_cycle, -1.0f, 1.0f);
			// Store the commanded duty cycle - don't output immediately
			_current_duty_cycle = duty_cycle;

		} else {
			// Disable command - stop the motor
			_current_duty_cycle = 0.0f;
		}
	}

	perf_end(_command_perf);
}

void HBridge::process_limit_sensors()
{
	sensor_limit_switch_s limit_msg;

	// Check forward limit sensor for this instance
	uint8_t forward_limit_id = get_fwd_limit();

	if (forward_limit_id != 255 &&
	    _limit_sensor_sub[forward_limit_id].updated() &&
	    _limit_sensor_sub[forward_limit_id].copy(&limit_msg)) {
		_forward_limit_active = limit_msg.state;
	}

	// Check reverse limit sensor for this instance
	uint8_t reverse_limit_id = get_rev_limit();

	if (reverse_limit_id != 255 &&
	    _limit_sensor_sub[reverse_limit_id].updated() &&
	    _limit_sensor_sub[reverse_limit_id].copy(&limit_msg)) {
		_reverse_limit_active = limit_msg.state;
	}
}

void HBridge::output_pwm()
{
	if (!_initialized || _board_config == nullptr) {
		return;
	}

	// Set direction based on current duty cycle and direction reverse parameter
	bool forward = _current_duty_cycle >= 0.0f;
	set_direction(forward);

	// Calculate PWM value
	float abs_duty = fabsf(_current_duty_cycle);

	if (abs_duty < 0.001f) {
		abs_duty = 0.0f;
	}

	// Convert to PWM range and output
	up_motor_pwm_set_duty_cycle(_board_config->pwm_ch, abs_duty);
}

void HBridge::set_direction(bool forward)
{
	if (_board_config != nullptr && _board_config->dir_gpio != 0) {
		// Apply direction reverse parameter
		bool actual_direction = forward;

		if (get_dir_reverse()) {
			actual_direction = !forward;  // Invert direction if reverse parameter is set
		}

		px4_arch_gpiowrite(_board_config->dir_gpio, actual_direction ? 1 : 0);
	}
}

void HBridge::output_enable(bool enable)
{
	// Only manager instance controls shared enable pin
	if (!is_manager_instance()) {
		return;
	}

	const hbridge_manager_config_t *manager_config = get_manager_config();

	if (manager_config != nullptr && manager_config->enable_gpio != 0) {
		px4_arch_gpiowrite(manager_config->enable_gpio, enable ? 1 : 0);
		PX4_INFO("H-Bridge enable: %s", enable ? "ON" : "OFF");
	}
}

void HBridge::apply_safety_limits()
{
	// Apply forward limit
	if (_forward_limit_active && _current_duty_cycle > 0.0f) {
		PX4_DEBUG("Forward limit active, blocking forward motion");
		_current_duty_cycle = 0.0f;
	}

	// Apply reverse limit
	if (_reverse_limit_active && _current_duty_cycle < 0.0f) {
		PX4_DEBUG("Reverse limit active, blocking reverse motion");
		_current_duty_cycle = 0.0f;
	}
}

void HBridge::publish_status()
{
	hbridge_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.instance = _instance;
	status.duty_cycle = _current_duty_cycle;
	status.forward_limit = _forward_limit_active;
	status.reverse_limit = _reverse_limit_active;
	status.enabled = _initialized;

	// Publish using orb_publish if we have a handle
	if (_pub_handle != nullptr) {
		orb_publish(ORB_ID(hbridge_status), _pub_handle, &status);
	}
}

int HBridge::task_spawn(int argc, char *argv[])
{
	// Parse command line arguments
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	int target_instance = -1; // -1 means start all enabled instances

	while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			target_instance = atoi(myoptarg);

			if (target_instance < 0 || target_instance >= MAX_INSTANCES) {
				PX4_ERR("Invalid instance %d, must be 0-%d", target_instance, MAX_INSTANCES - 1);
				return PX4_ERROR;
			}

			break;

		default:
			return print_usage("unknown option");
		}
	}

	// Create manager instance if needed
	if (_manager_instance == nullptr) {
		HBridge *manager = new HBridge(MANAGER_INSTANCE);

		if (manager == nullptr) {
			PX4_ERR("Failed to allocate manager instance");
			return PX4_ERROR;
		}

		// Store manager as the primary object
		_object.store(manager);
		_task_id = task_id_is_work_queue;

		PX4_INFO("HBridge manager started");
	}

#ifdef BOARD_HAS_HBRIDGE_CONFIG
	bool any_started = false;

	if (target_instance >= 0) {
		// Start specific instance
		any_started = start_instance(target_instance);

	} else {
		// Start all configured instances
		for (int i = 0; i < MAX_INSTANCES; i++) {
			if (start_instance(i)) {
				any_started = true;
			}
		}
	}

	if (any_started) {
		// Set the first instance as the primary object for status
		for (int i = 0; i < MAX_INSTANCES; i++) {
			if (_instances[i] != nullptr) {
				_object.store(_instances[i]);
				_task_id = task_id_is_work_queue;
				break;
			}
		}

		return PX4_OK;

	} else {
		PX4_ERR("No HBridge instances could be started");
		return PX4_ERROR;
	}

#else
	PX4_ERR("No HBridge controllers configured for this board");
	return PX4_ERROR;
#endif
}

bool HBridge::start_instance(int instance)
{
	// Check if already running
	if (_instances[instance] != nullptr) {
		PX4_INFO("Instance %d already running", instance);
		return true;
	}

	// Create new instance
	HBridge *obj = new HBridge(instance);

	if (obj == nullptr) {
		PX4_ERR("Failed to allocate HBridge instance %d", instance);
		return false;
	}

	// Initialize instance
	if (obj->init()) {
		PX4_INFO("Started HBridge instance %d", instance);
		return true;

	} else {
		delete obj;
		return false;
	}
}

void HBridge::stop_all_instances()
{
	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (_instances[i] != nullptr) {
			PX4_INFO("Stopping HBridge instance %d", i);
			delete _instances[i];
			_instances[i] = nullptr;
		}
	}

	_num_instances.store(0);
}

void HBridge::print_instance_status(uint8_t instance)
{
	if (instance >= MAX_INSTANCES) {
		PX4_ERR("Invalid instance %d (max: %d)", instance, MAX_INSTANCES - 1);
		return;
	}

	HBridge *inst = _instances[instance];

	if (inst == nullptr) {
		PX4_INFO("HBridge instance %d: NOT RUNNING", instance);
		return;
	}

	const hbridge_config_t *config = inst->_board_config;

	if (config == nullptr) {
		PX4_ERR("HBridge instance %d: No board configuration", instance);
		return;
	}

	PX4_INFO("HBridge instance %d (%s):", instance, config->name);
	PX4_INFO("  Enabled: %s", config->enabled ? "YES" : "NO");
	PX4_INFO("  PWM Channel: %d", config->pwm_ch);
	PX4_INFO("  Direction GPIO: 0x%08lx", config->dir_gpio);
	PX4_INFO("  Direction Reversed: %s", inst->get_dir_reverse() ? "YES" : "NO");
	PX4_INFO("  Manual Mode: %s", inst->_manual_mode ? "YES" : "NO");
	PX4_INFO("  Current Duty Cycle: %.2f", (double)inst->_current_duty_cycle);
	PX4_INFO("  Forward Limit: %s", inst->_forward_limit_active ? "ACTIVE" : "inactive");
	PX4_INFO("  Reverse Limit: %s", inst->_reverse_limit_active ? "ACTIVE" : "inactive");
	PX4_INFO("  Initialized: %s", inst->_initialized ? "YES" : "NO");
}

int HBridge::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("too few arguments");
	}

	if (!strcmp(argv[0], "manual")) {
		if (argc < 3) {
			PX4_ERR("Usage: hbridge manual <instance> <duty_cycle>");
			return PX4_ERROR;
		}

		int instance = atoi(argv[1]);
		float duty_cycle = atof(argv[2]);

		if (instance < 0 || instance >= MAX_INSTANCES) {
			PX4_ERR("Invalid instance %d, must be 0-%d", instance, MAX_INSTANCES - 1);
			return PX4_ERROR;
		}

		if (duty_cycle < -1.0f || duty_cycle > 1.0f) {
			PX4_ERR("Invalid duty cycle %.3f, must be -1.0 to 1.0", (double)duty_cycle);
			return PX4_ERROR;
		}

		if (_instances[instance] == nullptr) {
			PX4_ERR("Instance %d not running", instance);
			return PX4_ERROR;
		}

		if (!_instances[instance]->_manual_mode) {
			PX4_ERR("Instance %d not in manual mode. Use 'hbridge operate_mode %d manual' first", instance, instance);
			return PX4_ERROR;
		}

		// Auto-enable outputs in manual mode if manager exists
		if (_manager_instance != nullptr) {
			_manager_instance->output_enable(true);
		}

		// Manually set duty cycle and output
		_instances[instance]->_current_duty_cycle = duty_cycle;
		_instances[instance]->output_pwm();

		PX4_INFO("Manual PWM output: Instance %d, Duty cycle %.3f", instance, (double)duty_cycle);
		return PX4_OK;
	}

	if (!strcmp(argv[0], "enable")) {
		if (_manager_instance != nullptr) {
			_manager_instance->output_enable(true);
			PX4_INFO("H-Bridge enabled");
			return PX4_OK;

		} else {
			PX4_ERR("Manager instance not running");
			return PX4_ERROR;
		}
	}

	if (!strcmp(argv[0], "disable")) {
		if (_manager_instance != nullptr) {
			_manager_instance->output_enable(false);
			PX4_INFO("H-Bridge disabled");
			return PX4_OK;

		} else {
			PX4_ERR("Manager instance not running");
			return PX4_ERROR;
		}
	}

	if (!strcmp(argv[0], "operate_mode")) {
		if (argc < 3) {
			PX4_ERR("Usage: hbridge operate_mode <instance> <manual|auto>");
			return PX4_ERROR;
		}

		int instance = atoi(argv[1]);

		if (instance < 0 || instance >= MAX_INSTANCES) {
			PX4_ERR("Invalid instance %d, must be 0-%d", instance, MAX_INSTANCES - 1);
			return PX4_ERROR;
		}

		if (_instances[instance] == nullptr) {
			PX4_ERR("Instance %d not running", instance);
			return PX4_ERROR;
		}

		if (!strcmp(argv[2], "manual")) {
			// Enter manual mode - stop processing uORB commands
			_instances[instance]->_manual_mode = true;
			_instances[instance]->_current_duty_cycle = 0.0f; // Stop motor when entering manual mode
			_instances[instance]->output_pwm();

			// Auto-enable outputs when entering manual mode
			if (_manager_instance != nullptr) {
				_manager_instance->output_enable(true);
				PX4_INFO("H-Bridge outputs enabled for manual mode");
			}

			PX4_INFO("Instance %d entered manual mode", instance);
			return PX4_OK;

		} else if (!strcmp(argv[2], "auto")) {
			// Exit manual mode - resume processing uORB commands
			_instances[instance]->_manual_mode = false;
			_instances[instance]->_current_duty_cycle = 0.0f; // Stop motor when exiting manual mode
			_instances[instance]->output_pwm();
			PX4_INFO("Instance %d entered auto mode", instance);
			return PX4_OK;

		} else {
			PX4_ERR("Invalid mode '%s'. Use 'manual' or 'auto'", argv[2]);
			return PX4_ERROR;
		}
	}

	if (!strcmp(argv[0], "status")) {
		if (argc >= 2) {
			// Print specific instance status
			int instance = atoi(argv[1]);

			if (instance < 0 || instance >= MAX_INSTANCES) {
				PX4_ERR("Invalid instance %d, must be 0-%d", instance, MAX_INSTANCES - 1);
				return PX4_ERROR;
			}

			print_instance_status(static_cast<uint8_t>(instance));

		} else {
			// Print all instance status
			for (int i = 0; i < MAX_INSTANCES; i++) {
				print_instance_status(static_cast<uint8_t>(i));

				if (i < MAX_INSTANCES - 1) { PX4_INFO(""); } // Blank line between instances
			}
		}

		return PX4_OK;
	}

	return print_usage("unknown command");
}

int HBridge::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Multi-instance H-Bridge motor driver for wheel loader operations.

Each instance controls one H-bridge channel with PWM speed control and GPIO
direction control. Instance 0 acts as the manager and controls the shared
enable pin. Each instance subscribes to hbridge_setpoint messages with matching
instance numbers for independent control.

The driver integrates with limit sensors to prevent motion in restricted
directions. Forward limits stop forward motion, reverse limits stop reverse motion.

Direction reversal can be configured per channel using HBRIDGE_DIR_REV0/1 parameters.
PWM output uses direct duty cycle control (0.0 to 1.0 range).

### Implementation
- Supports 2 instances (0 and 1) for dual H-bridge channels
- Instance 0 is the manager and controls shared enable pin
- Each instance subscribes to hbridge_setpoint with same instance number
- Multi-instance status publishing with limit sensor states
- PWM output with configurable direction reversal
- Safety integration with limit sensors

### Examples
Start all H-Bridge instances:
$ hbridge start

Start specific instance:
$ hbridge start -i 0

Configure direction reversal:
$ param set HBRIDGE_DIR_REV0 1  # Reverse direction for channel 0
$ param set HBRIDGE_DIR_REV1 0  # Normal direction for channel 1
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hbridge", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('i', -1, 0, MAX_INSTANCES-1,
				     "Start specific instance (0-1), default: start all", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("operate_mode", "Set operation mode for instance");
	PRINT_MODULE_USAGE_ARG("<instance>", "H-bridge instance (0 or 1)", false);
	PRINT_MODULE_USAGE_ARG("<manual|auto>", "Operation mode: manual or auto", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("manual", "Manual PWM output control (requires manual mode)");
	PRINT_MODULE_USAGE_ARG("<instance>", "H-bridge instance (0 or 1)", false);
	PRINT_MODULE_USAGE_ARG("<duty_cycle>", "Duty cycle (-1.0 to 1.0)", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("enable", "Enable H-bridge power");
	PRINT_MODULE_USAGE_COMMAND_DESCR("disable", "Disable H-bridge power");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print instance status information");
	PRINT_MODULE_USAGE_ARG("[instance]", "Specific instance (0 or 1), default: all", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int HBridge::print_status()
{
	PX4_INFO("HBridge Status:");
	PX4_INFO("  Active instances: %d", _num_instances.load());

	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (_instances[i] != nullptr) {
			PX4_INFO("  Instance %d: %s", i,
				 _instances[i]->_board_config ? _instances[i]->_board_config->name : "unknown");
			PX4_INFO("    Current Speed: %.3f", static_cast<double>(_instances[i]->_current_duty_cycle));
			PX4_INFO("    Direction Reverse: %s", _instances[i]->get_dir_reverse() ? "YES" : "NO");
			PX4_INFO("    Forward Limit: %s", _instances[i]->_forward_limit_active ? "ACTIVE" : "inactive");
			PX4_INFO("    Reverse Limit: %s", _instances[i]->_reverse_limit_active ? "ACTIVE" : "inactive");
			PX4_INFO("    Manager: %s", _instances[i]->is_manager_instance() ? "YES" : "NO");

			perf_print_counter(_instances[i]->_loop_perf);
			perf_print_counter(_instances[i]->_command_perf);
		}
	}

	return 0;
}

void HBridge::updateParams()
{
	// Update module parameters
	ModuleParams::updateParams();

	// Log parameter updates for debugging (only for regular instances, not manager)
	if (!is_manager_instance()) {
		PX4_DEBUG("HBridge instance %d parameter update:", _instance);
		PX4_DEBUG("  Direction reverse: %s", get_dir_reverse() ? "YES" : "NO");
		PX4_DEBUG("  Forward limit sensor: %d", get_fwd_limit());
		PX4_DEBUG("  Reverse limit sensor: %d", get_rev_limit());

		// Validate limit sensor parameters
		int fwd_limit = get_fwd_limit();
		int rev_limit = get_rev_limit();

		if (fwd_limit < 0 || fwd_limit > 255) {
			PX4_WARN("Invalid forward limit sensor ID %d for instance %d (valid: 0-255)", fwd_limit, _instance);
		}

		if (rev_limit < 0 || rev_limit > 255) {
			PX4_WARN("Invalid reverse limit sensor ID %d for instance %d (valid: 0-255)", rev_limit, _instance);
		}
	}
}

extern "C" __EXPORT int hbridge_main(int argc, char *argv[])
{
	return HBridge::main(argc, argv);
}
