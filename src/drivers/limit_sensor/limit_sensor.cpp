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

#include "limit_sensor.hpp"

#include <fcntl.h>
#include <math.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <inttypes.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <lib/mathlib/mathlib.h>
#include <board_config.h>
#include <drivers/drv_hrt.h>
#include <lib/perf/perf_counter.h>
#include <parameters/param.h>
#include <uORB/topics/sensor_limit_switch.h>
#include <uORB/topics/parameter_update.h>

// External declaration of board configuration
#ifdef BOARD_HAS_LIMIT_SENSOR_CONFIG
extern const limit_sensor_config_t g_limit_sensor_config[];
#endif

// Static storage for multiple instances
LimitSensor *LimitSensor::_instances[MAX_INSTANCES] = {};
px4::atomic<uint8_t> LimitSensor::_num_instances{0};
LimitSensor *LimitSensor::_manager_instance = nullptr;

LimitSensor::LimitSensor(uint8_t instance) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_instance(instance),
	_parameter_update_sub(ORB_ID(parameter_update)),
	_cycle_perf(perf_alloc(PC_ELAPSED, instance == MANAGER_INSTANCE ? MODULE_NAME": cycle" :
			       (instance == 0 ? MODULE_NAME"0: cycle" :
				(instance == 1 ? MODULE_NAME"1: cycle" :
				 (instance == 2 ? MODULE_NAME"2: cycle" : MODULE_NAME"3: cycle"))))),
	_sample_perf(perf_alloc(PC_ELAPSED, instance == MANAGER_INSTANCE ? MODULE_NAME": sample" :
				(instance == 0 ? MODULE_NAME"0: sample" :
				 (instance == 1 ? MODULE_NAME"1: sample" :
				  (instance == 2 ? MODULE_NAME"2: sample" : MODULE_NAME"3: sample"))))),
	_fault_perf(perf_alloc(PC_COUNT, instance == MANAGER_INSTANCE ? MODULE_NAME": fault" :
			       (instance == 0 ? MODULE_NAME"0: fault" :
				(instance == 1 ? MODULE_NAME"1: fault" :
				 (instance == 2 ? MODULE_NAME"2: fault" : MODULE_NAME"3: fault")))))
{
	// Initialize switch states
	_switch_1 = {};
	_switch_2 = {};
	_sensor_state = {};
	_run_interval_us = 5000; // Default 5ms (200Hz)

	// Set manager instance if this is the manager
	if (_instance == MANAGER_INSTANCE) {
		_manager_instance = this;
	}

	// Note: Regular instances are registered in start_instance() after successful init
}

LimitSensor::~LimitSensor()
{
	// Stop the work queue
	ScheduleClear();

	// Manager instance cleanup
	if (_instance == MANAGER_INSTANCE) {
		stop_all_sensor_instances();
		_manager_instance = nullptr;
	}

	// Unadvertise publication
	if (_pub_handle != nullptr) {
		orb_unadvertise(_pub_handle);
	}

	// Unregister this instance
	if (_instance < MAX_INSTANCES && _instances[_instance] == this) {
		_instances[_instance] = nullptr;
		_num_instances.fetch_sub(1);
	}

	// Free performance counters
	perf_free(_cycle_perf);
	perf_free(_sample_perf);
	perf_free(_fault_perf);
}

const limit_sensor_config_t *LimitSensor::get_board_config(uint8_t instance)
{
#ifdef BOARD_HAS_LIMIT_SENSOR_CONFIG

	if (instance < BOARD_NUM_LIMIT_SENSORS) {
		return &g_limit_sensor_config[instance];
	}

#endif
	return nullptr;
}

bool LimitSensor::init()
{
	// Manager instance doesn't need initialization
	if (_instance == MANAGER_INSTANCE) {
		return true;
	}

	// Get board configuration for this instance
	_board_config = get_board_config(_instance);

	if (_board_config == nullptr) {
		PX4_ERR("No board configuration for instance %d", _instance);
		return false;
	}

	// Load runtime parameters
	updateParams();

	// Check if this instance is enabled
	if (!is_instance_enabled()) {
		PX4_INFO("Limit sensor instance %d is disabled via parameter", _instance);
		return false;
	}

	// Configure switches
	if (!configure_switches()) {
		return false;
	}

	// Initialize uORB publication
	if (!init_publication()) {
		return false;
	}

	// Start work queue
	ScheduleOnInterval(_run_interval_us);

	PX4_INFO("LimitSensor %d initialized: %s", _instance, _board_config->name);
	return true;
}

bool LimitSensor::is_instance_enabled() const
{
	switch (_instance) {
	case 0: return _param_ls0_enable.get();

	case 1: return _param_ls1_enable.get();

	case 2: return _param_ls2_enable.get();

	case 3: return _param_ls3_enable.get();

	default: return false;
	}
}

int LimitSensor::get_instance_function() const
{
	switch (_instance) {
	case 0: return _param_ls0_function.get();

	case 1: return _param_ls1_function.get();

	case 2: return _param_ls2_function.get();

	case 3: return _param_ls3_function.get();

	default: return -1;
	}
}

bool LimitSensor::configure_switches()
{
	// Configure primary switch
	if (_board_config->gpio_pin_1 != 0) {
		if (!configure_switch(_switch_1, _board_config->gpio_pin_1, _board_config->inverted)) {
			PX4_ERR("Failed to configure switch 1 for %s", _board_config->name);
			return false;
		}
	}

	// Configure secondary switch if redundancy is enabled
	if (_board_config->gpio_pin_2 != 0 && _board_config->redundancy_enabled) {
		if (!configure_switch(_switch_2, _board_config->gpio_pin_2, _board_config->inverted)) {
			PX4_ERR("Failed to configure switch 2 for %s", _board_config->name);
			return false;
		}
	}

	return true;
}

bool LimitSensor::init_publication()
{
	sensor_limit_switch_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.instance = _instance;
	msg.function = _board_config->function;

	int instance_copy = _instance;
	_pub_handle = orb_advertise_multi(ORB_ID(sensor_limit_switch), &msg, &instance_copy);

	if (_pub_handle == nullptr) {
		PX4_ERR("Failed to advertise sensor_limit_switch");
		return false;
	}

	return true;
}

bool LimitSensor::configure_switch(SwitchState &switch_state, uint32_t pin, bool inverted)
{
	switch_state.pin = pin;
	switch_state.inverted = inverted;

	// Configure GPIO pin
#if defined(__PX4_NUTTX)

	if (px4_arch_configgpio(pin) < 0) {
		PX4_ERR("Failed to configure GPIO pin 0x%08lx", pin);
		return false;
	}

#endif

	switch_state.configured = true;
	switch_state.current_state = false;
	switch_state.last_state = false;
	switch_state.last_change_time = 0;
	switch_state.debounce_count = 0;

	return true;
}

bool LimitSensor::read_switch_state(SwitchState &switch_state)
{
	if (!switch_state.configured) {
		return false;
	}

	// Read GPIO pin
	bool pin_state = false;
#if defined(__PX4_NUTTX)
	pin_state = px4_arch_gpioread(switch_state.pin);
#elif defined(__PX4_POSIX)
	// POSIX simulation - always return false for now
	pin_state = false;
#endif

	// Apply inversion if configured
	if (switch_state.inverted) {
		pin_state = !pin_state;
	}

	switch_state.current_state = pin_state;
	return true;
}

bool LimitSensor::debounce_switch(SwitchState &switch_state)
{
	if (!switch_state.configured) {
		return false;
	}

	uint64_t now = hrt_absolute_time();

	// Check if state changed
	if (switch_state.current_state != switch_state.last_state) {
		switch_state.last_change_time = now;
		switch_state.debounce_count = 1;
		switch_state.last_state = switch_state.current_state;
		return false; // State is not stable yet
	}

	// Check if enough time has passed for debouncing
	if ((now - switch_state.last_change_time) >= _debounce_time_us) {
		if (switch_state.debounce_count < DEBOUNCE_COUNTS) {
			switch_state.debounce_count++;
			return false; // Need more consistent reads
		}

		return true; // State is stable
	}

	return false; // Still debouncing
}

void LimitSensor::update_combined_state()
{
	bool new_state = false;

	if (_board_config->redundancy_enabled && _switch_2.configured) {
		// For redundant sensors (bucket load/dump), require both switches to agree
		new_state = _switch_1.current_state && _switch_2.current_state;

		// Check for redundancy fault (switches disagree)
		if (_switch_1.current_state != _switch_2.current_state) {
			if (!_sensor_state.redundancy_fault) {
				_sensor_state.redundancy_fault = true;
				perf_count(_fault_perf);
				PX4_WARN("%s: Redundancy fault detected", _board_config->name);
			}

		} else {
			_sensor_state.redundancy_fault = false;
		}

	} else {
		// For non-redundant sensors (boom, steering), use only switch 1
		new_state = _switch_1.current_state;
		_sensor_state.redundancy_fault = false;
	}

	// Update combined state
	_sensor_state.combined_state = new_state;

	// Track state changes
	if (new_state && !_sensor_state.last_combined_state) {
		_sensor_state.activation_count++;
		_sensor_state.last_activation_time = hrt_absolute_time();
	}

	_sensor_state.last_combined_state = new_state;
}

void LimitSensor::stop_all_sensor_instances()
{
	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (_instances[i] != nullptr) {
			PX4_INFO("Stopping limit sensor instance %d", i);
			delete _instances[i];
			_instances[i] = nullptr;
		}
	}

	_num_instances.store(0);
}

const char *LimitSensor::get_function_name(LimitFunction func)
{
	switch (func) {
	case BUCKET_LOAD: return "Bucket Load";

	case BUCKET_DUMP: return "Bucket Dump";

	case BOOM_UP: return "Boom Up";

	case BOOM_DOWN: return "Boom Down";

	case STEERING_LEFT: return "Steering Left";

	case STEERING_RIGHT: return "Steering Right";

	default: return "Unknown";
	}
}

void LimitSensor::publish_state()
{
	if (_pub_handle == nullptr) {
		return;
	}

	sensor_limit_switch_s msg{};
	msg.timestamp = hrt_absolute_time();
	msg.instance = _instance;
	msg.function = _board_config->function;
	msg.state = _sensor_state.combined_state;
	msg.switch_1_state = _switch_1.current_state;
	msg.switch_2_state = _switch_2.configured ? _switch_2.current_state : false;
	msg.redundancy_enabled = _board_config->redundancy_enabled;
	msg.redundancy_fault = _sensor_state.redundancy_fault;
	msg.activation_count = _sensor_state.activation_count;
	msg.last_activation_time = _sensor_state.last_activation_time;

	orb_publish(ORB_ID(sensor_limit_switch), _pub_handle, &msg);
}

void LimitSensor::updateParams()
{
	// Call parent updateParams
	ModuleParams::updateParams();

	// Apply poll rate parameter
	int32_t poll_rate = _param_poll_rate.get();

	if (poll_rate > 0 && poll_rate <= 1000) {
		uint32_t new_interval = 1000000 / poll_rate;

		// Only reschedule if the interval actually changed
		if (new_interval != _run_interval_us) {
			_run_interval_us = new_interval;

			// Update schedule if already running
			if (_board_config != nullptr) {
				ScheduleOnInterval(_run_interval_us);
			}
		}
	}

	// Apply debounce time parameter
	int32_t debounce_us = _param_debounce_us.get();

	if (debounce_us >= 1000 && debounce_us <= 100000) {
		_debounce_time_us = debounce_us;
	}

	// Log parameter updates if diagnostics enabled
	if (_param_diag_enable.get() && _instance != MANAGER_INSTANCE) {
		PX4_INFO("LimitSensor %d params: poll_rate=%ld Hz, debounce=%ld us",
			 _instance, poll_rate, debounce_us);
	}
}

void LimitSensor::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	// Manager instance doesn't need to run
	if (_instance == MANAGER_INSTANCE) {
		return;
	}

	perf_begin(_cycle_perf);

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	perf_begin(_sample_perf);

	// Read switch states
	read_switch_state(_switch_1);

	if (_switch_2.configured) {
		read_switch_state(_switch_2);
	}

	// Debounce switches
	bool switch_1_stable = debounce_switch(_switch_1);
	bool switch_2_stable = _switch_2.configured ? debounce_switch(_switch_2) : true;

	// Update combined state only when switches are stable
	if (switch_1_stable && switch_2_stable) {
		update_combined_state();
	}

	// Publish only on state change or at heartbeat interval
	uint64_t now = hrt_absolute_time();
	bool state_changed = (_sensor_state.combined_state != _last_published_state);
	bool heartbeat_due = (now - _last_publish_time) >= HEARTBEAT_PUBLISH_INTERVAL_US;

	if (state_changed || heartbeat_due) {
		publish_state();
		_last_published_state = _sensor_state.combined_state;
		_last_publish_time = now;
	}

	perf_end(_sample_perf);
	perf_end(_cycle_perf);
}

int LimitSensor::print_status()
{
	// Manager instance shows overview
	if (_instance == MANAGER_INSTANCE) {
		PX4_INFO("Limit Sensor Manager Status");
		PX4_INFO("  Active instances: %d", _num_instances.load());

		for (int i = 0; i < MAX_INSTANCES; i++) {
			if (_instances[i] != nullptr) {
				_instances[i]->print_status();
			}
		}

		return 0;
	}

	// Regular instance status
	if (_board_config == nullptr) {
		PX4_INFO("Instance %d: Not configured", _instance);
		return 0;
	}

	PX4_INFO("");
	PX4_INFO("Instance %d: %s [%s]", _instance, _board_config->name,
		 get_function_name(static_cast<LimitFunction>(_board_config->function)));
	PX4_INFO("  Enabled: %s", is_instance_enabled() ? "YES" : "NO");
	PX4_INFO("  Poll rate: %ld Hz", _param_poll_rate.get());
	PX4_INFO("  Debounce: %ld us", _param_debounce_us.get());
	PX4_INFO("  Diagnostics: %s", _param_diag_enable.get() ? "ENABLED" : "DISABLED");

	if (_switch_1.configured) {
		PX4_INFO("  Switch 1: %s (pin: 0x%08lx)",
			 _switch_1.current_state ? "ACTIVE" : "INACTIVE", _switch_1.pin);
	}

	if (_switch_2.configured) {
		PX4_INFO("  Switch 2: %s (pin: 0x%08lx)",
			 _switch_2.current_state ? "ACTIVE" : "INACTIVE", _switch_2.pin);
		PX4_INFO("  Redundancy: %s",
			 _sensor_state.redundancy_fault ? "FAULT" : "OK");
	}

	PX4_INFO("  Combined: %s", _sensor_state.combined_state ? "ACTIVE" : "INACTIVE");
	PX4_INFO("  Activations: %" PRIu32, _sensor_state.activation_count);

	perf_print_counter(_cycle_perf);
	perf_print_counter(_sample_perf);
	perf_print_counter(_fault_perf);

	return 0;
}

int LimitSensor::task_spawn(int argc, char *argv[])
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
		LimitSensor *manager = new LimitSensor(MANAGER_INSTANCE);

		if (manager == nullptr) {
			PX4_ERR("Failed to allocate manager instance");
			return PX4_ERROR;
		}

		// Store manager as the primary object
		_object.store(manager);
		_task_id = task_id_is_work_queue;

		PX4_INFO("Limit sensor manager started");
	}

#ifdef BOARD_HAS_LIMIT_SENSOR_CONFIG
	bool any_started = false;

	if (target_instance >= 0) {
		// Start specific instance
		any_started = start_instance(target_instance);

	} else {
		// Start all enabled instances
		for (int i = 0; i < math::min(MAX_INSTANCES, BOARD_NUM_LIMIT_SENSORS); i++) {
			if (start_instance(i)) {
				any_started = true;
			}
		}
	}

	if (any_started || _manager_instance != nullptr) {
		return PX4_OK;

	} else {
		PX4_ERR("No limit sensor instances could be started");
		return PX4_ERROR;
	}

#else
	PX4_ERR("No limit sensors configured for this board");
	return PX4_ERROR;
#endif
}

bool LimitSensor::start_instance(int instance)
{
	// Check if already running
	if (_instances[instance] != nullptr) {
		PX4_INFO("Instance %d already running", instance);
		return true;
	}

	// Create new instance
	LimitSensor *obj = new LimitSensor(instance);

	if (obj == nullptr) {
		PX4_ERR("Failed to allocate instance %d", instance);
		return false;
	}

	// Initialize instance
	if (obj->init()) {
		// Instance initialization successful - register it
		if (instance < MAX_INSTANCES) {
			_instances[instance] = obj;
			_num_instances.fetch_add(1);
		}

		PX4_INFO("Started limit sensor instance %d", instance);
		return true;

	} else {
		delete obj;
		return false;
	}
}

int LimitSensor::custom_command(int argc, char *argv[])
{
	const char *command = argv[0];

	if (!strcmp(command, "stop_instance")) {
		if (argc < 2) {
			PX4_ERR("Missing instance number");
			return PX4_ERROR;
		}

		int instance = atoi(argv[1]);

		if (instance < 0 || instance >= MAX_INSTANCES) {
			PX4_ERR("Invalid instance %d, must be 0-%d", instance, MAX_INSTANCES - 1);
			return PX4_ERROR;
		}

		if (_instances[instance] != nullptr) {
			PX4_INFO("Stopping limit sensor instance %d", instance);
			delete _instances[instance];
			PX4_INFO("Limit sensor instance %d stopped", instance);

		} else {
			PX4_INFO("Instance %d is not running", instance);
		}

		return PX4_OK;
	}

	return print_usage("unknown command");
}

int LimitSensor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Limit sensor driver for wheel loader operations.

The limit sensor driver monitors GPIO pins for limit switch states and publishes
them via uORB. It supports both single and redundant switch configurations.

### Implementation
The driver uses a manager instance pattern where a special manager instance
handles the lifecycle of sensor instances. Each sensor instance monitors
specific limit switches based on board configuration.

Features:
- Debounced switch inputs
- Redundant switch support with fault detection
- Configurable polling rate
- Per-instance enable/disable control
- Activation counting and timing

### Examples
Start all configured limit sensor instances:
$ limit_sensor start

Start a specific limit sensor instance:
$ limit_sensor start -i 0

Stop a specific limit sensor instance:
$ limit_sensor stop_instance 0

Show status of all instances:
$ limit_sensor status
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("limit_sensor", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_INT('i', -1, 0, MAX_INSTANCES-1,
                                 "Start specific instance (0-3), default: start all enabled", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop_instance", "Stop a specific instance");
    PRINT_MODULE_USAGE_ARG("instance", "Instance number to stop (0-3)", false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int limit_sensor_main(int argc, char *argv[])
{
    return LimitSensor::main(argc, argv);
}
