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

#include "quadrature_encoder.hpp"

#include <fcntl.h>
#include <math.h>
#include <cmath>
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
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/sensor_quad_encoder_reset.h>
#include <uORB/topics/parameter_update.h>
#include <px4_arch/quad_encoder.h>
#include <px4_arch/board_encoder.h>

// Static storage for multiple instances
QuadratureEncoder *QuadratureEncoder::_instances[MAX_INSTANCES] = {};
px4::atomic<uint8_t> QuadratureEncoder::_num_instances{0};
QuadratureEncoder *QuadratureEncoder::_manager_instance = nullptr;

QuadratureEncoder::QuadratureEncoder(uint8_t instance) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _instance(instance),
    _parameter_update_sub(ORB_ID(parameter_update)),
    _cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
    _sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": sample")),
    _fault_perf(perf_alloc(PC_COUNT, MODULE_NAME": fault"))
{
    // Initialize encoder state
    _encoder_state = {};
    _run_interval_us = 10000; // Default 10ms (100Hz)

    // Set manager instance if this is the manager
    if (_instance == MANAGER_INSTANCE) {
        _manager_instance = this;
    }
    // Note: Regular instances are registered in start_instance() after successful init
}

QuadratureEncoder::~QuadratureEncoder()
{
    // Stop the work queue
    ScheduleClear();

    // Manager instance cleanup
    if (_instance == MANAGER_INSTANCE) {
        stop_all_encoder_instances();
        _manager_instance = nullptr;
    }

    // Unadvertise publication
    if (_pub_handle != nullptr) {
        orb_unadvertise(_pub_handle);
    }

    // Stop platform encoder if this is not the manager instance
    if (_instance != MANAGER_INSTANCE) {
        quad_encoder_stop(_instance);
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

const quad_encoder_config_t* QuadratureEncoder::get_board_config(uint8_t instance)
{
#ifdef BOARD_HAS_QUAD_ENCODER_CONFIG
    if (instance < BOARD_NUM_QUAD_ENCODERS) {
        return board_get_encoder_config(instance);
    }
#endif
    return nullptr;
}

bool QuadratureEncoder::init()
{
    // Manager instance doesn't need initialization
    if (_instance == MANAGER_INSTANCE) {
        return true;
    }

    // Get board configuration for this instance
    _board_config = get_board_config(_instance);
    if (_board_config == nullptr) {
        PX4_ERR("No board configuration for encoder instance %d", _instance);
        return false;
    }

    // Set up timing configuration
    uint32_t poll_rate = static_cast<uint32_t>(math::constrain(_param_poll_rate.get(), static_cast<int32_t>(10), static_cast<int32_t>(1000)));
    _run_interval_us = 1000000 / poll_rate;

    // Check if this instance is enabled
    if (!is_instance_enabled()) {
        PX4_INFO("Encoder instance %d is disabled via parameter", _instance);
        return false;
    }

    // Configure encoder
    if (!configure_encoder()) {
        return false;
    }

    // Initialize uORB publication
    if (!init_publication()) {
        return false;
    }

    // Start work queue
    ScheduleOnInterval(_run_interval_us);

    PX4_INFO("QuadratureEncoder %d initialized: %s", _instance, board_get_encoder_name(_instance));
    return true;
}

// === Parameter Accessors ===

bool QuadratureEncoder::is_instance_enabled() const
{
    switch (_instance) {
    case 0: return _param_qe0_enable.get();
    case 1: return _param_qe1_enable.get();
    default: return false;
    }
}

int QuadratureEncoder::get_instance_overflow() const
{
    switch (_instance) {
    case 0: return _param_qe0_overflow.get();
    case 1: return _param_qe1_overflow.get();
    default: return 0;
    }
}

float QuadratureEncoder::get_instance_resolution() const
{
    switch (_instance) {
    case 0: return _param_qe0_resolution.get();
    case 1: return _param_qe1_resolution.get();
    default: return 1.0f;
    }
}

bool QuadratureEncoder::get_instance_reverse() const
{
    switch (_instance) {
    case 0: return _param_qe0_reverse.get();
    case 1: return _param_qe1_reverse.get();
    default: return false;
    }
}

bool QuadratureEncoder::configure_encoder()
{
    // Initialize the platform encoder
    if (quad_encoder_init() < 0) {
        PX4_ERR("Failed to initialize quad encoder platform");
        return false;
    }

    // Configure encoder overflow count from parameters
    int overflow_count = get_instance_overflow();
    uint16_t overflow_value = static_cast<uint16_t>(math::constrain(overflow_count, 0, 65535));

    if (quad_encoder_set_overflow_count(_instance, overflow_value) < 0) {
        PX4_ERR("Failed to set encoder overflow count for instance %d", _instance);
        return false;
    }

    // Start the platform encoder
    if (quad_encoder_start(_instance) < 0) {
        PX4_ERR("Failed to start encoder instance %d", _instance);
        return false;
    }

    PX4_INFO("Encoder %d configured successfully", _instance);
    return true;
}

bool QuadratureEncoder::init_publication()
{
    sensor_quad_encoder_s msg{};
    msg.timestamp = hrt_absolute_time();
    msg.instance = _instance;
    msg.position = 0.0;
    msg.velocity = 0.0;

    int instance_copy = _instance;
    _pub_handle = orb_advertise_multi(ORB_ID(sensor_quad_encoder), &msg, &instance_copy);

    if (_pub_handle == nullptr) {
        PX4_ERR("Failed to advertise sensor_quad_encoder for instance %d", _instance);
        return false;
    }

    return true;
}

bool QuadratureEncoder::read_encoder_state()
{
    encoder_raw_data_t raw_data;

    // Get raw hardware data
    if (!quad_encoder_get_raw_data(_instance, &raw_data)) {
        perf_count(_fault_perf);
        return false; // No new data available
    }

    // Process counter changes
    int32_t current_counter = (int32_t)raw_data.counter;

    if (_encoder_state.last_update_time != 0) { // Skip first reading
        int32_t delta_counter = calculate_delta_counter(current_counter);
        double delta_position = apply_resolution_and_direction(delta_counter);
        update_position_and_velocity(delta_position, raw_data.timestamp);
    }

    // Update state for next iteration
    _encoder_state.last_counter = current_counter;
    _encoder_state.last_update_time = raw_data.timestamp;

    return true;
}

// === Private Calculation Helpers ===

int32_t QuadratureEncoder::calculate_delta_counter(int32_t current_counter)
{
    int32_t delta_counter = current_counter - _encoder_state.last_counter;
    int32_t overflow_count = get_instance_overflow();

    // Handle counter overflow/wraparound
    if (overflow_count > 0) {
        // If delta is more than half the overflow count, assume wraparound
        if (delta_counter > overflow_count / 2) {
            delta_counter -= overflow_count;
        } else if (delta_counter < -overflow_count / 2) {
            delta_counter += overflow_count;
        }
    }

    return delta_counter;
}

double QuadratureEncoder::apply_resolution_and_direction(int32_t delta_counter)
{
    // Convert pulses to real-world units
    float resolution = get_instance_resolution();
    double delta_position = delta_counter * resolution;

    // Apply direction reverse if configured
    if (get_instance_reverse()) {
        delta_position = -delta_position;
    }

    return delta_position;
}

void QuadratureEncoder::update_position_and_velocity(double delta_position, uint64_t current_time)
{
    // Update position
    _encoder_state.position += delta_position;

    // Handle position overflow (prevent infinite accumulation)
    static constexpr double MAX_POSITION = 10000000.0;
    if (fabs(_encoder_state.position) > MAX_POSITION) {
        _encoder_state.position = 0.0;
        PX4_INFO("Encoder %d position reset due to overflow", _instance);
    }

    // Calculate velocity
    if (current_time > _encoder_state.last_update_time) {
        double dt = (current_time - _encoder_state.last_update_time) * 1e-6; // Convert to seconds
        _encoder_state.velocity = delta_position / dt;
    }
}

void QuadratureEncoder::publish_encoder_data()
{
    sensor_quad_encoder_s msg{};

    msg.timestamp = hrt_absolute_time();
    msg.instance = _instance;
    msg.position = _encoder_state.position;
    msg.velocity = _encoder_state.velocity;

    if (_pub_handle != nullptr) {
        orb_publish(ORB_ID(sensor_quad_encoder), _pub_handle, &msg);
    }
}

void QuadratureEncoder::reset_position()
{
    _encoder_state.position = 0.0;
    _encoder_state.velocity = 0.0;
    PX4_INFO("Encoder %d position reset to zero", _instance);
}

void QuadratureEncoder::handle_reset_events()
{
    // Check for reset events (only regular instances handle resets)
    if (_instance != MANAGER_INSTANCE && _reset_sub.updated()) {
        sensor_quad_encoder_reset_s reset_msg;
        if (_reset_sub.copy(&reset_msg) && reset_msg.instance == _instance) {
            reset_position();
            PX4_INFO("Encoder %d position reset via uORB event", _instance);
        }
    }
}

void QuadratureEncoder::handle_parameter_updates()
{
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        ModuleParams::updateParams();
        update_encoder_configuration();
    }
}

void QuadratureEncoder::update_encoder_configuration()
{
    // Update hardware overflow count
    int overflow_count = get_instance_overflow();
    uint16_t overflow_value = static_cast<uint16_t>(math::constrain(overflow_count, 0, 65535));

    if (quad_encoder_set_overflow_count(_instance, overflow_value) < 0) {
        PX4_ERR("Failed to update encoder overflow count for instance %d", _instance);
        perf_count(_fault_perf);
    }
}

void QuadratureEncoder::Run()
{
    // Early exit conditions
    if (should_exit()) {
        ScheduleClear();
        return;
    }

    // Manager instance doesn't process encoder data
    if (_instance == MANAGER_INSTANCE) {
        return;
    }

    // Performance monitoring
    perf_begin(_cycle_perf);

    // Handle system events
    handle_parameter_updates();
    handle_reset_events();

    // Core data processing
    perf_begin(_sample_perf);
    if (read_encoder_state()) {
        publish_encoder_data();
    }
    perf_end(_sample_perf);

    perf_end(_cycle_perf);
}

// === Private Helper Methods ===

void QuadratureEncoder::stop_all_encoder_instances()
{
    for (int i = 0; i < MAX_INSTANCES; i++) {
        if (_instances[i] != nullptr) {
            PX4_INFO("Stopping encoder instance %d", i);
            delete _instances[i];
            _instances[i] = nullptr;
        }
    }
    _num_instances.store(0);
}

int QuadratureEncoder::print_status()
{
    // Manager instance shows overview
    if (_instance == MANAGER_INSTANCE) {
        PX4_INFO("Quadrature Encoder Manager Status");
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
    PX4_INFO("Instance %d: %s", _instance, board_get_encoder_name(_instance));
    PX4_INFO("  Enabled: %s", is_instance_enabled() ? "YES" : "NO");
    PX4_INFO("  Poll rate: %ld Hz", _param_poll_rate.get());
    PX4_INFO("  Overflow: %d", get_instance_overflow());
    PX4_INFO("  Resolution: %.6f units/pulse", static_cast<double>(get_instance_resolution()));
    PX4_INFO("  Reverse: %s", get_instance_reverse() ? "YES" : "NO");
    PX4_INFO("  Position: %.3f", _encoder_state.position);
    PX4_INFO("  Velocity: %.3f", _encoder_state.velocity);

    perf_print_counter(_cycle_perf);
    perf_print_counter(_sample_perf);
    perf_print_counter(_fault_perf);

    return 0;
}

int QuadratureEncoder::task_spawn(int argc, char *argv[])
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
        QuadratureEncoder *manager = new QuadratureEncoder(MANAGER_INSTANCE);
        if (manager == nullptr) {
            PX4_ERR("Failed to allocate manager instance");
            return PX4_ERROR;
        }

        // Store manager as the primary object
        _object.store(manager);
        _task_id = task_id_is_work_queue;

        PX4_INFO("Quadrature encoder manager started");
    }

#ifdef BOARD_HAS_QUAD_ENCODER_CONFIG
    bool any_started = false;

    if (target_instance >= 0) {
        // Start specific instance
        any_started = start_instance(target_instance);
    } else {
        // Start all enabled instances
        for (int i = 0; i < math::min(static_cast<int>(MAX_INSTANCES),
                                      static_cast<int>(BOARD_NUM_QUAD_ENCODERS)); i++) {
            if (start_instance(i)) {
                any_started = true;
            }
        }
    }

    if (any_started || _manager_instance != nullptr) {
        return PX4_OK;
    } else {
        PX4_ERR("No encoder instances could be started");
        return PX4_ERROR;
    }
#else
    PX4_ERR("No encoders configured for this board");
    return PX4_ERROR;
#endif
}

bool QuadratureEncoder::start_instance(int instance)
{
    // Check if already running
    if (_instances[instance] != nullptr) {
        PX4_INFO("Instance %d already running", instance);
        return true;
    }

    // Create new instance
    QuadratureEncoder *obj = new QuadratureEncoder(instance);
    if (obj == nullptr) {
        PX4_ERR("Failed to allocate instance %d", instance);
        return false;
    }

    // Initialize instance
    if (obj->init()) {
        // Register instance after successful initialization
        _instances[instance] = obj;
        _num_instances.fetch_add(1);
        PX4_INFO("Started encoder instance %d", instance);
        return true;
    } else {
        delete obj;
        return false;
    }
}

int QuadratureEncoder::custom_command(int argc, char *argv[])
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
            PX4_INFO("Stopping encoder instance %d", instance);
            delete _instances[instance];
            PX4_INFO("Encoder instance %d stopped", instance);
        } else {
            PX4_INFO("Instance %d is not running", instance);
        }

        return PX4_OK;
    }

    if (!strcmp(command, "set_overflow")) {
        if (argc < 3) {
            PX4_ERR("Usage: quadrature_encoder set_overflow <instance> <overflow_count>");
            return PX4_ERROR;
        }

        int instance = atoi(argv[1]);
        int overflow_count = atoi(argv[2]);

        if (instance < 0 || instance >= MAX_INSTANCES) {
            PX4_ERR("Invalid instance %d, must be 0-%d", instance, MAX_INSTANCES - 1);
            return PX4_ERROR;
        }

        if (overflow_count < 0 || overflow_count > 65535) {
            PX4_ERR("Invalid overflow count %d, must be 0-65535", overflow_count);
            return PX4_ERROR;
        }

        if (quad_encoder_set_overflow_count(instance, (uint16_t)overflow_count) < 0) {
            PX4_ERR("Failed to set overflow count for instance %d", instance);
            return PX4_ERROR;
        }

        PX4_INFO("Set encoder %d overflow count to %d", instance, overflow_count);
        return PX4_OK;
    }

    if (!strcmp(command, "get_overflow")) {
        if (argc < 2) {
            PX4_ERR("Usage: quadrature_encoder get_overflow <instance>");
            return PX4_ERROR;
        }

        int instance = atoi(argv[1]);
        if (instance < 0 || instance >= MAX_INSTANCES) {
            PX4_ERR("Invalid instance %d, must be 0-%d", instance, MAX_INSTANCES - 1);
            return PX4_ERROR;
        }

        uint16_t overflow_count;
        if (quad_encoder_get_overflow_count(instance, &overflow_count) < 0) {
            PX4_ERR("Failed to get overflow count for instance %d", instance);
            return PX4_ERROR;
        }

        PX4_INFO("Encoder %d overflow count: %d", instance, overflow_count);
        return PX4_OK;
    }

    return print_usage("unknown command");
}

int QuadratureEncoder::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Quadrature encoder driver for wheel loader operations.

The quadrature encoder driver monitors GPIO pins for encoder signals and publishes
position and velocity data via uORB. It supports both rotary and linear encoders.

### Implementation
The driver uses a manager instance pattern where a special manager instance
handles the lifecycle of encoder instances. Each encoder instance monitors
specific quadrature encoder channels based on board configuration.

Features:
- Quadrature decoding with direction detection
- Position and velocity calculation
- Support for rotary and linear encoders
- Configurable polling rate
- Per-instance enable/disable control
- Direction reverse capability
- Event-based position reset functionality

### Examples
Start all configured encoder instances:
$ quadrature_encoder start

Start a specific encoder instance:
$ quadrature_encoder start -i 0

Stop a specific encoder instance:
$ quadrature_encoder stop_instance 0

Set encoder resolution at runtime:
$ quadrature_encoder set_resolution 0 1024

Get current encoder resolution:
$ quadrature_encoder get_resolution 0

Show status of all instances:
$ quadrature_encoder status

Reset encoder position via uORB message:
$ uorb top quad_encoder_reset    # See reset events
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("quadrature_encoder", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_INT('i', -1, 0, MAX_INSTANCES-1,
                                 "Start specific instance (0-7), default: start all enabled", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop_instance", "Stop a specific instance");
    PRINT_MODULE_USAGE_ARG("instance", "Instance number to stop (0-7)", false);
    PRINT_MODULE_USAGE_COMMAND_DESCR("set_resolution", "Set encoder resolution");
    PRINT_MODULE_USAGE_ARG("instance", "Instance number (0-7)", false);
    PRINT_MODULE_USAGE_ARG("pulses_per_revolution", "Pulses per revolution (0-65535)", false);
    PRINT_MODULE_USAGE_COMMAND_DESCR("get_resolution", "Get encoder resolution");
    PRINT_MODULE_USAGE_ARG("instance", "Instance number (0-7)", false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int quadrature_encoder_main(int argc, char *argv[])
{
    return QuadratureEncoder::main(argc, argv);
}
