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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/sensor_quad_encoder_reset.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/Subscription.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/atomic.h>
#include <px4_arch/quad_encoder.h>

using namespace time_literals;

// Module configuration
static constexpr uint8_t MAX_INSTANCES = 2;
static constexpr uint8_t MANAGER_INSTANCE = 255; // Special instance for lifecycle management

/**
 * @brief Multi-instance quadrature encoder driver
 *
 * This driver follows the PX4 multi-instance pattern where a manager instance
 * handles the lifecycle of encoder instances. Each encoder instance monitors
 * a specific quadrature encoder based on board configuration.
 */
class QuadratureEncoder : public ModuleBase<QuadratureEncoder>,
                          public ModuleParams,
                          public px4::ScheduledWorkItem
{
public:
    /**
     * @brief Constructor
     * @param instance Instance number (0-MAX_INSTANCES-1), or MANAGER_INSTANCE for manager
     */
    QuadratureEncoder(uint8_t instance = MANAGER_INSTANCE);

    /**
     * @brief Destructor
     */
    ~QuadratureEncoder();

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);

    /** @see ModuleBase */
    static int custom_command(int argc, char *argv[]);

    /** @see ModuleBase */
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::print_status() */
    int print_status() override;

    /**
     * @brief Initialize the encoder instance
     */
    bool init();

    /** @see ScheduledWorkItem::Run() */
    void Run() override;

private:
    // === Instance Management ===
    uint8_t _instance{MANAGER_INSTANCE};

    // Static storage for multiple instances (following PX4 multi-instance pattern)
    static QuadratureEncoder *_instances[MAX_INSTANCES];
    static px4::atomic<uint8_t> _num_instances;
    static QuadratureEncoder *_manager_instance;

    // === Hardware Configuration ===
    const quad_encoder_config_t *_board_config{nullptr};

    // === Encoder State ===
    struct EncoderState {
        int32_t last_counter{0};          // Previous raw counter value for delta calculation
        double position{0.0};             // Accumulated position in configured units
        double velocity{0.0};             // Instantaneous velocity in configured units/sec
        uint64_t last_update_time{0};     // Last successful update timestamp
    } _encoder_state;

    // === uORB Communication ===
    orb_advert_t _pub_handle{nullptr};                                    // Multi-instance publication handle
    uORB::Subscription _reset_sub{ORB_ID(sensor_quad_encoder_reset)};           // Position reset events
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};  // Parameter update notifications

    // === Performance Monitoring ===
    perf_counter_t _cycle_perf;           // Main loop performance
    perf_counter_t _sample_perf;          // Sampling performance
    perf_counter_t _fault_perf;           // Error/fault tracking

    // === Timing Control ===
    uint32_t _run_interval_us{10000};     // Polling interval (default 100Hz)

    // === Instance Lifecycle ===
    static bool start_instance(int instance);
    void stop_all_encoder_instances();

    // === Configuration ===
    bool configure_encoder();
    void update_encoder_configuration();
    const quad_encoder_config_t* get_board_config(uint8_t instance);

    // === Data Processing ===
    bool read_encoder_state();
    void publish_encoder_data();
    bool init_publication();

    // === Calculation Helpers ===
    int32_t calculate_delta_counter(int32_t current_counter);
    double apply_resolution_and_direction(int32_t delta_counter);
    void update_position_and_velocity(double delta_position, uint64_t current_time);

    // === Event Handlers ===
    void reset_position();
    void handle_reset_events();
    void handle_parameter_updates();

    // === Parameter Accessors ===
    bool is_instance_enabled() const;
    int get_instance_overflow() const;
    float get_instance_resolution() const;
    bool get_instance_reverse() const;

    // === Parameters (ModuleParams framework with YAML configuration) ===
    DEFINE_PARAMETERS(
        // Global parameters
        (ParamInt<px4::params::QE_POLL_RATE>) _param_poll_rate,

        // Encoder 0 parameters
        (ParamBool<px4::params::QE_0_ENABLE>) _param_qe0_enable,
        (ParamInt<px4::params::QE_0_OVERFLOW>) _param_qe0_overflow,
        (ParamFloat<px4::params::QE_0_RESOLUTION>) _param_qe0_resolution,
        (ParamBool<px4::params::QE_0_REVERSE>) _param_qe0_reverse,

        // Encoder 1 parameters
        (ParamBool<px4::params::QE_1_ENABLE>) _param_qe1_enable,
        (ParamInt<px4::params::QE_1_OVERFLOW>) _param_qe1_overflow,
        (ParamFloat<px4::params::QE_1_RESOLUTION>) _param_qe1_resolution,
        (ParamBool<px4::params::QE_1_REVERSE>) _param_qe1_reverse
    )
};
