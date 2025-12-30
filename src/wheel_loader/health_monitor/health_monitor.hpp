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

#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_magnetometer.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/input_rc.h>

#include <uORB/topics/health_monitor_status.h>
#include <uORB/topics/sensor_status.h>
#include <uORB/topics/actuator_status.h>

/**
 * @brief Health Monitor
 *
 * Monitors system health across sensors, actuators, power, and vehicle stability.
 * Publishes comprehensive health status at 10Hz for decision making and diagnostics.
 */
class HealthMonitor : public ModuleBase<HealthMonitor>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:
	HealthMonitor();
	~HealthMonitor() override;
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
	 * Check sensor health
	 */
	void check_sensor_health();
	bool check_gps_health();
	bool check_imu_health();
	bool check_magnetometer_health();
	bool check_barometer_health();
	bool check_wheel_loader_sensors();

	/**
	 * Check actuator health
	 */
	void check_actuator_health();
	bool check_actuator_tracking(float error, float threshold);
	bool check_actuator_timeout(uint32_t timeout_ms, uint32_t threshold_ms);

	/**
	 * Check power system health
	 */
	void check_power_health();

	/**
	 * Check vehicle stability
	 */
	void check_vehicle_stability();
	bool check_tilt_angle();
	bool check_slip();
	bool check_shock();

	/**
	 * Check communication connections
	 */
	void check_communication_health();

	/**
	 * Update fault tracking
	 */
	void update_fault_tracking();

	/**
	 * Log health status (1Hz)
	 */
	void log_health_status();

	// uORB subscriptions - Standard PX4 sensors
	uORB::Subscription _sensor_gps_sub{ORB_ID(sensor_gps)};
	uORB::Subscription _sensor_combined_sub{ORB_ID(sensor_combined)};
	uORB::Subscription _vehicle_magnetometer_sub{ORB_ID(vehicle_magnetometer)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// Wheel loader specific subscriptions
	// uORB::Subscription _wl_sensor_status_sub{ORB_ID(wheel_loader_sensor_status)}; // Message not yet defined
	// uORB::Subscription _wl_actuator_status_sub{ORB_ID(wheel_loader_actuator_status)}; // Use standard actuator_status instead
	uORB::Subscription _actuator_status_sub{ORB_ID(actuator_status)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};

	// Publications
	uORB::Publication<health_monitor_status_s> _health_status_pub{ORB_ID(health_monitor_status)};

	// Health status
	health_monitor_status_s _health_status{};

	// Timing
	hrt_abstime _last_run_timestamp{0};
	hrt_abstime _last_log_timestamp{0};

	// Fault tracking
	hrt_abstime _last_sensor_fault_time{0};
	hrt_abstime _last_actuator_fault_time{0};
	hrt_abstime _last_power_fault_time{0};
	hrt_abstime _last_stability_fault_time{0};

	// Shock detection
	float _max_accel_since_last_report{0.0f};

	// Communication tracking
	hrt_abstime _mavlink_last_heartbeat{0};
	hrt_abstime _rc_last_update{0};

	// Performance counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::HM_SHOCK_THRESH>) _param_shock_threshold,
		(ParamFloat<px4::params::HM_SLIP_THRESH>) _param_slip_threshold,
		(ParamFloat<px4::params::HM_ROLL_MAX>) _param_roll_max,
		(ParamFloat<px4::params::HM_PITCH_MAX>) _param_pitch_max,

		(ParamFloat<px4::params::HM_BATT_V_MIN>) _param_battery_voltage_min,
		(ParamFloat<px4::params::HM_BATT_V_MAX>) _param_battery_voltage_max,
		(ParamFloat<px4::params::HM_BATT_I_MAX>) _param_battery_current_max,
		(ParamFloat<px4::params::HM_BATT_PCT_MIN>) _param_battery_percent_min,
		(ParamFloat<px4::params::HM_BATT_T_MAX>) _param_battery_temp_max,

		(ParamInt<px4::params::HM_SNS_TIMEOUT>) _param_sensor_timeout_ms,

		(ParamInt<px4::params::HM_MAV_TIMEOUT>) _param_mavlink_timeout_ms,
		(ParamInt<px4::params::HM_RC_TIMEOUT>) _param_rc_timeout_ms,

		(ParamFloat<px4::params::HM_ACT_CH_ERR>) _param_chassis_error_max,
		(ParamFloat<px4::params::HM_ACT_BOOM_ERR>) _param_boom_error_max,
		(ParamFloat<px4::params::HM_ACT_TILT_ERR>) _param_tilt_error_max,
		(ParamFloat<px4::params::HM_ACT_ART_ERR>) _param_articulation_error_max,
		(ParamInt<px4::params::HM_ACT_TIMEOUT>) _param_actuator_timeout_ms
	)

	static constexpr uint32_t SCHEDULE_INTERVAL_US = 100000;	// 10 Hz (100ms)
	static constexpr uint32_t LOG_INTERVAL_US = 1000000;		// 1 Hz logging (1s)
};
