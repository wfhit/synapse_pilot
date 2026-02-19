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

#include "health_monitor.hpp"

#include <mathlib/mathlib.h>

HealthMonitor::HealthMonitor() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
}

HealthMonitor::~HealthMonitor()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool HealthMonitor::init()
{
	ScheduleOnInterval(SCHEDULE_INTERVAL_US);
	return true;
}

void HealthMonitor::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	const hrt_abstime now = hrt_absolute_time();
	// const float dt = math::constrain((now - _last_run_timestamp) / 1e6f, 0.001f, 0.5f);
	_last_run_timestamp = now;

	// Check all subsystems
	check_sensor_health();
	check_actuator_health();
	check_power_health();
	check_vehicle_stability();
	check_communication_health();

	// Update overall health summary
	_health_status.sensors_healthy =
		_health_status.sensor_gps &&
		_health_status.sensor_imu &&
		_health_status.sensor_mag &&
		_health_status.sensor_baro &&
		_health_status.sensor_boom &&
		_health_status.sensor_tilt &&
		_health_status.sensor_articulation &&
		_health_status.sensor_limit_switches;

	_health_status.actuators_healthy =
		_health_status.actuator_chassis_left &&
		_health_status.actuator_chassis_right &&
		_health_status.actuator_boom &&
		_health_status.actuator_tilt &&
		_health_status.actuator_articulation;

	_health_status.power_healthy =
		_health_status.battery_voltage_ok &&
		_health_status.battery_current_ok &&
		_health_status.battery_charge_ok &&
		_health_status.battery_temp_ok;

	_health_status.vehicle_stable =
		_health_status.tilt_angle_safe &&
		_health_status.slip_ok &&
		_health_status.shock_ok &&
		_health_status.temperature_ok;

	_health_status.system_healthy =
		_health_status.sensors_healthy &&
		_health_status.actuators_healthy &&
		_health_status.power_healthy &&
		_health_status.vehicle_stable;

	// Update fault tracking
	update_fault_tracking();

	// Publish health status at 10Hz
	_health_status.timestamp = now;
	_health_status_pub.publish(_health_status);

	// Log at 1Hz
	if (now - _last_log_timestamp > LOG_INTERVAL_US) {
		log_health_status();
		_last_log_timestamp = now;

		// Reset peak acceleration after logging
		_health_status.max_accel_magnitude = _max_accel_since_last_report;
		_max_accel_since_last_report = 0.0f;
	}

	perf_end(_loop_perf);
}

void HealthMonitor::check_sensor_health()
{
	_health_status.sensor_gps = check_gps_health();
	_health_status.sensor_imu = check_imu_health();
	_health_status.sensor_mag = check_magnetometer_health();
	_health_status.sensor_baro = check_barometer_health();

	// bool wl_sensors_ok = check_wheel_loader_sensors();
	// Individual WL sensor bits updated in check_wheel_loader_sensors()
}

bool HealthMonitor::check_gps_health()
{
	sensor_gps_s gps;

	if (_sensor_gps_sub.copy(&gps)) {
		const hrt_abstime now = hrt_absolute_time();
		const uint32_t timeout_ms = (now - gps.timestamp) / 1000;

		if ((int32_t)timeout_ms > _param_sensor_timeout_ms.get()) {
			return false;
		}

		// Check fix type (3D fix required)
		if (gps.fix_type < 3) {
			return false;
		}

		// Check number of satellites
		if (gps.satellites_used < 6) {
			return false;
		}

		return true;
	}

	return false;
}

bool HealthMonitor::check_imu_health()
{
	sensor_combined_s sensor;

	if (_sensor_combined_sub.copy(&sensor)) {
		const hrt_abstime now = hrt_absolute_time();
		const uint32_t timeout_ms = (now - sensor.timestamp) / 1000;

		if ((int32_t)timeout_ms > _param_sensor_timeout_ms.get()) {
			return false;
		}

		// Check for reasonable accelerometer values (not NaN, within limits)
		if (!PX4_ISFINITE(sensor.accelerometer_m_s2[0]) ||
		    !PX4_ISFINITE(sensor.accelerometer_m_s2[1]) ||
		    !PX4_ISFINITE(sensor.accelerometer_m_s2[2])) {
			return false;
		}

		// Track peak acceleration for shock detection
		float accel_magnitude = sqrtf(
						sensor.accelerometer_m_s2[0] * sensor.accelerometer_m_s2[0] +
						sensor.accelerometer_m_s2[1] * sensor.accelerometer_m_s2[1] +
						sensor.accelerometer_m_s2[2] * sensor.accelerometer_m_s2[2]
					);

		if (accel_magnitude > _max_accel_since_last_report) {
			_max_accel_since_last_report = accel_magnitude;
		}

		// Check for reasonable gyro values
		if (!PX4_ISFINITE(sensor.gyro_rad[0]) ||
		    !PX4_ISFINITE(sensor.gyro_rad[1]) ||
		    !PX4_ISFINITE(sensor.gyro_rad[2])) {
			return false;
		}

		return true;
	}

	return false;
}

bool HealthMonitor::check_magnetometer_health()
{
	vehicle_magnetometer_s mag;

	if (_vehicle_magnetometer_sub.copy(&mag)) {
		const hrt_abstime now = hrt_absolute_time();
		const uint32_t timeout_ms = (now - mag.timestamp) / 1000;

		if ((int32_t)timeout_ms > _param_sensor_timeout_ms.get()) {
			return false;
		}

		// Check for reasonable values
		if (!PX4_ISFINITE(mag.magnetometer_ga[0]) ||
		    !PX4_ISFINITE(mag.magnetometer_ga[1]) ||
		    !PX4_ISFINITE(mag.magnetometer_ga[2])) {
			return false;
		}

		return true;
	}

	return false;
}

bool HealthMonitor::check_barometer_health()
{
	vehicle_air_data_s baro;

	if (_vehicle_air_data_sub.copy(&baro)) {
		const hrt_abstime now = hrt_absolute_time();
		const uint32_t timeout_ms = (now - baro.timestamp) / 1000;

		if ((int32_t)timeout_ms > _param_sensor_timeout_ms.get()) {
			return false;
		}

		// Check for reasonable pressure value
		if (!PX4_ISFINITE(baro.baro_pressure_pa)) {
			return false;
		}

		return true;
	}

	return false;
}

bool HealthMonitor::check_wheel_loader_sensors()
{
	// wheel_loader_sensor_status_s sensor_status; // Message not yet defined
	// if (_wl_sensor_status_sub.copy(&sensor_status)) {
	// 	const hrt_abstime now = hrt_absolute_time();
	// 	const uint32_t timeout_ms = (now - sensor_status.timestamp) / 1000;
	//
	// 	if ((int32_t)timeout_ms > _param_sensor_timeout_ms.get()) {
	// 		_health_status.sensor_boom = false;
	// 		_health_status.sensor_tilt = false;
	// 		_health_status.sensor_articulation = false;
	// 		_health_status.sensor_limit_switches = false;
	// 		return false;
	// 	}
	//
	// 	// Check individual sensors with quality thresholds
	// 	_health_status.sensor_boom = sensor_status.boom_valid &&
	// 				     (sensor_status.boom_quality > 50);
	//
	// 	_health_status.sensor_tilt = sensor_status.tilt_valid &&
	// 				     (sensor_status.tilt_quality > 50);
	//
	// 	_health_status.sensor_articulation = sensor_status.articulation_valid &&
	// 					     (sensor_status.articulation_quality > 50);
	//
	// 	// All limit switches should be readable (at least one of each pair active is OK)
	// 	_health_status.sensor_limit_switches = true; // Assume OK if message received
	//
	// 	return _health_status.sensor_boom &&
	// 	       _health_status.sensor_tilt &&
	// 	       _health_status.sensor_articulation;
	// }

	// No data received
	_health_status.sensor_boom = false;
	_health_status.sensor_tilt = false;
	_health_status.sensor_articulation = false;
	_health_status.sensor_limit_switches = false;
	return false;
}

void HealthMonitor::check_actuator_health()
{
	actuator_status_s actuator_status;

	if (_actuator_status_sub.copy(&actuator_status)) {
		// Chassis left
		_health_status.actuator_chassis_left =
			actuator_status.chassis_left_valid &&
			check_actuator_tracking(actuator_status.chassis_left_error,
						_param_chassis_error_max.get()) &&
			check_actuator_timeout(actuator_status.chassis_left_timeout_ms,
					       _param_actuator_timeout_ms.get());

		// Chassis right
		_health_status.actuator_chassis_right =
			actuator_status.chassis_right_valid &&
			check_actuator_tracking(actuator_status.chassis_right_error,
						_param_chassis_error_max.get()) &&
			check_actuator_timeout(actuator_status.chassis_right_timeout_ms,
					       _param_actuator_timeout_ms.get());

		// Boom
		_health_status.actuator_boom =
			actuator_status.boom_valid &&
			check_actuator_tracking(actuator_status.boom_error,
						_param_boom_error_max.get()) &&
			check_actuator_timeout(actuator_status.boom_timeout_ms,
					       _param_actuator_timeout_ms.get());

		// Tilt
		_health_status.actuator_tilt =
			actuator_status.tilt_valid &&
			check_actuator_tracking(actuator_status.tilt_error,
						_param_tilt_error_max.get()) &&
			check_actuator_timeout(actuator_status.tilt_timeout_ms,
					       _param_actuator_timeout_ms.get());

		// Articulation
		_health_status.actuator_articulation =
			actuator_status.articulation_valid &&
			check_actuator_tracking(actuator_status.articulation_error,
						_param_articulation_error_max.get()) &&
			check_actuator_timeout(actuator_status.articulation_timeout_ms,
					       _param_actuator_timeout_ms.get());

	} else {
		// No actuator data - all unhealthy
		_health_status.actuator_chassis_left = false;
		_health_status.actuator_chassis_right = false;
		_health_status.actuator_boom = false;
		_health_status.actuator_tilt = false;
		_health_status.actuator_articulation = false;
	}
}

bool HealthMonitor::check_actuator_tracking(float error, float threshold)
{
	return fabsf(error) < threshold;
}

bool HealthMonitor::check_actuator_timeout(uint32_t timeout_ms, uint32_t threshold_ms)
{
	return timeout_ms < threshold_ms;
}

void HealthMonitor::check_power_health()
{
	battery_status_s battery;

	if (_battery_status_sub.copy(&battery)) {
		// Check voltage
		_health_status.battery_voltage_ok =
			(battery.voltage_v > _param_battery_voltage_min.get()) &&
			(battery.voltage_v < _param_battery_voltage_max.get());

		// Check current
		_health_status.battery_current_ok =
			fabsf(battery.current_a) < _param_battery_current_max.get();

		// Check charge level
		_health_status.battery_charge_ok =
			battery.remaining > _param_battery_percent_min.get();

		// Check temperature
		_health_status.battery_temp_ok =
			battery.temperature < _param_battery_temp_max.get();

	} else {
		// No battery data - assume unhealthy
		_health_status.battery_voltage_ok = false;
		_health_status.battery_current_ok = false;
		_health_status.battery_charge_ok = false;
		_health_status.battery_temp_ok = false;
	}
}

void HealthMonitor::check_vehicle_stability()
{
	_health_status.tilt_angle_safe = check_tilt_angle();
	_health_status.slip_ok = check_slip();
	_health_status.shock_ok = check_shock();

	// Temperature monitoring (placeholder - would use actual temperature sensors)
	_health_status.temperature_ok = true;
}

bool HealthMonitor::check_tilt_angle()
{
	vehicle_attitude_s attitude;

	if (_vehicle_attitude_sub.copy(&attitude)) {
		// Extract roll and pitch from quaternion
		const float q0 = attitude.q[0];
		const float q1 = attitude.q[1];
		const float q2 = attitude.q[2];
		const float q3 = attitude.q[3];

		// Calculate Euler angles
		float roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
				    1.0f - 2.0f * (q1 * q1 + q2 * q2));
		float pitch = asinf(2.0f * (q0 * q2 - q3 * q1));

		_health_status.vehicle_roll = roll;
		_health_status.vehicle_pitch = pitch;

		// Check against limits
		if (fabsf(roll) > _param_roll_max.get()) {
			return false;
		}

		if (fabsf(pitch) > _param_pitch_max.get()) {
			return false;
		}

		return true;
	}

	return false;
}

bool HealthMonitor::check_slip()
{
	// Slip detection: compare wheel odometry velocity with IMU-derived velocity
	// This requires both actuator status (wheel speeds) and local position (IMU velocity)

	actuator_status_s actuator_status;
	vehicle_local_position_s local_pos;

	if (_actuator_status_sub.copy(&actuator_status) &&
	    _vehicle_local_position_sub.copy(&local_pos)) {

		// Calculate wheel-based velocity estimate
		float wheel_velocity = (actuator_status.chassis_left_velocity_actual +
					actuator_status.chassis_right_velocity_actual) / 2.0f;

		// Get IMU-based velocity estimate (forward velocity)
		float imu_velocity = sqrtf(local_pos.vx * local_pos.vx +
					   local_pos.vy * local_pos.vy);

		// Calculate slip ratio
		float slip_ratio = 0.0f;

		if (fabsf(wheel_velocity) > 0.1f) {
			slip_ratio = fabsf(wheel_velocity - imu_velocity) / fabsf(wheel_velocity);
		}

		_health_status.slip_ratio = slip_ratio;

		return slip_ratio < _param_slip_threshold.get();
	}

	// No data - assume OK
	_health_status.slip_ratio = 0.0f;
	return true;
}

bool HealthMonitor::check_shock()
{
	// Shock is detected by monitoring peak acceleration
	// Peak acceleration is tracked in check_imu_health()

	// Check if peak acceleration exceeds threshold (in g's)
	const float shock_threshold_ms2 = _param_shock_threshold.get() * 9.80665f; // Convert g's to m/s²

	return _max_accel_since_last_report < shock_threshold_ms2;
}

void HealthMonitor::check_communication_health()
{
	const hrt_abstime now = hrt_absolute_time();

	// Check MAVLink connection (via vehicle_command as heartbeat proxy)
	vehicle_command_s cmd;

	if (_vehicle_command_sub.copy(&cmd)) {
		_mavlink_last_heartbeat = now;
	}

	const int mavlink_timeout_ms = _param_mavlink_timeout_ms.get();

	if (mavlink_timeout_ms > 0) {
		_health_status.mavlink_connected = (now - _mavlink_last_heartbeat) < (uint64_t)(mavlink_timeout_ms * 1000);

	} else {
		_health_status.mavlink_connected = false;  // Disabled
	}

	_health_status.mavlink_last_heartbeat = _mavlink_last_heartbeat;

	// Check RC connection
	input_rc_s rc;

	if (_input_rc_sub.copy(&rc)) {
		if (rc.timestamp_last_signal > _rc_last_update) {
			_rc_last_update = rc.timestamp_last_signal;
		}
	}

	const int rc_timeout_ms = _param_rc_timeout_ms.get();

	if (rc_timeout_ms > 0) {
		_health_status.rc_connected = (now - _rc_last_update) < (uint64_t)(rc_timeout_ms * 1000);

	} else {
		_health_status.rc_connected = false;  // Disabled
	}

	_health_status.rc_last_update = _rc_last_update;
}

void HealthMonitor::update_fault_tracking()
{
	const hrt_abstime now = hrt_absolute_time();

	// Update sensor faults
	if (!_health_status.sensors_healthy) {
		if (_last_sensor_fault_time == 0 ||
		    (now - _last_sensor_fault_time) > 1000000) {  // 1 second in microseconds
			_health_status.sensor_fault_count++;
		}

		_last_sensor_fault_time = now;
	}

	// Update actuator faults
	if (!_health_status.actuators_healthy) {
		if (_last_actuator_fault_time == 0 ||
		    (now - _last_actuator_fault_time) > 1000000) {  // 1 second in microseconds
			_health_status.actuator_fault_count++;
		}

		_last_actuator_fault_time = now;
	}

	// Update power faults
	if (!_health_status.power_healthy) {
		if (_last_power_fault_time == 0 ||
		    (now - _last_power_fault_time) > 1000000) {  // 1 second in microseconds
			_health_status.power_fault_count++;
		}

		_last_power_fault_time = now;
	}

	// Update stability faults
	if (!_health_status.vehicle_stable) {
		if (_last_stability_fault_time == 0 ||
		    (now - _last_stability_fault_time) > 1000000) {  // 1 second in microseconds
			_health_status.stability_fault_count++;
		}

		_last_stability_fault_time = now;
	}

	// Calculate time since last fault
	_health_status.time_since_sensor_fault =
		_last_sensor_fault_time > 0 ? (now - _last_sensor_fault_time) / 1e6f : -1.0f;

	_health_status.time_since_actuator_fault =
		_last_actuator_fault_time > 0 ? (now - _last_actuator_fault_time) / 1e6f : -1.0f;

	_health_status.time_since_power_fault =
		_last_power_fault_time > 0 ? (now - _last_power_fault_time) / 1e6f : -1.0f;

	_health_status.time_since_stability_fault =
		_last_stability_fault_time > 0 ? (now - _last_stability_fault_time) / 1e6f : -1.0f;
}

void HealthMonitor::log_health_status()
{
	// Log comprehensive health status at 1Hz
	PX4_INFO("=== Health Monitor Status ===");
	PX4_INFO("System: %s | Sensors: %s | Actuators: %s | Power: %s | Stable: %s",
		 _health_status.system_healthy ? "OK" : "FAIL",
		 _health_status.sensors_healthy ? "OK" : "FAIL",
		 _health_status.actuators_healthy ? "OK" : "FAIL",
		 _health_status.power_healthy ? "OK" : "FAIL",
		 _health_status.vehicle_stable ? "OK" : "FAIL");

	// Log fault counts if any exist
	if (_health_status.sensor_fault_count > 0 ||
	    _health_status.actuator_fault_count > 0 ||
	    _health_status.power_fault_count > 0 ||
	    _health_status.stability_fault_count > 0) {
		PX4_INFO("Fault counts - Sensor: %lu, Actuator: %lu, Power: %lu, Stability: %lu",
			 (unsigned long)_health_status.sensor_fault_count,
			 (unsigned long)_health_status.actuator_fault_count,
			 (unsigned long)_health_status.power_fault_count,
			 (unsigned long)_health_status.stability_fault_count);
	}

	// Log critical issues
	if (!_health_status.tilt_angle_safe) {
		PX4_WARN("TILT ANGLE EXCEEDED: roll=%.2f pitch=%.2f",
			 (double)_health_status.vehicle_roll,
			 (double)_health_status.vehicle_pitch);
	}

	if (!_health_status.slip_ok) {
		PX4_WARN("EXCESSIVE SLIP DETECTED: %.1f%%",
			 (double)(_health_status.slip_ratio * 100.0f));
	}

	if (!_health_status.shock_ok) {
		PX4_WARN("SHOCK DETECTED: %.2f m/s^2",
			 (double)_health_status.max_accel_magnitude);
	}
}

int HealthMonitor::task_spawn(int argc, char *argv[])
{
	HealthMonitor *instance = new HealthMonitor();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int HealthMonitor::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int HealthMonitor::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Health monitor for wheel loader system. Monitors sensors, actuators, power system,
and vehicle stability. Publishes comprehensive health status at 10Hz.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("health_monitor", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int health_monitor_main(int argc, char *argv[])
{
	return HealthMonitor::main(argc, argv);
}
