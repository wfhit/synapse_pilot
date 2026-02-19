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

#include "boom_hardware_interface.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

BoomHardwareInterface::BoomHardwareInterface(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool BoomHardwareInterface::initialize(int encoder_instance, int motor_instance)
{
	_encoder_instance = encoder_instance;
	_motor_instance = motor_instance;

	// Update parameters on initialization
	update_parameters();

	// Initialize H-bridge command publisher
	hbridge_setpoint_s initial_cmd{};
	initial_cmd.timestamp = hrt_absolute_time();
	initial_cmd.duty_cycle = 0.0f;
	initial_cmd.enable = false;

	_hbridge_command_pub = orb_advertise(ORB_ID(hbridge_setpoint), &initial_cmd);

	if (_hbridge_command_pub == nullptr) {
		PX4_ERR("Failed to advertise hbridge_setpoint");
		return false;
	}

	_initialized = true;
	PX4_INFO("Hardware interface initialized (encoder: %d, motor: %d)", encoder_instance, motor_instance);
	return true;
}

bool BoomHardwareInterface::update_sensors(SensorData &data)
{
	bool all_sensors_updated = true;
	data.timestamp = hrt_absolute_time();

	// Update each sensor type following bucket control pattern
	all_sensors_updated &= update_encoder_data(data);
	all_sensors_updated &= update_limit_switches(data);

	// Update overall health status
	_sensor_healthy = all_sensors_updated;
	_last_sensor_time = data.timestamp;

	// Cache sensor data for performance
	if (all_sensors_updated) {
		_last_sensor_data = data;
	}

	return all_sensors_updated;
}

bool BoomHardwareInterface::send_command(const HbridgeSetpoint &command)
{
	if (!_initialized || _hbridge_command_pub == nullptr) {
		return false;
	}

	// Apply current limiting if status is available (current not available in hbridge_status_s)
	HbridgeSetpoint limited_command = command;

	// Check limit switches and block movement if necessary
	if (limited_command.enable && fabsf(limited_command.duty_cycle) > 0.1f) {
		bool direction_up = limited_command.duty_cycle > 0.0f;

		if (is_movement_blocked_by_limits(direction_up)) {
			PX4_WARN("Movement blocked by limit switch, setting duty cycle to 0");
			limited_command.duty_cycle = 0.0f;
			limited_command.enable = false;
		}
	}

	// Prepare uORB message
	hbridge_setpoint_s cmd_msg{};
	cmd_msg.timestamp = hrt_absolute_time();
	cmd_msg.duty_cycle = math::constrain(limited_command.duty_cycle, -1.0f, 1.0f);
	cmd_msg.enable = limited_command.enable;

	// Publish command
	int ret = orb_publish(ORB_ID(hbridge_setpoint), _hbridge_command_pub, &cmd_msg);

	if (ret == PX4_OK) {
		_last_command = limited_command;
		_last_command_time = cmd_msg.timestamp;
		return true;
	}

	return false;
}

bool BoomHardwareInterface::update_status(HbridgeStatus &status)
{
	// Select H-bridge instance if not done yet
	if (_hbridge_selected < 0) {
		_hbridge_selected = select_hbridge_instance();

		if (_hbridge_selected < 0) {
			return false;
		}
	}

	// Read H-bridge status
	hbridge_status_s status_msg;

	if (_hbridge_status_sub[_hbridge_selected].copy(&status_msg)) {
		status.enabled = status_msg.enabled;
		status.timestamp = status_msg.timestamp;

		// Update health monitoring (fault monitoring not available in hbridge_status_s)
		_actuator_healthy = status.enabled;  // Consider healthy if enabled

		_last_status = status;
		_last_status_time = status.timestamp;
		return true;
	}

	// Check for timeout
	hrt_abstime now = hrt_absolute_time();

	if (now - _last_status_time > STATUS_TIMEOUT_US) {
		_actuator_healthy = false;
	}

	return false;
}

void BoomHardwareInterface::emergency_stop()
{
	HbridgeSetpoint stop_cmd{};
	stop_cmd.duty_cycle = 0.0f;
	stop_cmd.enable = false;
	stop_cmd.mode = 0;

	send_command(stop_cmd);
	PX4_WARN("Emergency stop activated");
}

bool BoomHardwareInterface::calibrate_sensor(float reference_angle)
{
	if (!_sensor_healthy || _last_sensor_data.timestamp == 0) {
		PX4_WARN("Cannot calibrate: sensor not healthy");
		return false;
	}

	// Calculate new offset based on reference
	_calibration_offset = reference_angle - _last_sensor_data.raw_angle;
	PX4_INFO("Sensor calibrated: offset = %.2f deg", (double)_calibration_offset);
	return true;
}

void BoomHardwareInterface::reset_sensor_calibration()
{
	_calibration_scale = 1.0f;
	_calibration_offset = 0.0f;
	_angle_reversed = false;
	PX4_INFO("Sensor calibration reset");
}

bool BoomHardwareInterface::is_healthy() const
{
	return _sensor_healthy && _actuator_healthy && _limits_healthy;
}

hrt_abstime BoomHardwareInterface::time_since_last_update() const
{
	hrt_abstime now = hrt_absolute_time();
	return now - _last_sensor_time;
}

void BoomHardwareInterface::update_parameters()
{
	// Update all parameters from the parameter system
	updateParams();

	// Apply encoder parameters
	_encoder_instance = _param_encoder_instance.get();
	_calibration_scale = _param_encoder_scale.get();
	_calibration_offset = _param_encoder_offset.get();
	_angle_reversed = (_param_angle_reverse.get() != 0);

	// Apply motor parameters
	_motor_instance = _param_motor_index.get();

	// Apply limit sensor parameters
	_limit_up_instance = _param_limit_up_index.get();
	_limit_down_instance = _param_limit_down_index.get();

	PX4_DEBUG("Parameters updated: enc_inst=%d, motor_inst=%d, limit_up=%d, limit_down=%d",
		  _encoder_instance, _motor_instance, _limit_up_instance, _limit_down_instance);
}

int BoomHardwareInterface::select_hbridge_instance()
{
	// Find H-bridge instance matching our motor index
	for (uint8_t i = 0; i < _hbridge_status_sub.size(); i++) {
		hbridge_status_s status;

		if (_hbridge_status_sub[i].copy(&status)) {
			// Check if this instance matches our configuration
			// Note: hbridge_status_s doesn't have device_id, use instance field instead
			if (static_cast<int>(status.instance) == _motor_instance) {
				PX4_INFO("Selected H-bridge instance %d (instance: %u)", i, status.instance);
				return i;
			}
		}
	}

	PX4_WARN("No matching H-bridge instance found for motor %d", _motor_instance);
	return -1;
}

float BoomHardwareInterface::apply_sensor_calibration(float raw_angle) const
{
	float calibrated_angle = raw_angle;

	// Apply scale and offset
	calibrated_angle = calibrated_angle * _calibration_scale + _calibration_offset;

	// Apply reversal if configured
	if (_angle_reversed) {
		calibrated_angle = -calibrated_angle;
	}

	// Normalize to 0-360 degrees
	calibrated_angle = fmodf(calibrated_angle + 360.0f, 360.0f);

	return calibrated_angle;
}

bool BoomHardwareInterface::validate_sensor_data(const SensorData &data) const
{
	// Check timestamp freshness
	hrt_abstime now = hrt_absolute_time();

	if (now - data.timestamp > SENSOR_TIMEOUT_US) {
		return false;
	}

	// Check magnet detection
	if (!data.magnet_detected) {
		return false;
	}

	// Check angle bounds
	if (!PX4_ISFINITE(data.raw_angle) || !PX4_ISFINITE(data.calibrated_angle)) {
		return false;
	}

	// Check status flags for errors
	if ((data.status_flags & 0xFE) != 0) { // Any error bits set
		return false;
	}

	return true;
}

bool BoomHardwareInterface::is_timestamp_valid(hrt_abstime timestamp, hrt_abstime timeout_us) const
{
	hrt_abstime now = hrt_absolute_time();
	return (now - timestamp) <= timeout_us;
}

BoomHardwareInterface::HbridgeSetpoint BoomHardwareInterface::apply_current_limit(const HbridgeSetpoint &command,
		float current) const
{
	HbridgeSetpoint limited_command = command;
	float current_limit = _param_current_limit.get();
	float duty_max = _param_duty_max.get();

	// Apply duty cycle limit
	limited_command.duty_cycle = math::constrain(command.duty_cycle, -duty_max, duty_max);

	// Apply current limiting if current exceeds limit
	if (fabsf(current) > current_limit && fabsf(command.duty_cycle) > 0.1f) {
		float reduction_factor = current_limit / fabsf(current);
		limited_command.duty_cycle *= math::constrain(reduction_factor, 0.1f, 1.0f);
		PX4_DEBUG("Current limiting applied: %.2fA -> duty reduced to %.3f",
			  (double)current, (double)limited_command.duty_cycle);
	}

	return limited_command;
}

bool BoomHardwareInterface::select_limit_sensor_instances()
{
	// Select limit sensor instances if not already done
	if (_limit_up_selected < 0 || _limit_down_selected < 0) {
		bool up_found = false, down_found = false;

		for (uint8_t i = 0; i < _limit_sensor_sub.size(); i++) {
			sensor_limit_switch_s limit_msg;

			if (_limit_sensor_sub[i].copy(&limit_msg)) {
				// Check if this instance matches our up limit configuration
				if (static_cast<int>(limit_msg.instance) == _limit_up_instance) {
					_limit_up_selected = i;
					up_found = true;
					PX4_INFO("Selected limit up instance %d (instance: %u)", i, limit_msg.instance);
				}

				// Check if this instance matches our down limit configuration
				if (static_cast<int>(limit_msg.instance) == _limit_down_instance) {
					_limit_down_selected = i;
					down_found = true;
					PX4_INFO("Selected limit down instance %d (instance: %u)", i, limit_msg.instance);
				}
			}
		}

		if (!up_found) {
			PX4_WARN("No matching limit up sensor found for device ID %d", _limit_up_instance);
		}

		if (!down_found) {
			PX4_WARN("No matching limit down sensor found for device ID %d", _limit_down_instance);
		}

		return up_found && down_found;
	}

	return true;
}

bool BoomHardwareInterface::update_encoder_data(SensorData &data)
{
	sensor_mag_encoder_s encoder_msg;

	if (_mag_encoder_sub.update(&encoder_msg)) {
		// Check if this is the correct encoder instance
		if (encoder_msg.device_id != static_cast<uint32_t>(_encoder_instance)) {
			data.is_valid = false;
			return false;
		}

		// Validate magnetic encoder readings (following bucket control pattern)
		bool magnet_valid = (encoder_msg.magnet_detected == 1); // Check magnet detect flag

		if (!magnet_valid) {
			PX4_DEBUG("Boom encoder: No magnet detected");
			data.is_valid = false;
			return false;
		}

		// Store raw data
		data.raw_angle = math::degrees(encoder_msg.angle);
		data.calibrated_angle = apply_sensor_calibration(data.raw_angle);
		data.magnet_detected = magnet_valid;
		// Construct status from available flags (magnet_detected is bit 0)
		data.status_flags = (encoder_msg.magnet_detected ? 0x01 : 0x00) |
				    (encoder_msg.magnet_too_strong ? 0x02 : 0x00) |
				    (encoder_msg.magnet_too_weak ? 0x04 : 0x00);
		data.is_valid = (encoder_msg.error_count == 0);

		// Update encoder timestamp tracking
		_last_encoder_update = encoder_msg.timestamp;

		// Validate all sensor data
		data.is_valid = validate_sensor_data(data);

		// Check timestamp validity (following bucket control pattern)
		if (!is_timestamp_valid(_last_encoder_update, SENSOR_TIMEOUT_US)) {
			data.is_valid = false;
			return false;
		}

		return data.is_valid;
	}

	// No encoder data received
	data.is_valid = false;
	return false;
}

bool BoomHardwareInterface::update_limit_switches(SensorData &data)
{
	// Initialize limit switch states
	data.limit_up_active = false;
	data.limit_down_active = false;
	data.limits_valid = false;

	// Select limit sensor instances if needed
	if (!select_limit_sensor_instances()) {
		return false;
	}

	bool up_updated = false, down_updated = false;

	// Read up limit switch
	if (_limit_up_selected >= 0) {
		sensor_limit_switch_s limit_up_msg;

		if (_limit_sensor_sub[_limit_up_selected].copy(&limit_up_msg)) {
			data.limit_up_active = limit_up_msg.state;
			up_updated = true;
		}
	}

	// Read down limit switch
	if (_limit_down_selected >= 0) {
		sensor_limit_switch_s limit_down_msg;

		if (_limit_sensor_sub[_limit_down_selected].copy(&limit_down_msg)) {
			data.limit_down_active = limit_down_msg.state;
			down_updated = true;
		}
	}

	// Update validity and health status
	data.limits_valid = up_updated && down_updated;

	if (data.limits_valid) {
		_last_limit_update = hrt_absolute_time();
		_limits_healthy = true;

	} else {
		// Check for timeout
		hrt_abstime now = hrt_absolute_time();

		if (now - _last_limit_update > SENSOR_TIMEOUT_US) {
			_limits_healthy = false;
		}
	}

	return data.limits_valid;
}

bool BoomHardwareInterface::is_movement_blocked_by_limits(bool direction_up) const
{
	// Check if limit switches are healthy
	if (!_limits_healthy) {
		return false; // Allow movement if limit switches are not working
	}

	// Check appropriate limit switch based on direction
	if (direction_up && _last_sensor_data.limit_up_active) {
		PX4_DEBUG("Movement blocked: up limit switch active");
		return true;
	}

	if (!direction_up && _last_sensor_data.limit_down_active) {
		PX4_DEBUG("Movement blocked: down limit switch active");
		return true;
	}

	return false;
}
