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

#include "tilt_hardware_interface.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>
#include <uORB/uORB.h>

TiltHardwareInterface::TiltHardwareInterface(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool TiltHardwareInterface::initialize(uint8_t motor_index, uint8_t encoder_index)
{
	_motor_index = motor_index;
	_encoder_index = encoder_index;

	// Load hardware configuration parameters
	_limit_load_index = static_cast<uint8_t>(_param_limit_load_index.get());
	_limit_dump_index = static_cast<uint8_t>(_param_limit_dump_index.get());
	_encoder_scale_factor = _param_encoder_scale.get();
	_encoder_zero_offset = _param_encoder_offset.get();

	// Initialize health status
	_is_healthy = false;
	_last_update = hrt_absolute_time();

	PX4_INFO("Bucket hardware initialized: motor=%d, encoder=%d, limits=[%d,%d]",
		 _motor_index, _encoder_index, _limit_load_index, _limit_dump_index);

	return true;
}

bool TiltHardwareInterface::update_sensors(SensorData& data)
{
	bool all_sensors_updated = true;
	data.timestamp = hrt_absolute_time();

	// Update each sensor type
	all_sensors_updated &= update_encoder_data(data);
	all_sensors_updated &= update_boom_angle(data);
	all_sensors_updated &= update_limit_switches(data);
	all_sensors_updated &= update_motor_status(data);

	// Update overall health status
	_is_healthy = all_sensors_updated;
	_last_update = data.timestamp;

	// Cache sensor data for performance
	if (all_sensors_updated) {
		_cached_sensor_data = data;
		_sensor_data_valid = true;
	}

	return all_sensors_updated;
}

bool TiltHardwareInterface::update_encoder_data(SensorData& data)
{
	sensor_quad_encoder_s encoder_msg;

	// Select encoder instance if not already selected
	if (_encoder_selected < 0) {
		_encoder_selected = select_encoder_instance();
		if (_encoder_selected < 0) {
			data.encoder_fault = true;
			return false;
		}
	}

	// Read from selected encoder instance
	if (_encoder_selected >= 0 && _encoder_sub[_encoder_selected].update(&encoder_msg)) {
		// Verify this is our encoder instance
		if (encoder_msg.instance == _encoder_index) {
			// Convert encoder position to hbridge length
			float position_rad = static_cast<float>(encoder_msg.position) * 1e-6f;
			data.hbridge_position = (position_rad - _encoder_zero_offset) * _encoder_scale_factor +
					_param_hbridge_min_length.get();

			// Convert encoder velocity to hbridge velocity
			float velocity_rad_s = static_cast<float>(encoder_msg.velocity) * 1e-6f;
			data.hbridge_velocity = velocity_rad_s * _encoder_scale_factor;			_last_encoder_update = encoder_msg.timestamp;
			data.encoder_fault = false;

			return is_timestamp_valid(_last_encoder_update, SENSOR_TIMEOUT_US);
		}
	}

	data.encoder_fault = true;
	return false;
}

bool TiltHardwareInterface::update_boom_angle(SensorData& data)
{
	sensor_mag_encoder_s mag_encoder_msg;

	if (_mag_encoder_sub.update(&mag_encoder_msg)) {
		// Validate magnetic encoder readings
		if (mag_encoder_msg.magnet_detected &&
		    !mag_encoder_msg.magnet_too_strong &&
		    !mag_encoder_msg.magnet_too_weak) {

			// Store raw sensor angle - boom angle calculation should be done in kinematics
			data.sensor_angle = mag_encoder_msg.angle;
			data.sensor_angle_valid = true;
			return true;
		} else {
			// Log sensor issues for debugging
			if (!mag_encoder_msg.magnet_detected) {
				PX4_DEBUG("Boom encoder: No magnet detected");
			} else if (mag_encoder_msg.magnet_too_strong) {
				PX4_DEBUG("Boom encoder: Magnet too strong");
			} else if (mag_encoder_msg.magnet_too_weak) {
				PX4_DEBUG("Boom encoder: Magnet too weak");
			}
		}
	}

	data.sensor_angle_valid = false;
	return false;
}

bool TiltHardwareInterface::update_limit_switches(SensorData& data)
{
	sensor_limit_switch_s limit_msg;
	bool load_updated = false;
	bool dump_updated = false;

	// Select limit sensor instances if not already selected
	if (_limit_load_selected < 0 || _limit_dump_selected < 0) {
		int load_instance, dump_instance;
		if (select_limit_sensor_instances(load_instance, dump_instance)) {
			_limit_load_selected = load_instance;
			_limit_dump_selected = dump_instance;
		}
	}

	// Update load limit sensor
	if (_limit_load_selected >= 0 && _limit_sensor_sub[_limit_load_selected].update(&limit_msg)) {
		if (limit_msg.instance == _limit_load_index) {
			data.limit_switch_load = limit_msg.state;
			load_updated = true;
		}
	}

	// Update dump limit sensor
	if (_limit_dump_selected >= 0 && _limit_sensor_sub[_limit_dump_selected].update(&limit_msg)) {
		if (limit_msg.instance == _limit_dump_index) {
			data.limit_switch_dump = limit_msg.state;
			dump_updated = true;
		}
	}

	return load_updated && dump_updated;
}

bool TiltHardwareInterface::update_motor_status(SensorData& data)
{
	hbridge_status_s hbridge_msg;

	// Select hbridge instance if not already selected
	if (_hbridge_status_selected < 0) {
		_hbridge_status_selected = select_hbridge_instance();
		if (_hbridge_status_selected < 0) {
			data.motor_fault = true;
			return false;
		}
	}

	// Read from selected hbridge instance
	if (_hbridge_status_selected >= 0 && _hbridge_status_sub[_hbridge_status_selected].update(&hbridge_msg)) {
		if (hbridge_msg.instance == _motor_index) {
			data.motor_fault = !hbridge_msg.enabled;
			_last_hbridge_update = hbridge_msg.timestamp;

			return is_timestamp_valid(_last_hbridge_update, SENSOR_TIMEOUT_US);
		}
	}

	data.motor_fault = true;
	return false;
}

int TiltHardwareInterface::select_encoder_instance()
{
	sensor_quad_encoder_s encoder_msg;
	const hrt_abstime timestamp_stale = math::max(hrt_absolute_time(), SENSOR_TIMEOUT_US) - SENSOR_TIMEOUT_US;

	if (_encoder_sub.advertised()) {
		for (unsigned i = 0; i < _encoder_sub.size(); i++) {
			if (_encoder_sub[i].update(&encoder_msg)) {
				if ((encoder_msg.timestamp != 0) &&
				    (encoder_msg.timestamp > timestamp_stale) &&
				    (encoder_msg.instance == _encoder_index)) {

					int n_encoders = orb_group_count(ORB_ID(sensor_quad_encoder));
					if (n_encoders > 1) {
						PX4_INFO("Selected encoder instance %d (index %d)", i, _encoder_index);
					}

					return static_cast<int>(i);
				}
			}
		}
	}

	return -1;
}

int TiltHardwareInterface::select_hbridge_instance()
{
	hbridge_status_s hbridge_msg;
	const hrt_abstime timestamp_stale = math::max(hrt_absolute_time(), SENSOR_TIMEOUT_US) - SENSOR_TIMEOUT_US;

	if (_hbridge_status_sub.advertised()) {
		for (unsigned i = 0; i < _hbridge_status_sub.size(); i++) {
			if (_hbridge_status_sub[i].update(&hbridge_msg)) {
				if ((hbridge_msg.timestamp != 0) &&
				    (hbridge_msg.timestamp > timestamp_stale) &&
				    (hbridge_msg.instance == _motor_index)) {

					int n_hbridge = orb_group_count(ORB_ID(hbridge_status));
					if (n_hbridge > 1) {
						PX4_INFO("Selected hbridge instance %d (motor %d)", i, _motor_index);
					}

					return static_cast<int>(i);
				}
			}
		}
	}

	return -1;
}

bool TiltHardwareInterface::select_limit_sensor_instances(int& load_instance, int& dump_instance)
{
	sensor_limit_switch_s limit_msg;
	const hrt_abstime timestamp_stale = math::max(hrt_absolute_time(), SENSOR_TIMEOUT_US) - SENSOR_TIMEOUT_US;
	bool load_found = false, dump_found = false;

	if (_limit_sensor_sub.advertised()) {
		for (unsigned i = 0; i < _limit_sensor_sub.size(); i++) {
			if (_limit_sensor_sub[i].update(&limit_msg)) {
				if ((limit_msg.timestamp != 0) && (limit_msg.timestamp > timestamp_stale)) {
					if (limit_msg.instance == _limit_load_index && !load_found) {
						load_instance = static_cast<int>(i);
						load_found = true;
					} else if (limit_msg.instance == _limit_dump_index && !dump_found) {
						dump_instance = static_cast<int>(i);
						dump_found = true;
					}
				}
			}
		}
	}

	if (load_found && dump_found) {
		int n_limits = orb_group_count(ORB_ID(sensor_limit_switch));
		if (n_limits > 2) {
			PX4_INFO("Selected limit sensors: load=%d, dump=%d", load_instance, dump_instance);
		}
	}

	return load_found && dump_found;
}

bool TiltHardwareInterface::send_hbridge_setpoint(const HbridgeSetpoint& command)
{
	// Start with the command as-is
	HbridgeSetpoint limited_command = command;

	// Check limit switches and block movement if necessary
	if (limited_command.enable && fabsf(limited_command.duty_cycle) > 0.1f) {
		bool direction_dump = limited_command.duty_cycle > 0.0f;
		if (is_movement_blocked_by_limits(direction_dump)) {
			PX4_WARN("Movement blocked by limit switch, setting duty cycle to 0");
			limited_command.duty_cycle = 0.0f;
			limited_command.enable = false;
		}
	}

	hbridge_setpoint_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.instance = _motor_index;
	cmd.duty_cycle = math::constrain(limited_command.duty_cycle, -1.0f, 1.0f);
	cmd.enable = limited_command.enable;

	return _hbridge_command_pub.publish(cmd);
}

void TiltHardwareInterface::emergency_stop()
{
	HbridgeSetpoint stop_cmd{};
	stop_cmd.duty_cycle = 0.0f;
	stop_cmd.enable = false;

	send_hbridge_setpoint(stop_cmd);

	PX4_WARN("Bucket hbridge emergency stop");
}

bool TiltHardwareInterface::reset_encoder()
{
	sensor_quad_encoder_reset_s reset_cmd{};
	reset_cmd.timestamp = hrt_absolute_time();
	reset_cmd.instance = _encoder_index;

	bool success = _encoder_reset_pub.publish(reset_cmd);

	if (success) {
		// Reset our internal tracking
		_encoder_zero_offset = 0.0f;
		PX4_INFO("Encoder reset command sent for instance %d", _encoder_index);
	}

	return success;
}

bool TiltHardwareInterface::perform_self_test()
{
	PX4_INFO("Starting bucket hardware self-test...");

	// Test 1: Check encoder communication
	SensorData test_data{};
	if (!update_encoder_data(test_data)) {
		PX4_ERR("Self-test failed: Encoder communication");
		return false;
	}

	// Test 2: Check limit switches
	if (!update_limit_switches(test_data)) {
		PX4_ERR("Self-test failed: Limit switches");
		return false;
	}

	// Test 3: Check motor status
	if (!update_motor_status(test_data)) {
		PX4_ERR("Self-test failed: Motor status");
		return false;
	}

	// Test 4: Send test command (zero output)
	HbridgeSetpoint test_cmd{};
	test_cmd.duty_cycle = 0.0f;
	test_cmd.enable = true;
	if (!send_hbridge_setpoint(test_cmd)) {
		PX4_ERR("Self-test failed: Motor command");
		return false;
	}

	// Test 5: Check hbridge position range
	float position_range = _param_hbridge_max_length.get() - _param_hbridge_min_length.get();
	if (position_range <= 0.0f) {
		PX4_ERR("Self-test failed: Invalid hbridge range");
		return false;
	}

	PX4_INFO("Bucket hardware self-test passed");
	return true;
}

bool TiltHardwareInterface::get_hardware_status(bool& motor_enabled, bool& encoder_valid, bool& limits_valid) const
{
	if (!_sensor_data_valid) {
		return false;
	}

	motor_enabled = !_cached_sensor_data.motor_fault;
	encoder_valid = !_cached_sensor_data.encoder_fault;
	limits_valid = true; // Simplified - could add more detailed limit sensor validation

	return true;
}

void TiltHardwareInterface::update_parameters()
{
	// Update parameters from parameter system
	ModuleParams::updateParams();

	// Update hardware configuration from parameters
	uint8_t new_motor_index = static_cast<uint8_t>(_param_motor_index.get());
	uint8_t new_encoder_index = static_cast<uint8_t>(_param_encoder_index.get());
	uint8_t new_limit_load_index = static_cast<uint8_t>(_param_limit_load_index.get());
	uint8_t new_limit_dump_index = static_cast<uint8_t>(_param_limit_dump_index.get());

	// Check if motor or encoder indices have changed (requires reinitialization)
	bool indices_changed = (new_motor_index != _motor_index) ||
	                      (new_encoder_index != _encoder_index) ||
	                      (new_limit_load_index != _limit_load_index) ||
	                      (new_limit_dump_index != _limit_dump_index);

	if (indices_changed) {
		PX4_INFO("Hardware indices changed, reinitializing interface");
		PX4_INFO("  Motor: %d -> %d, Encoder: %d -> %d",
			_motor_index, new_motor_index, _encoder_index, new_encoder_index);
		PX4_INFO("  Limit load: %d -> %d, Limit dump: %d -> %d",
			_limit_load_index, new_limit_load_index, _limit_dump_index, new_limit_dump_index);

		// Update indices
		_motor_index = new_motor_index;
		_encoder_index = new_encoder_index;
		_limit_load_index = new_limit_load_index;
		_limit_dump_index = new_limit_dump_index;

		// Reset instance selections to force reselection
		_encoder_selected = -1;
		_hbridge_status_selected = -1;
		_limit_load_selected = -1;
		_limit_dump_selected = -1;

		// Mark sensor data as invalid to force refresh
		_sensor_data_valid = false;
	}

	// Update encoder calibration parameters
	float new_encoder_scale = _param_encoder_scale.get();
	float new_encoder_offset = _param_encoder_offset.get();

	if (fabsf(new_encoder_scale - _encoder_scale_factor) > 0.001f ||
	    fabsf(new_encoder_offset - _encoder_zero_offset) > 0.001f) {

		PX4_DEBUG("Encoder calibration updated: scale=%.4f->%.4f, offset=%.2f->%.2f",
			(double)_encoder_scale_factor, (double)new_encoder_scale,
			(double)_encoder_zero_offset, (double)new_encoder_offset);

		_encoder_scale_factor = new_encoder_scale;
		_encoder_zero_offset = new_encoder_offset;
	}

	PX4_DEBUG("Hardware interface parameters updated");
}

bool TiltHardwareInterface::is_movement_blocked_by_limits(bool direction_dump) const
{
	// Always allow movement if sensor data is not valid
	if (!_sensor_data_valid) {
		return false; // Allow movement if limit switches are not working
	}

	// Check appropriate limit switch based on direction
	if (direction_dump && _cached_sensor_data.limit_switch_dump) {
		PX4_DEBUG("Movement blocked: dump limit switch active");
		return true;
	}

	if (!direction_dump && _cached_sensor_data.limit_switch_load) {
		PX4_DEBUG("Movement blocked: load limit switch active");
		return true;
	}

	return false;
}

bool TiltHardwareInterface::is_timestamp_valid(hrt_abstime timestamp, hrt_abstime timeout_us) const
{
	if (timestamp == 0) {
		return false;
	}

	hrt_abstime now = hrt_absolute_time();
	return (now - timestamp) < timeout_us;
}
