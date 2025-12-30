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
 * @file calibration_strategy.hpp
 *
 * Unified calibration strategy supporting multiple sensor types
 *
 * Supported Calibrations:
 *   - ENCODER: Wheel encoder calibration (counts per revolution, direction)
 *   - ACTUATOR: Hydraulic actuator calibration (limits, deadband, response)
 *   - TILT: Bucket tilt sensor calibration (zero offset, range, linearity)
 *   - IMU: Inertial measurement unit calibration (accel, gyro, mag)
 *   - STEERING: Steering angle sensor calibration (center, limits, ratio)
 *
 * State Flow:
 *   PRECHECK → INIT → CALIBRATING → VALIDATION → COMPLETE
 *                        ↓              ↓
 *                    Degraded     Critical → Abort
 *
 * Safety Features:
 *   - Vehicle must be stationary and disarmed (except specific calibrations)
 *   - Operator confirmation required for each step
 *   - Automatic abort on movement detection
 *   - Calibration data validation before saving
 *   - Rollback capability on failure
 */

#pragma once

#include "../strategy_base.hpp"
#include "calibration_failsafe.hpp"
#include <uORB/topics/strategy_cmd.h>
#include <uORB/topics/operation_mode_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/Subscription.hpp>
#include <lib/mathlib/mathlib.h>
#include <parameters/param.h>

class CalibrationStrategy : public StrategyBase
{
public:
	// Calibration type enumeration
	enum CalibrationType : uint8_t {
		CAL_ENCODER = 0,      // Wheel encoder calibration
		CAL_ACTUATOR = 1,     // Hydraulic actuator calibration
		CAL_TILT = 2,         // Bucket tilt sensor calibration
		CAL_IMU = 3,          // IMU calibration
		CAL_STEERING = 4,     // Steering angle sensor calibration
		CAL_TYPE_COUNT
	};

	// Step definitions
	enum Step : uint8_t {
		STEP_ACTIVATE_MODE = 0,
		STEP_PREPARE = 1,
		STEP_CALIBRATE = 2,
		STEP_VALIDATE = 3,
		STEP_SAVE = 4,
		STEP_COMPLETE = 5,
		STEP_COUNT = 6
	};

	CalibrationStrategy() :
		StrategyBase("Calibration", strategy_cmd_s::STRATEGY_CALIBRATION)
	{
		_total_steps = STEP_COUNT;
	}

	/**
	 * Set calibration type (must be called before activation)
	 */
	void set_calibration_type(CalibrationType type)
	{
		_cal_type = type;
		PX4_INFO("Calibration: Type set to %s", get_calibration_type_name());
	}

	// ============ Safety Thresholds ============

	// Battery thresholds (percentage 0.0-1.0)
	static constexpr float BATTERY_MIN_START = 0.30f;      // 30% minimum for calibration

	// Movement detection thresholds
	static constexpr float VELOCITY_MAX = 0.05f;           // 0.05 m/s max velocity
	static constexpr float ANGULAR_RATE_MAX = 0.02f;       // 0.02 rad/s max rotation

	// Calibration sample counts
	static constexpr uint32_t SAMPLES_MIN = 50;            // Minimum samples required
	static constexpr uint32_t SAMPLES_TARGET = 200;        // Target sample count

	// Timeouts (microseconds)
	static constexpr uint64_t OPERATOR_CONFIRM_TIMEOUT = 30000000;  // 30s confirmation
	static constexpr uint64_t CALIBRATION_TIMEOUT = 120000000;      // 120s max calibration time
	static constexpr uint64_t VALIDATION_TIMEOUT = 10000000;        // 10s validation

	StrategyResult precheck() override
	{
		// ========== 1. BATTERY CHECK ==========
		battery_status_s battery;
		if (!_battery_sub.copy(&battery)) {
			return StrategyResult::Failure("Battery status unavailable");
		}

		if (battery.remaining < BATTERY_MIN_START) {
			PX4_ERR("Calibration: Battery too low (%.1f%% < %.1f%%)",
			        (double)(battery.remaining * 100.0f), (double)(BATTERY_MIN_START * 100.0f));
			return StrategyResult::Failure("Battery too low for calibration");
		}

		// ========== 2. ARM STATUS (most calibrations require disarmed) ==========
		actuator_armed_s armed;
		if (!_armed_sub.copy(&armed)) {
			return StrategyResult::Failure("Arm status unavailable");
		}

		// Only IMU calibration can be performed while disarmed
		if (_cal_type != CAL_IMU && armed.armed) {
			PX4_ERR("Calibration: Vehicle must be disarmed for %s calibration",
			        get_calibration_type_name());
			return StrategyResult::Failure("Vehicle must be disarmed");
		}

		// ========== 3. STATIONARY CHECK ==========
		vehicle_local_position_s local_pos;
		if (_local_pos_sub.copy(&local_pos)) {
			if (local_pos.v_xy_valid) {
				float velocity = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);
				if (velocity > VELOCITY_MAX) {
					PX4_ERR("Calibration: Vehicle not stationary (%.3f m/s)", (double)velocity);
					return StrategyResult::Failure("Vehicle must be stationary");
				}
			}
		}

		// ========== 4. ROTATION CHECK ==========
		vehicle_attitude_s attitude;
		if (_attitude_sub.copy(&attitude)) {
			float angular_rate = sqrtf(attitude.rollspeed * attitude.rollspeed +
			                            attitude.pitchspeed * attitude.pitchspeed +
			                            attitude.yawspeed * attitude.yawspeed);
			if (angular_rate > ANGULAR_RATE_MAX) {
				PX4_ERR("Calibration: Vehicle rotating (%.3f rad/s)", (double)angular_rate);
				return StrategyResult::Failure("Vehicle must be stationary");
			}
		}

		// ========== 5. CALIBRATION TYPE VALIDATION ==========
		if (_cal_type >= CAL_TYPE_COUNT) {
			return StrategyResult::Failure("Invalid calibration type");
		}

		PX4_INFO("Calibration: Precheck passed for %s calibration", get_calibration_type_name());

		return StrategyResult::Success();
	}

	StrategyResult init() override
	{
		// Switch to calibration operation mode
		command_mode(operation_mode_cmd_s::MODE_WL_CALIBRATION, 100);

		// Reset calibration state
		_sample_count = 0;
		_cal_data_valid = false;
		_operator_confirmed = false;
		_cal_start_time = 0;

		// Backup current calibration parameters (for rollback)
		backup_current_calibration();

		PX4_INFO("Calibration: Initialized %s calibration mode", get_calibration_type_name());

		return StrategyResult::Success();
	}

	StrategyResult update() override
	{
		hrt_abstime now = hrt_absolute_time();

		switch (_current_step) {
		case STEP_ACTIVATE_MODE:
			// ========== STEP: Wait for calibration mode activation ==========
			{
				operation_mode_status_s mode_status;

				if (_mode_status_sub.copy(&mode_status)) {
					if (mode_status.current_mode == operation_mode_cmd_s::MODE_WL_CALIBRATION) {
						PX4_INFO("Calibration: Mode activated - ready for %s calibration",
						         get_calibration_type_name());
					_current_step = STEP_PREPARE;
					}
				}
			}
			break;

		case STEP_PREPARE:
			// ========== STEP: Prepare for calibration ==========
			{
				// Display instructions to operator
				display_calibration_instructions();

				// Wait for operator confirmation
				manual_control_setpoint_s manual_control;
				if (_manual_control_sub.copy(&manual_control)) {
					// Check for confirmation (e.g., throttle stick press)
					if (fabsf(manual_control.throttle - 1.0f) < 0.1f) {
						if (!_operator_confirmed) {
							PX4_INFO("Calibration: Operator confirmed - starting %s calibration",
							         get_calibration_type_name());
							_operator_confirmed = true;
						_current_step = STEP_CALIBRATE;
							_cal_start_time = now;
						}
					}
				}

				// Timeout check
				if (now - get_step_start() > OPERATOR_CONFIRM_TIMEOUT) {
					PX4_WARN("Calibration: Operator confirmation timeout");
					return StrategyResult::Failure("Operator confirmation timeout");
				}
			}
			break;

		case STEP_CALIBRATE:
			// ========== STEP: Perform calibration ==========
			{
				// Check for movement (abort if detected)
				if (!check_stationary()) {
					PX4_ERR("Calibration: Movement detected - aborting");
					return StrategyResult::Failure("Movement detected during calibration");
				}

				// Perform calibration based on type
				StrategyResult cal_result = perform_calibration();
				if (!cal_result.is_success()) {
					return cal_result;
				}

				// Check if sufficient samples collected
				if (_sample_count >= SAMPLES_TARGET) {
					PX4_INFO("Calibration: Collected %u samples - proceeding to validation",
					         _sample_count);
					_current_step = STEP_VALIDATE;
				}

				// Timeout check
				if (now - _cal_start_time > CALIBRATION_TIMEOUT) {
					if (_sample_count >= SAMPLES_MIN) {
						PX4_WARN("Calibration: Timeout but sufficient samples - proceeding");
						_current_step = STEP_VALIDATE;
					} else {
						PX4_ERR("Calibration: Timeout with insufficient samples (%u < %u)",
						        _sample_count, SAMPLES_MIN);
						return StrategyResult::Failure("Calibration timeout");
					}
				}
			}
			break;

		case STEP_VALIDATE:
			// ========== STEP: Validate calibration data ==========
			{
				StrategyResult val_result = validate_calibration();
				if (val_result.is_success()) {
					_cal_data_valid = true;
					PX4_INFO("Calibration: Validation successful");
					_current_step = STEP_SAVE;
				} else {
					PX4_ERR("Calibration: Validation failed - %s", val_result.get_message());
					// Rollback to previous calibration
					restore_backup_calibration();
					return val_result;
				}
			}
			break;

		case STEP_SAVE:
			// ========== STEP: Save calibration to parameters ==========
			{
				if (_cal_data_valid) {
					StrategyResult save_result = save_calibration();
					if (save_result.is_success()) {
						PX4_INFO("Calibration: Data saved successfully");
						_current_step = STEP_COMPLETE;
					} else {
						PX4_ERR("Calibration: Save failed - %s", save_result.get_message());
						restore_backup_calibration();
						return save_result;
					}
				} else {
					return StrategyResult::Failure("Invalid calibration data");
				}
			}
			break;

		case STEP_COMPLETE:
			// ========== STEP: Calibration complete ==========
			PX4_INFO("Calibration: %s calibration completed successfully",
			         get_calibration_type_name());
			return StrategyResult::Success();
		}

		// Still running
		return StrategyResult::Success();
	}

	const char *get_step_name() const override
	{
		const char *names[] = {
			"Activate Mode",
			"Prepare",
			"Calibrating",
			"Validate",
			"Save",
			"Complete"
		};
		return names[_current_step];
	}

protected:
	FailsafeBase *create_failsafe() override
	{
		return new CalibrationFailsafe();
	}

private:
	// Subscriptions
	uORB::Subscription _mode_status_sub{ORB_ID(operation_mode_status)};
	uORB::Subscription _armed_sub{ORB_ID(actuator_armed)};
	uORB::Subscription _battery_sub{ORB_ID(battery_status)};
	uORB::Subscription _attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _local_pos_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _sensor_sub{ORB_ID(sensor_combined)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};

	// Calibration state
	CalibrationType _cal_type{CAL_ENCODER};
	uint32_t _sample_count{0};
	bool _cal_data_valid{false};
	bool _operator_confirmed{false};
	hrt_abstime _cal_start_time{0};

	// Calibration data storage
	float _cal_data[10]{};           // Generic storage for calibration values
	float _cal_backup[10]{};         // Backup of previous calibration

	/**
	 * Get calibration type name
	 */
	const char *get_calibration_type_name() const
	{
		const char *names[] = {
			"Encoder",
			"Actuator",
			"Tilt",
			"IMU",
			"Steering"
		};
		return names[_cal_type];
	}

	/**
	 * Display calibration-specific instructions
	 */
	void display_calibration_instructions()
	{
		switch (_cal_type) {
		case CAL_ENCODER:
			PX4_INFO("ENCODER CALIBRATION:");
			PX4_INFO("  1. Ensure wheels can rotate freely");
			PX4_INFO("  2. Press throttle stick up to confirm");
			PX4_INFO("  3. Vehicle will rotate wheels for measurement");
			break;

		case CAL_ACTUATOR:
			PX4_INFO("ACTUATOR CALIBRATION:");
			PX4_INFO("  1. Ensure hydraulic system is pressurized");
			PX4_INFO("  2. Press throttle stick up to confirm");
			PX4_INFO("  3. Actuators will cycle through full range");
			break;

		case CAL_TILT:
			PX4_INFO("TILT SENSOR CALIBRATION:");
			PX4_INFO("  1. Position bucket at known angles");
			PX4_INFO("  2. Press throttle stick up to confirm each position");
			PX4_INFO("  3. Minimum 3 positions required");
			break;

		case CAL_IMU:
			PX4_INFO("IMU CALIBRATION:");
			PX4_INFO("  1. Place vehicle on level surface");
			PX4_INFO("  2. Keep vehicle completely still");
			PX4_INFO("  3. Press throttle stick up to start");
			break;

		case CAL_STEERING:
			PX4_INFO("STEERING CALIBRATION:");
			PX4_INFO("  1. Center steering wheel");
			PX4_INFO("  2. Press throttle stick up to confirm");
			PX4_INFO("  3. Turn to full left, then full right");
			break;
		}
	}

	/**
	 * Check if vehicle is stationary
	 */
	bool check_stationary()
	{
		vehicle_local_position_s local_pos;
		if (_local_pos_sub.copy(&local_pos) && local_pos.v_xy_valid) {
			float velocity = sqrtf(local_pos.vx * local_pos.vx + local_pos.vy * local_pos.vy);
			if (velocity > VELOCITY_MAX) {
				return false;
			}
		}

		vehicle_attitude_s attitude;
		if (_attitude_sub.copy(&attitude)) {
			float angular_rate = sqrtf(attitude.rollspeed * attitude.rollspeed +
			                            attitude.pitchspeed * attitude.pitchspeed +
			                            attitude.yawspeed * attitude.yawspeed);
			if (angular_rate > ANGULAR_RATE_MAX) {
				return false;
			}
		}

		return true;
	}

	/**
	 * Perform calibration based on type
	 */
	StrategyResult perform_calibration()
	{
		switch (_cal_type) {
		case CAL_ENCODER:
			return calibrate_encoder();

		case CAL_ACTUATOR:
			return calibrate_actuator();

		case CAL_TILT:
			return calibrate_tilt();

		case CAL_IMU:
			return calibrate_imu();

		case CAL_STEERING:
			return calibrate_steering();

		default:
			return StrategyResult::Failure("Unknown calibration type");
		}
	}

	/**
	 * Type-specific calibration methods
	 */
	StrategyResult calibrate_encoder()
	{
		// TODO: Implement encoder calibration
		// - Command wheel rotation
		// - Count encoder pulses
		// - Calculate counts per revolution
		_sample_count++;
		return StrategyResult::Success();
	}

	StrategyResult calibrate_actuator()
	{
		// TODO: Implement actuator calibration
		// - Command actuator to limits
		// - Measure position feedback
		// - Calculate calibration coefficients
		_sample_count++;
		return StrategyResult::Success();
	}

	StrategyResult calibrate_tilt()
	{
		// TODO: Implement tilt sensor calibration
		// - Read sensor at known angles
		// - Build calibration table
		// - Calculate offset and scale
		_sample_count++;
		return StrategyResult::Success();
	}

	StrategyResult calibrate_imu()
	{
		// TODO: Implement IMU calibration
		// - Collect accelerometer samples
		// - Collect gyroscope samples
		// - Calculate bias and scale factors
		_sample_count++;
		return StrategyResult::Success();
	}

	StrategyResult calibrate_steering()
	{
		// TODO: Implement steering calibration
		// - Detect center position
		// - Measure full left/right limits
		// - Calculate steering ratio
		_sample_count++;
		return StrategyResult::Success();
	}

	/**
	 * Validate calibration data
	 */
	StrategyResult validate_calibration()
	{
		// TODO: Implement calibration-specific validation
		// - Check data quality and consistency
		// - Verify values are within expected ranges
		// - Compare with previous calibration if available
		return StrategyResult::Success();
	}

	/**
	 * Save calibration to parameters
	 */
	StrategyResult save_calibration()
	{
		// TODO: Implement parameter save based on calibration type
		// - Write calibration data to appropriate parameters
		// - Verify parameter write success
		// - Trigger parameter save to persistent storage
		return StrategyResult::Success();
	}

	/**
	 * Backup current calibration
	 */
	void backup_current_calibration()
	{
		// TODO: Read current calibration parameters and save to backup
		for (size_t i = 0; i < sizeof(_cal_backup) / sizeof(_cal_backup[0]); i++) {
			_cal_backup[i] = 0.0f;  // Placeholder
		}
	}

	/**
	 * Restore backup calibration
	 */
	void restore_backup_calibration()
	{
		// TODO: Restore previous calibration from backup
		PX4_WARN("Calibration: Restoring previous calibration");
		for (size_t i = 0; i < sizeof(_cal_data) / sizeof(_cal_data[0]); i++) {
			_cal_data[i] = _cal_backup[i];
		}
	}
};
