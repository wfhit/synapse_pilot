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

#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/hbridge_setpoint.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/sensor_limit_switch.h>
#include <uORB/topics/sensor_mag_encoder.h>

/**
 * @brief Unified hardware interface for boom control system
 *
 * Abstracts all hardware interactions for boom control:
 * - AS5600 magnetic encoder position feedback
 * - H-bridge motor control and status monitoring
 * - Sensor validation and fault detection
 * - Hardware health monitoring
 * - Parameter management
 *
 * This interface combines sensor and actuator management into a single
 * cohesive hardware abstraction layer for better maintainability.
 */
class BoomHardwareInterface : public ModuleParams
{
public:
	static constexpr hrt_abstime SENSOR_TIMEOUT_US = 500000;  // 500ms
	static constexpr hrt_abstime STATUS_TIMEOUT_US = 100000;  // 100ms
	static constexpr uint32_t MAX_FAULT_COUNT = 5;

	/**
	 * @brief Consolidated sensor data structure
	 */
	struct SensorData {
		float raw_angle;           // degrees - Raw encoder reading
		float calibrated_angle;    // degrees - After calibration
		bool is_valid;            // Sensor data validity
		bool magnet_detected;     // Magnet presence
		uint8_t status_flags;     // Sensor status flags
		bool limit_up_active;     // Boom up limit switch active
		bool limit_down_active;   // Boom down limit switch active
		bool limits_valid;        // Limit switches data validity
		hrt_abstime timestamp;    // Data timestamp
	};

	/**
	 * @brief H-bridge setpoint structure
	 */
	struct HbridgeSetpoint {
		float duty_cycle;         // -1 to 1
		bool enable;             // Enable motor
		uint8_t mode;            // Control mode
	};

	/**
	 * @brief H-bridge status structure
	 */
	struct HbridgeStatus {
		bool enabled;            // Motor enabled
		hrt_abstime timestamp;  // Status timestamp
		// Note: fault, current, voltage, temperature not available in hbridge_status_s
	};

	explicit BoomHardwareInterface(ModuleParams *parent);

	/**
	 * @brief Initialize hardware interface
	 * @param encoder_instance AS5600 instance to use
	 * @param motor_instance H-bridge instance to use
	 * @return True if initialization successful
	 */
	bool initialize(int encoder_instance, int motor_instance);

	/**
	 * @brief Update sensor readings
	 * @param data Output sensor data structure
	 * @return True if new valid data available
	 */
	bool update_sensors(SensorData &data);

	/**
	 * @brief Send H-bridge command
	 * @param command Command to send
	 * @return True if command sent successfully
	 */
	bool send_command(const HbridgeSetpoint &command);

	/**
	 * @brief Update H-bridge status
	 * @param status Output status structure
	 * @return True if new status available
	 */
	bool update_status(HbridgeStatus &status);

	/**
	 * @brief Emergency stop - disable motor immediately
	 */
	void emergency_stop();

	/**
	 * @brief Perform sensor calibration
	 * @param reference_angle Known reference angle in degrees
	 * @return True if calibration successful
	 */
	bool calibrate_sensor(float reference_angle);

	/**
	 * @brief Reset sensor to default calibration
	 */
	void reset_sensor_calibration();

	/**
	 * @brief Check overall hardware health
	 * @return True if all hardware is healthy
	 */
	bool is_healthy() const;

	/**
	 * @brief Get time since last valid sensor reading
	 * @return Time in microseconds
	 */
	hrt_abstime time_since_last_update() const;

	/**
	 * @brief Get last command sent
	 * @return Last H-bridge command
	 */
	HbridgeSetpoint get_last_command() const { return _last_command; }

	/**
	 * @brief Update hardware interface parameters from parameter system
	 * This should be called when parameters change to reconfigure the hardware interface
	 */
	void update_parameters();

private:
	// Hardware configuration
	int _encoder_instance{-1};
	int _motor_instance{-1};
	int _hbridge_selected{-1};
	bool _initialized{false};

	// Sensor configuration
	float _calibration_scale{1.0f};
	float _calibration_offset{0.0f};
	bool _angle_reversed{false};

	// Limit sensor configuration
	int _limit_up_instance{-1};
	int _limit_down_instance{-1};
	int _limit_up_selected{-1};
	int _limit_down_selected{-1};
	hrt_abstime _last_limit_update{0};
	bool _limits_healthy{false};

	// State tracking
	SensorData _last_sensor_data{};
	HbridgeSetpoint _last_command{};
	HbridgeStatus _last_status{};
	hrt_abstime _last_sensor_time{0};
	hrt_abstime _last_encoder_update{0};
	hrt_abstime _last_command_time{0};
	hrt_abstime _last_status_time{0};

	// Health monitoring
	bool _sensor_healthy{false};
	bool _actuator_healthy{false};
	uint32_t _fault_count{0};

	// uORB interface
	uORB::Subscription _mag_encoder_sub{ORB_ID(sensor_mag_encoder)};
	uORB::SubscriptionMultiArray<hbridge_status_s, 4> _hbridge_status_sub{ORB_ID::hbridge_status};
	uORB::SubscriptionMultiArray<sensor_limit_switch_s, 8> _limit_sensor_sub{ORB_ID::sensor_limit_switch};
	orb_advert_t _hbridge_command_pub{nullptr};

	// Hardware parameters
	DEFINE_PARAMETERS(
		// Encoder parameters
		(ParamInt<px4::params::BOOM_ENC_INST>) _param_encoder_instance,
		(ParamFloat<px4::params::BOOM_ENC_SCALE>) _param_encoder_scale,
		(ParamFloat<px4::params::BOOM_ENC_OFF>) _param_encoder_offset,
		(ParamInt<px4::params::BOOM_ENC_REV>) _param_angle_reverse,
		// Motor parameters
		(ParamInt<px4::params::BOOM_MOTOR_IDX>) _param_motor_index,
		(ParamFloat<px4::params::BOOM_CUR_LIM>) _param_current_limit,
		(ParamFloat<px4::params::BOOM_DUTY_MAX>) _param_duty_max,
		// Limit sensor parameters
		(ParamInt<px4::params::BOOM_LMT_UP_IDX>) _param_limit_up_index,
		(ParamInt<px4::params::BOOM_LMT_DN_IDX>) _param_limit_down_index
	)

	/**
	 * @brief Select best limit sensor instances
	 * @return True if both up and down limit instances found
	 */
	bool select_limit_sensor_instances();

	/**
	 * @brief Update limit switch states
	 * @param data Output sensor data
	 * @return True if limit switches updated successfully
	 */
	bool update_limit_switches(SensorData &data);

	/**
	 * @brief Update encoder data from AS5600 magnetic encoder
	 * @param data Output sensor data
	 * @return True if encoder data updated successfully
	 */
	bool update_encoder_data(SensorData &data);

	/**
	 * @brief Check if limit switch prevents movement in given direction
	 * @param direction_up True for up movement, false for down
	 * @return True if movement is blocked by limit switch
	 */
	bool is_movement_blocked_by_limits(bool direction_up) const;

	/**
	 * @brief Select best H-bridge instance
	 * @return Selected instance index or -1 if none found
	 */
	int select_hbridge_instance();

	/**
	 * @brief Apply sensor calibration to raw angle
	 * @param raw_angle Raw angle in degrees
	 * @return Calibrated angle in degrees
	 */
	float apply_sensor_calibration(float raw_angle) const;

	/**
	 * @brief Validate sensor data
	 * @param data Sensor data to validate
	 * @return True if data is valid
	 */
	bool validate_sensor_data(const SensorData &data) const;

	/**
	 * @brief Apply current limiting to H-bridge command
	 * @param command Command to limit
	 * @param current Current motor current
	 * @return Limited command
	 */
	HbridgeSetpoint apply_current_limit(const HbridgeSetpoint &command, float current) const;

	/**
	 * @brief Check if timestamp is still valid
	 * @param timestamp Timestamp to check
	 * @param timeout_us Timeout in microseconds
	 * @return True if timestamp is fresh
	 */
	bool is_timestamp_valid(hrt_abstime timestamp, hrt_abstime timeout_us) const;
};
