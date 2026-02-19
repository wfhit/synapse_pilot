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

// System includes
#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/hbridge_setpoint.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/sensor_mag_encoder.h>
#include <uORB/topics/sensor_limit_switch.h>
#include <uORB/topics/sensor_quad_encoder_reset.h>

/**
 * @brief Abstracts all hardware interactions for tilt control
 *
 * This class provides a clean interface to all tilt-related hardware:
 * - Hbridge motor control
 * - Position encoder feedback
 * - Boom angle sensor
 * - Limit switches
 * - Hardware health monitoring
 */
class TiltHardwareInterface : public ModuleParams
{
public:
	/**
	 * @brief Consolidated sensor data structure
	 */
	struct SensorData {
		float hbridge_position;       // mm
		float hbridge_velocity;       // mm/s
		float sensor_angle;           // rad - Raw AS5600 sensor angle
		bool sensor_angle_valid;      // True if sensor angle is valid
		bool limit_switch_load;       // Load limit (tilt down)
		bool limit_switch_dump;       // Dump limit (tilt up)
		bool motor_fault;             // Motor fault status
		bool encoder_fault;           // Encoder fault status
		hrt_abstime timestamp;        // Data timestamp
	};

	/**
	 * @brief Hbridge setpoint structure
	 */
	struct HbridgeSetpoint {
		float duty_cycle;             // -1.0 to 1.0
		bool enable;                  // Enable motor
		uint8_t control_mode;         // Control mode flag
	};

	explicit TiltHardwareInterface(ModuleParams *parent);

	/**
	 * @brief Initialize hardware interface
	 * @param motor_index Motor instance index
	 * @param encoder_index Encoder instance index
	 * @return True if initialization successful
	 */
	bool initialize(uint8_t motor_index, uint8_t encoder_index);

	/**
	 * @brief Update all sensor readings
	 * @param data Output sensor data structure
	 * @return True if data is valid and fresh
	 */
	bool update_sensors(SensorData &data);

	/**
	 * @brief Send command to hbridge motor
	 * @param command Hbridge command to send
	 * @return True if command sent successfully
	 */
	bool send_hbridge_setpoint(const HbridgeSetpoint &command);

	/**
	 * @brief Perform hardware self-test
	 * @return True if all hardware passes self-test
	 */
	bool perform_self_test();

	/**
	 * @brief Emergency stop - immediately disable motor
	 */
	void emergency_stop();

	/**
	 * @brief Reset encoder to zero position
	 * @return True if reset command sent successfully
	 */
	bool reset_encoder();

	/**
	 * @brief Get overall hardware health status
	 * @return True if all hardware is healthy
	 */
	bool is_healthy() const { return _is_healthy; }

	/**
	 * @brief Get detailed hardware status
	 * @param motor_enabled Output: motor enabled status
	 * @param encoder_valid Output: encoder validity
	 * @param limits_valid Output: limit switches validity
	 * @return True if status retrieved successfully
	 */
	bool get_hardware_status(bool &motor_enabled, bool &encoder_valid, bool &limits_valid) const;

	/**
	 * @brief Update hardware interface parameters from parameter system
	 * This should be called when parameters change to reconfigure the hardware interface
	 */
	void update_parameters();

private:
	// Hardware configuration
	uint8_t _motor_index{0};
	uint8_t _encoder_index{0};
	uint8_t _limit_load_index{0};
	uint8_t _limit_dump_index{0};

	// Instance selection tracking (EKF2-style)
	int _encoder_selected{-1};
	int _hbridge_status_selected{-1};
	int _limit_load_selected{-1};
	int _limit_dump_selected{-1};

	// Health monitoring
	bool _is_healthy{false};
	hrt_abstime _last_update{0};
	hrt_abstime _last_encoder_update{0};
	hrt_abstime _last_hbridge_update{0};
	hrt_abstime _last_limit_update{0};

	// Sensor data caching for performance
	SensorData _cached_sensor_data{};
	bool _sensor_data_valid{false};

	// Encoder calibration
	float _encoder_zero_offset{0.0f};
	float _encoder_scale_factor{1.0f};

	// uORB subscriptions (multi-instance)
	uORB::SubscriptionMultiArray<sensor_quad_encoder_s> _encoder_sub{ORB_ID::sensor_quad_encoder};
	uORB::SubscriptionMultiArray<hbridge_status_s> _hbridge_status_sub{ORB_ID::hbridge_status};
	uORB::SubscriptionMultiArray<sensor_limit_switch_s> _limit_sensor_sub{ORB_ID::sensor_limit_switch};
	uORB::Subscription _mag_encoder_sub{ORB_ID(sensor_mag_encoder)};

	// uORB publications
	uORB::PublicationMulti<hbridge_setpoint_s> _hbridge_command_pub{ORB_ID(hbridge_setpoint)};
	uORB::Publication<sensor_quad_encoder_reset_s> _encoder_reset_pub{ORB_ID(sensor_quad_encoder_reset)};

	// Timeout constants
	static constexpr hrt_abstime SENSOR_TIMEOUT_US = 100000;      // 100ms
	static constexpr hrt_abstime HEALTH_CHECK_INTERVAL_US = 1000000; // 1s

	/**
	 * @brief Select best encoder instance using EKF2 pattern
	 * @return Selected instance index or -1 if none found
	 */
	int select_encoder_instance();

	/**
	 * @brief Select best hbridge status instance
	 * @return Selected instance index or -1 if none found
	 */
	int select_hbridge_instance();

	/**
	 * @brief Select best limit sensor instances
	 * @param load_instance Output: load limit instance
	 * @param dump_instance Output: dump limit instance
	 * @return True if both instances found
	 */
	bool select_limit_sensor_instances(int &load_instance, int &dump_instance);

	/**
	 * @brief Update encoder data from selected instance
	 * @param data Output sensor data
	 * @return True if encoder data updated
	 */
	bool update_encoder_data(SensorData &data);

	/**
	 * @brief Update boom angle from magnetic encoder
	 * @param data Output sensor data
	 * @return True if boom angle updated
	 */
	bool update_boom_angle(SensorData &data);

	/**
	 * @brief Update limit switch states
	 * @param data Output sensor data
	 * @return True if limit switches updated
	 */
	bool update_limit_switches(SensorData &data);

	/**
	 * @brief Update motor status from hbridge
	 * @param data Output sensor data
	 * @return True if motor status updated
	 */
	bool update_motor_status(SensorData &data);

	/**
	 * @brief Check if limit switch prevents movement in given direction
	 * @param direction_dump True for dump movement (tilt up), false for load movement (tilt down)
	 * @return True if movement is blocked by limit switch
	 */
	bool is_movement_blocked_by_limits(bool direction_dump) const;

	/**
	 * @brief Check if timestamp is still valid
	 * @param timestamp Timestamp to check
	 * @param timeout_us Timeout in microseconds
	 * @return True if timestamp is fresh
	 */
	bool is_timestamp_valid(hrt_abstime timestamp, hrt_abstime timeout_us) const;

	// Parameters for hardware configuration
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BCT_MOTOR_IDX>) _param_motor_index,
		(ParamInt<px4::params::BCT_ENC_IDX>) _param_encoder_index,
		(ParamInt<px4::params::BCT_LMT_LOAD_IDX>) _param_limit_load_index,
		(ParamInt<px4::params::BCT_LMT_DUMP_IDX>) _param_limit_dump_index,
		(ParamFloat<px4::params::BCT_ENC_SCALE>) _param_encoder_scale,
		(ParamFloat<px4::params::BCT_ENC_OFFSET>) _param_encoder_offset,
		(ParamFloat<px4::params::BCT_HBG_MIN_LEN>) _param_hbridge_min_length,
		(ParamFloat<px4::params::BCT_HBG_MAX_LEN>) _param_hbridge_max_length
	)
};
