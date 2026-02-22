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
 * @file st3215_servo.hpp
 * @author PX4 Development Team
 *
 * Simplified ST3215 smart servo driver header
 * Uses SerialPort API like UWB SR150 for robust UART communication
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/robotic_servo_setpoint.h>
#include <uORB/topics/robotic_servo_status.h>
#include <uORB/topics/sensor_limit_switch.h>

#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

using namespace time_literals;

class ST3215Servo : public ModuleBase<ST3215Servo>,
	public ModuleParams
{
public:
	ST3215Servo(const char *serial_port = "/dev/ttyS1");
	~ST3215Servo() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	void run();

	// Add SMS_STS style utility functions
	bool wheel_mode(uint8_t servo_id);
	bool write_speed(uint8_t servo_id, float speed_rad_s, uint8_t acc = 0);
	bool unlock_eprom(uint8_t servo_id);
	bool lock_eprom(uint8_t servo_id);
	int read_moving(uint8_t servo_id);
	int read_mode(uint8_t servo_id);

	// Position calibration functions
	bool calibrate_middle_position_sts(uint8_t servo_id); // STS standard method using torque enable = 128

private:

	// Serial communication (simplified like test_serial)
	bool configure_port();
	bool send_packet(const uint8_t *data, uint8_t length);
	bool receive_packet(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms);
	uint8_t calculate_checksum(const uint8_t *data, uint8_t length);

	// Servo operations (simplified)
	bool ping_servo(uint8_t servo_id);
	bool write_position(uint8_t servo_id, float position_rad, float speed_rad_s);
	bool read_status(uint8_t servo_id);
	bool set_torque_enable(uint8_t servo_id, bool enable);

	// Helper functions
	bool read_register(uint8_t servo_id, uint8_t reg_addr, uint8_t *data, uint8_t length);
	bool write_register(uint8_t servo_id, uint8_t reg_addr, const uint8_t *data, uint8_t length);

	void process_message();
	void process_command_line();
	void publish_feedback();

	// Safety functions
	void process_limit_sensors();
	bool check_servo_safety(float goal_position);
	void emergency_stop();

	// Limit sensor utilities (HBridge pattern)
	uint8_t get_left_limit_id() const { return static_cast<uint8_t>(_param_left_limit.get()); }
	uint8_t get_right_limit_id() const { return static_cast<uint8_t>(_param_right_limit.get()); }

	// Serial port (simplified like test_serial)
	int _uart{-1};
	char _port_name[32];

	// uORB topics
	uORB::Subscription _servo_command_sub{ORB_ID(robotic_servo_setpoint)};
	uORB::Publication<robotic_servo_status_s> _servo_feedback_pub{ORB_ID(robotic_servo_status)};
	uORB::SubscriptionMultiArray<sensor_limit_switch_s, 4> _limit_sensor_sub{ORB_ID::sensor_limit_switch};

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _comms_error_perf;
	perf_counter_t _packet_count_perf;

	// Current servo state
	float _current_position{0.0f};     // Current position in radians
	float _current_speed{0.0f};        // Current speed in rad/s
	float _current_load{0.0f};         // Current load percentage
	uint8_t _current_temperature{0};   // Current temperature in degrees C
	uint8_t _current_voltage{0};       // Current voltage in 0.1V units
	bool _servo_enabled{false};        // Torque enable state
	bool _connection_ok{false};        // Communication status
	hrt_abstime _last_update_time{0};  // Last successful update time
	int _consecutive_errors{0};        // Count of consecutive communication errors

	// Safety state
	bool _safety_stop_active{false};   // Emergency stop due to limit sensor (legacy)
	uint8_t _active_limit_function{255}; // Which limit function is active (255 = none, legacy)

	// HBridge pattern limit sensor states
	bool _left_limit_active{false};    // Left rotation limit sensor state
	bool _right_limit_active{false};   // Right rotation limit sensor state

	// Command flags for process_command_line
	bool _flag_ping{false};
	bool _flag_wheel_mode{false};
	bool _flag_write_speed{false};
	bool _flag_unlock_eprom{false};
	bool _flag_lock_eprom{false};
	bool _flag_read_moving{false};
	bool _flag_read_mode{false};
	bool _flag_set_abs_position{false};
	bool _flag_set_rel_position{false};
	bool _flag_calibrate_middle_sts{false};

	// Command parameters
	float _cmd_speed{0.0f};
	uint8_t _cmd_acceleration{0};
	float _cmd_position{0.0f};
	float _cmd_position_speed{0.0f};

	// ST3215 Protocol constants
	static constexpr uint8_t ST3215_HEADER = 0xFF;
	static constexpr uint8_t ST3215_HEADER2 = 0xFF;

	// Instruction set
	static constexpr uint8_t ST3215_CMD_PING = 0x01;
	static constexpr uint8_t ST3215_CMD_READ = 0x02;
	static constexpr uint8_t ST3215_CMD_WRITE = 0x03;

	// Register addresses (based on SMS_STS reference)
	// EPROM (read only)
	static constexpr uint8_t ST3215_REG_MODEL_L = 3;
	static constexpr uint8_t ST3215_REG_MODEL_H = 4;

	// EPROM (read & write)
	static constexpr uint8_t ST3215_REG_ID = 5;
	static constexpr uint8_t ST3215_REG_BAUD_RATE = 6;
	static constexpr uint8_t ST3215_REG_MIN_ANGLE_LIMIT_L = 9;
	static constexpr uint8_t ST3215_REG_MIN_ANGLE_LIMIT_H = 10;
	static constexpr uint8_t ST3215_REG_MAX_ANGLE_LIMIT_L = 11;
	static constexpr uint8_t ST3215_REG_MAX_ANGLE_LIMIT_H = 12;
	static constexpr uint8_t ST3215_REG_POSITION_CALIBRATION_L = 31;
	static constexpr uint8_t ST3215_REG_POSITION_CALIBRATION_H = 32;
	static constexpr uint8_t ST3215_REG_MODE = 33;

	// SRAM (read & write)
	static constexpr uint8_t ST3215_REG_TORQUE_ENABLE = 40;
	static constexpr uint8_t ST3215_REG_ACC = 41;
	static constexpr uint8_t ST3215_REG_GOAL_POSITION_L = 42;
	static constexpr uint8_t ST3215_REG_GOAL_POSITION_H = 43;
	static constexpr uint8_t ST3215_REG_GOAL_TIME_L = 44;
	static constexpr uint8_t ST3215_REG_GOAL_TIME_H = 45;
	static constexpr uint8_t ST3215_REG_GOAL_SPEED_L = 46;
	static constexpr uint8_t ST3215_REG_GOAL_SPEED_H = 47;
	static constexpr uint8_t ST3215_REG_TORQUE_LIMIT_L = 48;
	static constexpr uint8_t ST3215_REG_TORQUE_LIMIT_H = 49;
	static constexpr uint8_t ST3215_REG_LOCK = 55;

	// SRAM (read only)
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION_L = 56;
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION_H = 57;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED_L = 58;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED_H = 59;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD_L = 60;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD_H = 61;
	static constexpr uint8_t ST3215_REG_PRESENT_VOLTAGE = 62;
	static constexpr uint8_t ST3215_REG_PRESENT_TEMP = 63;
	static constexpr uint8_t ST3215_REG_MOVING = 66;
	static constexpr uint8_t ST3215_REG_PRESENT_CURRENT_L = 69;
	static constexpr uint8_t ST3215_REG_PRESENT_CURRENT_H = 70;

	// Timing constants
	static constexpr uint32_t SCHEDULE_INTERVAL_US = 20000;  // 20ms = 50Hz
	static constexpr uint32_t PACKET_TIMEOUT_MS = 10;  // Changed to 10ms as requested

	static int run_trampoline(int argc, char *argv[]);

	// Module parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ST3215_ID>) _param_servo_id,
		(ParamInt<px4::params::ST3215_BAUDRATE>) _param_baudrate,
		(ParamFloat<px4::params::ST3215_MIN_POS>) _param_min_position,
		(ParamFloat<px4::params::ST3215_MAX_POS>) _param_max_position,
		(ParamFloat<px4::params::ST3215_MAX_SPD>) _param_max_speed,
		(ParamInt<px4::params::ST3215_LEFT_LMT>) _param_left_limit,
		(ParamInt<px4::params::ST3215_RIGHT_LMT>) _param_right_limit
	)
};
