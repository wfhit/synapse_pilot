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
 * @file st3215_servo.cpp
 * @author PX4 Development Team
 *
 * Simplified ST3215 smart servo driver implementation
 * Uses SerialPort API like UWB SR150 for robust UART communication
 */

#include "st3215_servo.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <cstring>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>

ST3215Servo::ST3215Servo(const char *serial_port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": loop")),
	_comms_error_perf(perf_alloc(PC_COUNT, MODULE_NAME": comm_err")),
	_packet_count_perf(perf_alloc(PC_COUNT, MODULE_NAME": packets"))
{
	strncpy(_port_name, serial_port, sizeof(_port_name) - 1);
	_port_name[sizeof(_port_name) - 1] = '\0';
}

ST3215Servo::~ST3215Servo()
{
	if (_uart >= 0) {
		::close(_uart);
		_uart = -1;
	}

	perf_free(_loop_perf);
	perf_free(_comms_error_perf);
	perf_free(_packet_count_perf);
}

bool ST3215Servo::init()
{
	// Start work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	PX4_INFO("ST3215 servo driver started on %s", _port_name);
	return true;
}

bool ST3215Servo::configure_port()
{
	// Close existing connection if open
	if (_uart >= 0) {
		::close(_uart);
		_uart = -1;
	}

	// Open serial port
	PX4_INFO("Opening serial port %s...", _port_name);
	_uart = ::open(_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart < 0) {
		PX4_ERR("Failed to open %s: %s", _port_name, strerror(errno));
		return false;
	}

	PX4_INFO("Serial port opened successfully (fd=%d)", _uart);

	// Configure port settings (similar to test_serial)
	struct termios uart_config;

	if (tcgetattr(_uart, &uart_config) != 0) {
		PX4_ERR("Error getting serial port attributes: %s", strerror(errno));
		::close(_uart);
		_uart = -1;
		return false;
	}

	// Get baudrate parameter (default to 115200)
	int32_t baudrate = _param_baudrate.get();
	if (baudrate <= 0) {
		baudrate = 115200;  // Default changed to 115200
	}

	PX4_INFO("Configuring baudrate: %ld", (long)baudrate);

	speed_t speed;
	switch (baudrate) {
	case 9600:    speed = B9600; break;
	case 19200:   speed = B19200; break;
	case 38400:   speed = B38400; break;
	case 57600:   speed = B57600; break;
	case 115200:  speed = B115200; break;
	case 230400:  speed = B230400; break;
	case 460800:  speed = B460800; break;
	case 921600:  speed = B921600; break;
	case 1000000: speed = B1000000; break;
	default:
		PX4_WARN("Unsupported baudrate: %ld, using 115200", (long)baudrate);
		speed = B115200;
		break;
	}

	cfsetospeed(&uart_config, speed);
	cfsetispeed(&uart_config, speed);

	// Configure port settings (8N1, no flow control)
	uart_config.c_cflag &= ~PARENB;  // No parity
	uart_config.c_cflag &= ~CSTOPB;  // One stop bit
	uart_config.c_cflag &= ~CSIZE;   // Clear size bits
	uart_config.c_cflag |= CS8;      // 8 data bits
	uart_config.c_cflag &= ~CRTSCTS; // No hardware flow control
	uart_config.c_cflag |= CREAD | CLOCAL; // Enable reading and ignore modem control lines

	uart_config.c_lflag &= ~ICANON;  // Non-canonical mode
	uart_config.c_lflag &= ~ECHO;    // No echo
	uart_config.c_lflag &= ~ECHOE;   // No echo erase
	uart_config.c_lflag &= ~ECHONL;  // No echo newline
	uart_config.c_lflag &= ~ISIG;    // No signal processing

	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	uart_config.c_oflag &= ~OPOST;   // No output processing
	uart_config.c_oflag &= ~ONLCR;   // No CR to NL translation

	// Set timeouts for blocking reads
	uart_config.c_cc[VTIME] = 1;     // Wait for up to 0.1s (1 decisecond)
	uart_config.c_cc[VMIN] = 0;      // No minimum number of characters

	if (tcsetattr(_uart, TCSANOW, &uart_config) != 0) {
		PX4_ERR("Error setting serial port attributes: %s", strerror(errno));
		::close(_uart);
		_uart = -1;
		return false;
	}

	// Clear any existing data
	tcflush(_uart, TCIOFLUSH);

	return true;
}

void ST3215Servo::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	perf_begin(_loop_perf);

	// Update parameters
	updateParams();

	// Configure serial port if not already configured
	if (_uart < 0) {
		if (!configure_port()) {
			PX4_WARN("Failed to configure serial port, will retry later");
			perf_end(_loop_perf);
			return;
		}
	}

	// Process limit sensors for safety
	process_limit_sensors();

	// Process incoming servo commands
	process_message();

	// Process command line commands
	process_command_line();

	// Read servo status periodically (every 50ms)
	static hrt_abstime last_status_read = 0;
	if (hrt_elapsed_time(&last_status_read) > 50_ms && _uart >= 0) {
		last_status_read = hrt_absolute_time();

		uint8_t servo_id = _param_servo_id.get();

		if (servo_id > 0 && read_status(servo_id)) {
			// Successful status read
			_connection_ok = true;
			_last_update_time = hrt_absolute_time();
			_consecutive_errors = 0;
			publish_feedback();
		} else {
			// Handle communication error
			_consecutive_errors++;
			perf_count(_comms_error_perf);
			PX4_WARN("Status read failed, consecutive errors: %d", _consecutive_errors);

			// Mark as disconnected after multiple errors or timeout
			if (_consecutive_errors >= 5 || hrt_elapsed_time(&_last_update_time) > 1_s) {
				_connection_ok = false;
				_consecutive_errors = 0;
			}
		}
	}

	perf_end(_loop_perf);
}

bool ST3215Servo::send_packet(const uint8_t *data, uint8_t length)
{
	if (_uart < 0 || !data || length == 0) {
		PX4_ERR("send_packet: Invalid parameters - uart=%d, data=%p, length=%d", _uart, data, length);
		return false;
	}

	// Clear input buffer before sending to avoid stale data
	tcflush(_uart, TCIFLUSH);

	// Send the packet
	ssize_t bytes_written = ::write(_uart, data, length);
	if (bytes_written != length) {
		PX4_ERR("Write failed: expected %d, wrote %d, error: %s", length, (int)bytes_written, strerror(errno));
		return false;
	}

	// Wait for transmission to complete
	tcdrain(_uart);
	return true;
}

bool ST3215Servo::receive_packet(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms)
{
	if (_uart < 0 || !buffer || buffer_size == 0) {
		PX4_ERR("receive_packet: Invalid parameters - uart=%d, buffer=%p, size=%zu", _uart, buffer, buffer_size);
		return false;
	}

	hrt_abstime start_time = hrt_absolute_time();
	size_t bytes_received = 0;

	// First, read at least the header (4 bytes: 0xFF 0xFF ID LENGTH)
	const size_t header_size = 4;

	while (bytes_received < header_size && bytes_received < buffer_size) {
		// Check for timeout
		if (hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			PX4_WARN("Header timeout after %lu ms, got %zu bytes", timeout_ms, bytes_received);
			return false;
		}

		ssize_t bytes_read = ::read(_uart, buffer + bytes_received, buffer_size - bytes_received);
		if (bytes_read > 0) {
			bytes_received += bytes_read;
		} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_ERR("Read error: %s", strerror(errno));
			return false;
		}

		// Small delay to prevent CPU spinning
		usleep(500);
	}

	// Check if we have valid header
	if (bytes_received < header_size) {
		PX4_WARN("Insufficient header bytes: got %zu, need %zu", bytes_received, header_size);
		return false;
	}

	// Validate header
	if (buffer[0] != ST3215_HEADER || buffer[1] != ST3215_HEADER2) {
		PX4_ERR("Invalid header: 0x%02X 0x%02X (expected 0xFF 0xFF)", buffer[0], buffer[1]);
		return false;
	}

	// Parse packet length from header
	uint8_t packet_length = buffer[3];

	// Calculate total expected packet size
	// Format: Header(2) + ID(1) + Length(1) + Data(Length) = 4 + Length
	// Where Data includes: Status(1) + Parameters(Length-2) + Checksum(1)
	size_t total_expected = 4 + packet_length;

	if (total_expected > buffer_size) {
		PX4_ERR("Packet too large: expected %zu bytes, buffer only %zu", total_expected, buffer_size);
		return false;
	}

	// Read remaining bytes
	while (bytes_received < total_expected && bytes_received < buffer_size) {
		if (hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			PX4_WARN("Data timeout after %lu ms, got %zu/%zu bytes", timeout_ms, bytes_received, total_expected);
			break;
		}

		ssize_t bytes_read = ::read(_uart, buffer + bytes_received, buffer_size - bytes_received);
		if (bytes_read > 0) {
			bytes_received += bytes_read;
		} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_ERR("Read error: %s", strerror(errno));
			return false;
		}
		usleep(500);
	}

	// Check if we got complete packet
	if (bytes_received < total_expected) {
		PX4_WARN("Incomplete packet: expected %zu, got %zu", total_expected, bytes_received);
		return false;
	}

	// Verify checksum
	// Checksum = ~(ID + Length + Status + Parameters...)
	// Calculate from buffer[2] (ID) for (packet_length + 1) bytes (ID + Length + Status + Parameters)
	uint8_t calculated_checksum = calculate_checksum(&buffer[2], packet_length + 1);
	uint8_t received_checksum = buffer[total_expected - 1];

	if (calculated_checksum == received_checksum) {
		return true;
	} else {
		PX4_ERR("Checksum mismatch: calc=0x%02X, recv=0x%02X", calculated_checksum, received_checksum);
		return false;
	}
}

bool ST3215Servo::read_register(uint8_t servo_id, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
	uint8_t packet[8] = {
		ST3215_HEADER,      // 0xFF
		ST3215_HEADER2,     // 0xFF
		servo_id,           // ID
		0x04,               // Length
		ST3215_CMD_READ,    // Instruction
		reg_addr,           // Starting address
		length,             // Number of bytes to read
		0x00                // Checksum (will be calculated)
	};
	packet[7] = calculate_checksum(&packet[2], 5);

	if (!send_packet(packet, 8)) {
		PX4_ERR("Failed to send read command");
		return false;
	}

	// Wait for response
	uint8_t response[32];
	if (receive_packet(response, sizeof(response), PACKET_TIMEOUT_MS)) {
		// Check if response is from correct servo
		if (response[2] == servo_id && response[3] >= (length + 2)) {

		// Copy data from response packet (skip header, ID, length, error)
		memcpy(data, &response[5], length);

		return true;
		} else {
			PX4_ERR("Invalid read response: servo_id=%d (expected %d), length=%d (expected >=%d)",
				response[2], servo_id, response[3], length + 2);
		}
	} else {
		PX4_ERR("No read response received");
	}

	return false;
}

bool ST3215Servo::write_register(uint8_t servo_id, uint8_t reg_addr, const uint8_t *data, uint8_t length)
{
	if (length > 20) {  // Safety check
		PX4_ERR("Write length too large: %d bytes (max 20)", length);
		return false;
	}

	uint8_t packet[32] = {
		ST3215_HEADER,              // 0xFF
		ST3215_HEADER2,             // 0xFF
		servo_id,                   // ID
		(uint8_t)(3 + length),      // Length (instruction + address + data)
		ST3215_CMD_WRITE,           // Instruction
		reg_addr                    // Starting address
	};

	// Copy data payload
	memcpy(&packet[6], data, length);

	// Calculate and add checksum
	packet[6 + length] = calculate_checksum(&packet[2], 4 + length);

	bool result = send_packet(packet, 7 + length);
	if (!result) {
		PX4_ERR("Failed to send write command");
	}

	return result;
}

uint8_t ST3215Servo::calculate_checksum(const uint8_t *data, uint8_t length)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < length; i++) {
		sum += data[i];
	}
	return ~sum;
}

bool ST3215Servo::ping_servo(uint8_t servo_id)
{
	PX4_INFO("Pinging servo %d", servo_id);

	uint8_t packet[6] = {
		ST3215_HEADER,      // 0xFF
		ST3215_HEADER2,     // 0xFF
		servo_id,           // ID
		0x02,               // Length
		ST3215_CMD_PING,    // PING command (0x01)
		0x00                // Checksum (will be calculated)
	};
	packet[5] = calculate_checksum(&packet[2], 3);

	if (!send_packet(packet, 6)) {
		PX4_ERR("Failed to send ping command");
		return false;
	}

	// Wait for response
	uint8_t response[16];
	if (receive_packet(response, sizeof(response), PACKET_TIMEOUT_MS)) {
		// Check if response is from correct servo and is a status response
		if (response[2] == servo_id && response[3] >= 2) {
			PX4_INFO("Ping successful - servo %d responded", servo_id);
			return true;
		} else {
			PX4_ERR("Invalid ping response: servo_id=%d (expected %d), length=%d",
				response[2], servo_id, response[3]);
		}
	} else {
		PX4_ERR("No ping response received from servo %d", servo_id);
	}

	return false;
}

bool ST3215Servo::write_position(uint8_t servo_id, float position_rad, float speed_rad_s)
{
	// Convert radians to servo position units
	float position_deg = position_rad * 180.0f / M_PI_F;

	// Convert to servo units (0-4095 maps to 0-360 degrees)
	// Handle negative positions like SMS_STS WritePosEx
	int16_t position_value = (int16_t)(position_deg * 4095.0f / 360.0f);
	uint16_t position_raw;

	if (position_value < 0) {
		position_raw = (-position_value) | (1 << 15); // Set sign bit for negative
	} else {
		position_raw = (uint16_t)position_value;
	}

	// Convert speed from rad/s to servo units
	// Based on SMS_STS implementation, speed range is 0-1023
	float speed_deg_s = fabsf(speed_rad_s) * 180.0f / M_PI_F;
	speed_deg_s = math::constrain(speed_deg_s, 0.0f, 360.0f); // Max ~6.28 rad/s
	uint16_t speed_raw = (uint16_t)(speed_deg_s * 1023.0f / 360.0f);

	// Write position and speed using the SMS_STS style (ACC + Position + Time + Speed)
	uint8_t data[7] = {
		0,                                      // ACC (acceleration, 0 = default)
		(uint8_t)(position_raw & 0xFF),        // Position L
		(uint8_t)((position_raw >> 8) & 0xFF), // Position H
		0, 0,                                   // Time L, H (0 = move immediately)
		(uint8_t)(speed_raw & 0xFF),           // Speed L
		(uint8_t)((speed_raw >> 8) & 0xFF)     // Speed H
	};

	// Retry once on failure
	if (!write_register(servo_id, ST3215_REG_ACC, data, 7)) {
		// Single retry
		usleep(1000); // 1ms delay
		return write_register(servo_id, ST3215_REG_ACC, data, 7);
	}
	return true;
}

bool ST3215Servo::read_status(uint8_t servo_id)
{
	// Read each register individually like SMS_STS.cpp does
	uint8_t pos_data[2], speed_data[2], load_data[2];
	uint8_t voltage_data[1], temp_data[1];

	// Read position (2 bytes) - like ReadPos in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_POSITION_L, pos_data, 2)) {
		PX4_ERR("Failed to read position register");
		return false;
	}

	// Read speed (2 bytes) - like ReadSpeed in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_SPEED_L, speed_data, 2)) {
		PX4_ERR("Failed to read speed register");
		return false;
	}

	// Read load (2 bytes) - like ReadLoad in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_LOAD_L, load_data, 2)) {
		PX4_ERR("Failed to read load register");
		return false;
	}

	// Read voltage (1 byte) - like ReadVoltage in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_VOLTAGE, voltage_data, 1)) {
		PX4_ERR("Failed to read voltage register");
		return false;
	}

	// Read temperature (1 byte) - like ReadTemper in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_TEMP, temp_data, 1)) {
		PX4_ERR("Failed to read temperature register");
		return false;
	}

	// Parse the response data according to SMS_STS register layout
	uint16_t position_raw = pos_data[0] | (pos_data[1] << 8);
	uint16_t speed_raw = speed_data[0] | (speed_data[1] << 8);
	uint16_t load_raw = load_data[0] | (load_data[1] << 8);
	_current_voltage = voltage_data[0];
	_current_temperature = temp_data[0];

	// Convert position to radians (0-4095 maps to 0-360 degrees)
	// SMS_STS ReadPos handles sign bit, but for position it's typically not used in normal operation
	uint16_t pos_value = position_raw;
	if (position_raw & (1 << 15)) {  // Handle negative position like SMS_STS
		pos_value = -(position_raw & ~(1 << 15));
	}
	float position_deg = (pos_value * 360.0f / 4095.0f);
	_current_position = position_deg * M_PI_F / 180.0f;

	// Convert speed to rad/s - handle sign bit according to SMS_STS
	float speed_deg_s = (speed_raw & ~(1 << 15)) * 360.0f / 1023.0f;  // Remove sign bit before conversion
	if (speed_raw & (1 << 15)) {  // Check sign bit
		speed_deg_s = -speed_deg_s;
	}
	_current_speed = speed_deg_s * M_PI_F / 180.0f;

	// Convert load - handle sign bit according to SMS_STS
	float load_value = (load_raw & ~(1 << 10)) / 10.0f; // Remove sign bit, 0-1000 -> 0-100%
	if (load_raw & (1 << 10)) {  // Check sign bit for load (bit 10, not 15)
		load_value = -load_value;
	}
	_current_load = load_value;

	return true;
}

bool ST3215Servo::set_torque_enable(uint8_t servo_id, bool enable)
{
	uint8_t value = enable ? 1 : 0;

	// Retry once on failure
	if (!write_register(servo_id, ST3215_REG_TORQUE_ENABLE, &value, 1)) {
		// Single retry
		usleep(1000); // 1ms delay
		return write_register(servo_id, ST3215_REG_TORQUE_ENABLE, &value, 1);
	}
	return true;
}

void ST3215Servo::process_message()
{
	robotic_servo_setpoint_s cmd;
	if (_servo_command_sub.update(&cmd)) {
		uint8_t servo_id = _param_servo_id.get();

		if (servo_id == 0) {
			return;  // Invalid servo ID
		}

		// Handle enable/disable
		if (cmd.torque_enable != _servo_enabled) {
			if (set_torque_enable(servo_id, cmd.torque_enable)) {
				_servo_enabled = cmd.torque_enable;
			}
		}

		// Send position command if servo is enabled and safe
		if (_servo_enabled && cmd.torque_enable) {
			// Check if movement is safe (not blocked by limit sensors)
			if (!check_servo_safety(cmd.goal_position)) {
				PX4_WARN("Servo movement blocked by limit sensor (function %d)", _active_limit_function);
				return;
			}

			float target_speed = fabsf(cmd.goal_velocity);

			// Use default speed if not specified
			if (target_speed < 0.01f) {
				target_speed = _param_max_speed.get();
				if (target_speed < 0.01f) {
					target_speed = 2.0f;  // Default 2 rad/s
				}
			}

			write_position(servo_id, cmd.goal_position, target_speed);
		}
	}
}

void ST3215Servo::process_command_line()
{
	// Check command flags and execute utility functions
	uint8_t servo_id = _param_servo_id.get();

	if (servo_id == 0) {
		// Clear all flags if servo ID is invalid
		_flag_ping = false;
		_flag_wheel_mode = false;
		_flag_write_speed = false;
		_flag_unlock_eprom = false;
		_flag_lock_eprom = false;
		_flag_read_moving = false;
		_flag_read_mode = false;
		_flag_set_abs_position = false;
		_flag_set_rel_position = false;
		_flag_calibrate_middle_sts = false;
		return;
	}

	// Execute ping command
	if (_flag_ping) {
		_flag_ping = false;
		if (ping_servo(servo_id)) {
			PX4_INFO("Servo %d ping successful", servo_id);
		} else {
			PX4_ERR("Failed to ping servo %d", servo_id);
		}
	}

	// Execute wheel_mode command
	if (_flag_wheel_mode) {
		_flag_wheel_mode = false;
		if (wheel_mode(servo_id)) {
			PX4_INFO("Servo %d switched to wheel mode", servo_id);
		} else {
			PX4_ERR("Failed to switch servo %d to wheel mode", servo_id);
		}
	}

	// Execute write_speed command
	if (_flag_write_speed) {
		_flag_write_speed = false;
		if (write_speed(servo_id, _cmd_speed, _cmd_acceleration)) {
			PX4_INFO("Servo %d speed set to %.2f rad/s (acc=%d)", servo_id, (double)_cmd_speed, _cmd_acceleration);
		} else {
			PX4_ERR("Failed to set servo %d speed", servo_id);
		}
	}

	// Execute unlock_eprom command
	if (_flag_unlock_eprom) {
		_flag_unlock_eprom = false;
		if (unlock_eprom(servo_id)) {
			PX4_INFO("Servo %d EPROM unlocked", servo_id);
		} else {
			PX4_ERR("Failed to unlock servo %d EPROM", servo_id);
		}
	}

	// Execute lock_eprom command
	if (_flag_lock_eprom) {
		_flag_lock_eprom = false;
		if (lock_eprom(servo_id)) {
			PX4_INFO("Servo %d EPROM locked", servo_id);
		} else {
			PX4_ERR("Failed to lock servo %d EPROM", servo_id);
		}
	}

	// Execute read_moving command
	if (_flag_read_moving) {
		_flag_read_moving = false;
		int moving = read_moving(servo_id);
		if (moving >= 0) {
			PX4_INFO("Servo %d moving status: %s", servo_id, moving ? "MOVING" : "STOPPED");
		} else {
			PX4_ERR("Failed to read servo %d moving status", servo_id);
		}
	}

	// Execute read_mode command
	if (_flag_read_mode) {
		_flag_read_mode = false;
		int mode = read_mode(servo_id);
		if (mode >= 0) {
			const char *mode_str = (mode == 0) ? "POSITION" : (mode == 1) ? "WHEEL" : "UNKNOWN";
			PX4_INFO("Servo %d mode: %s (%d)", servo_id, mode_str, mode);
		} else {
			PX4_ERR("Failed to read servo %d mode", servo_id);
		}
	}

	// Execute set_abs_position command
	if (_flag_set_abs_position) {
		_flag_set_abs_position = false;
		if (write_position(servo_id, _cmd_position, _cmd_position_speed)) {
			PX4_INFO("Servo %d moved to absolute position %.2f rad at %.2f rad/s", servo_id, (double)_cmd_position, (double)_cmd_position_speed);
		} else {
			PX4_ERR("Failed to set servo %d absolute position", servo_id);
		}
	}

	// Execute set_rel_position command
	if (_flag_set_rel_position) {
		_flag_set_rel_position = false;
		float target_position = _current_position + _cmd_position;
		if (write_position(servo_id, target_position, _cmd_position_speed)) {
			PX4_INFO("Servo %d moved relative %.2f rad to position %.2f rad at %.2f rad/s", servo_id, (double)_cmd_position, (double)target_position, (double)_cmd_position_speed);
		} else {
			PX4_ERR("Failed to set servo %d relative position", servo_id);
		}
	}

	// Execute calibrate_middle_sts command (STS standard method)
	if (_flag_calibrate_middle_sts) {
		_flag_calibrate_middle_sts = false;
		if (calibrate_middle_position_sts(servo_id)) {
			PX4_INFO("Servo %d STS middle position calibrated successfully", servo_id);
		} else {
			PX4_ERR("Failed to calibrate servo %d STS middle position", servo_id);
		}
	}
}

void ST3215Servo::publish_feedback()
{
	robotic_servo_status_s feedback{};
	feedback.timestamp = hrt_absolute_time();
	feedback.id = _param_servo_id.get();
	feedback.position = _current_position;
	feedback.velocity = _current_speed;
	feedback.load = _current_load / 100.0f;  // Convert percentage to -1.0 to 1.0
	feedback.temperature = _current_temperature;
	feedback.voltage = _current_voltage * 0.1f;  // Convert to volts
	feedback.torque_enabled = _servo_enabled;
	feedback.goal_position = _current_position;  // For now, use current position
	feedback.current = 0.0f;  // Not available from ST3215
	feedback.error_flags = _safety_stop_active ? (1 << 0) : 0;  // Bit 0 = safety stop active
	feedback.position_error = 0.0f;
	feedback.moving = (fabsf(_current_speed) > 0.01f);
	feedback.position_reached = true;  // Simplification for now
	feedback.hardware_alarm = 0;
	feedback.shutdown_alarm = _safety_stop_active ? 1 : 0;  // Report safety stop as shutdown alarm

	_servo_feedback_pub.publish(feedback);
}

int ST3215Servo::print_status()
{
	PX4_INFO("ST3215 Servo Driver Status:");
	PX4_INFO("  Port: %s", _port_name);
	PX4_INFO("  UART: %s (fd: %d)", (_uart >= 0) ? "Open" : "Closed", _uart);
	PX4_INFO("  Servo ID: %ld", _param_servo_id.get());
	PX4_INFO("  Connected: %s", _connection_ok ? "Yes" : "No");
	PX4_INFO("  Enabled: %s", _servo_enabled ? "Yes" : "No");
	PX4_INFO("  Safety Stop: %s", _safety_stop_active ? "ACTIVE" : "Normal");
	if (_safety_stop_active) {
		PX4_INFO("  Active Limit Function: %d", _active_limit_function);
	}

	// Show limit sensor configuration
	uint8_t left_limit = get_left_limit_id();
	uint8_t right_limit = get_right_limit_id();
	PX4_INFO("  Left Limit Sensor: %s", (left_limit == 255) ? "Disabled" : "Enabled");
	PX4_INFO("  Right Limit Sensor: %s", (right_limit == 255) ? "Disabled" : "Enabled");
	if (left_limit != 255) {
		PX4_INFO("    Left Sensor ID: %d, Active: %s", left_limit, _left_limit_active ? "YES" : "NO");
	}
	if (right_limit != 255) {
		PX4_INFO("    Right Sensor ID: %d, Active: %s", right_limit, _right_limit_active ? "YES" : "NO");
	}

	if (_connection_ok) {
		PX4_INFO("  Position: %.1f deg", (double)(_current_position * 180.0f / M_PI_F));
		PX4_INFO("  Speed: %.1f deg/s", (double)(_current_speed * 180.0f / M_PI_F));
		PX4_INFO("  Load: %.1f %%", (double)_current_load);
		PX4_INFO("  Temperature: %d °C", _current_temperature);
		PX4_INFO("  Voltage: %.1f V", (double)(_current_voltage * 0.1f));
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_comms_error_perf);
	perf_print_counter(_packet_count_perf);

	return 0;
}

int ST3215Servo::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	if (!strcmp(argv[0], "ping")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}
		_object.load()->_flag_ping = true;
		PX4_INFO("ping command queued");
		return 0;
	}

	if (!strcmp(argv[0], "wheel_mode")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}
		_object.load()->_flag_wheel_mode = true;
		PX4_INFO("wheel_mode command queued");
		return 0;
	}

	if (!strcmp(argv[0], "write_speed")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}

		if (argc < 2) {
			PX4_ERR("write_speed requires speed parameter");
			return 1;
		}

		float speed = atof(argv[1]);
		uint8_t acc = 0;
		if (argc >= 3) {
			acc = (uint8_t)atoi(argv[2]);
		}

		_object.load()->_cmd_speed = speed;
		_object.load()->_cmd_acceleration = acc;
		_object.load()->_flag_write_speed = true;
		PX4_INFO("write_speed command queued: %.2f rad/s, acc=%d", (double)speed, acc);
		return 0;
	}

	if (!strcmp(argv[0], "unlock_eprom")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}
		_object.load()->_flag_unlock_eprom = true;
		PX4_INFO("unlock_eprom command queued");
		return 0;
	}

	if (!strcmp(argv[0], "lock_eprom")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}
		_object.load()->_flag_lock_eprom = true;
		PX4_INFO("lock_eprom command queued");
		return 0;
	}

	if (!strcmp(argv[0], "read_moving")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}
		_object.load()->_flag_read_moving = true;
		PX4_INFO("read_moving command queued");
		return 0;
	}

	if (!strcmp(argv[0], "read_mode")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}
		_object.load()->_flag_read_mode = true;
		PX4_INFO("read_mode command queued");
		return 0;
	}

	if (!strcmp(argv[0], "safety_reset")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}
		// Reset all safety states (legacy and new HBridge pattern)
		_object.load()->_safety_stop_active = false;
		_object.load()->_active_limit_function = 255;
		_object.load()->_left_limit_active = false;
		_object.load()->_right_limit_active = false;
		PX4_INFO("Safety stop reset - servo movement allowed again");
		return 0;
	}

	if (!strcmp(argv[0], "set_abs_position")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}

		if (argc < 2) {
			PX4_ERR("set_abs_position requires position parameter (radians)");
			return 1;
		}

		float position = atof(argv[1]);
		float speed = 2.0f; // Default speed
		if (argc >= 3) {
			speed = atof(argv[2]);
		}

		_object.load()->_cmd_position = position;
		_object.load()->_cmd_position_speed = speed;
		_object.load()->_flag_set_abs_position = true;
		PX4_INFO("set_abs_position command queued: %.2f rad at %.2f rad/s", (double)position, (double)speed);
		return 0;
	}

	if (!strcmp(argv[0], "set_rel_position")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}

		if (argc < 2) {
			PX4_ERR("set_rel_position requires relative position parameter (radians)");
			return 1;
		}

		float rel_position = atof(argv[1]);
		float speed = 2.0f; // Default speed
		if (argc >= 3) {
			speed = atof(argv[2]);
		}

		_object.load()->_cmd_position = rel_position;
		_object.load()->_cmd_position_speed = speed;
		_object.load()->_flag_set_rel_position = true;
		PX4_INFO("set_rel_position command queued: %.2f rad at %.2f rad/s", (double)rel_position, (double)speed);
		return 0;
	}

	if (!strcmp(argv[0], "calibrate_middle_sts")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return 1;
		}
		_object.load()->_flag_calibrate_middle_sts = true;
		PX4_INFO("calibrate_middle_sts command queued (STS standard method)");
		return 0;
	}

	return print_usage("unknown command");
}

int ST3215Servo::task_spawn(int argc, char *argv[])
{
	const char *device_name = "/dev/ttyS3";
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			return PX4_ERROR;
		}
	}

	ST3215Servo *instance = new ST3215Servo(device_name);

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

int ST3215Servo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for ST3215 smart servo over serial interface

### Implementation
This driver communicates with ST3215 servo using its serial protocol.
The driver supports position control, speed setting, and status feedback.

### Examples
To start the driver on UART4 (default):
$ st3215_servo start -d /dev/ttyS3

To stop the driver:
$ st3215_servo stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("st3215_servo", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "UART device", false);

	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("ping");
	PRINT_MODULE_USAGE_COMMAND("wheel_mode");
	PRINT_MODULE_USAGE_COMMAND_DESCR("write_speed", "Set servo speed <speed_rad_s> [acceleration]");
	PRINT_MODULE_USAGE_COMMAND("unlock_eprom");
	PRINT_MODULE_USAGE_COMMAND("lock_eprom");
	PRINT_MODULE_USAGE_COMMAND("read_moving");
	PRINT_MODULE_USAGE_COMMAND("read_mode");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_abs_position", "Set absolute position <position_rad> [speed_rad_s]");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set_rel_position", "Set relative position <delta_rad> [speed_rad_s]");
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate_middle_sts", "Calibrate middle position using STS standard method (torque enable = 128)");
	PRINT_MODULE_USAGE_COMMAND("safety_reset");

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// SMS_STS style utility functions
bool ST3215Servo::wheel_mode(uint8_t servo_id)
{
	return write_register(servo_id, ST3215_REG_MODE, (const uint8_t[]){1}, 1);
}

bool ST3215Servo::write_speed(uint8_t servo_id, float speed_rad_s, uint8_t acc)
{
	// Convert speed to servo units with sign handling
	float speed_deg_s = speed_rad_s * 180.0f / M_PI_F;
	uint16_t speed_raw = (uint16_t)(fabsf(speed_deg_s) * 1023.0f / 360.0f);

	if (speed_deg_s < 0) {
		speed_raw |= (1 << 15); // Set sign bit for negative speed
	}

	// First write acceleration
	if (!write_register(servo_id, ST3215_REG_ACC, &acc, 1)) {
		return false;
	}

	// Then write speed
	uint8_t speed_data[2] = {
		(uint8_t)(speed_raw & 0xFF),
		(uint8_t)((speed_raw >> 8) & 0xFF)
	};

	return write_register(servo_id, ST3215_REG_GOAL_SPEED_L, speed_data, 2);
}

bool ST3215Servo::unlock_eprom(uint8_t servo_id)
{
	return write_register(servo_id, ST3215_REG_LOCK, (const uint8_t[]){0}, 1);
}

bool ST3215Servo::lock_eprom(uint8_t servo_id)
{
	return write_register(servo_id, ST3215_REG_LOCK, (const uint8_t[]){1}, 1);
}

int ST3215Servo::read_moving(uint8_t servo_id)
{
	uint8_t moving_data;
	if (read_register(servo_id, ST3215_REG_MOVING, &moving_data, 1)) {
		return moving_data;
	}
	return -1;
}

int ST3215Servo::read_mode(uint8_t servo_id)
{
	uint8_t mode_data;
	if (read_register(servo_id, ST3215_REG_MODE, &mode_data, 1)) {
		return mode_data;
	}
	return -1;
}

bool ST3215Servo::calibrate_middle_position_sts(uint8_t servo_id)
{
	// STS standard method: Write 128 to torque enable register for middle position calibration
	// This is the standard STS servo method mentioned: "STS舵机支持扭矩开关写128实现中位校准"
	uint8_t calibration_value = 128;

	PX4_INFO("Starting STS middle position calibration for servo %d", servo_id);

	// Write 128 to torque enable register to trigger middle position calibration
	if (!write_register(servo_id, ST3215_REG_TORQUE_ENABLE, &calibration_value, 1)) {
		PX4_ERR("Failed to write calibration command to torque enable register");
		return false;
	}

	// Wait a moment for the calibration to complete
	usleep(100000); // 100ms delay to allow calibration to finish

	PX4_INFO("STS middle position calibration completed for servo %d", servo_id);
	return true;
}

void ST3215Servo::process_limit_sensors()
{
	sensor_limit_switch_s limit_msg;

	// Check left limit sensor (HBridge pattern with bounds checking)
	uint8_t left_limit_id = get_left_limit_id();
	if (left_limit_id != 255 && left_limit_id < _limit_sensor_sub.size() &&
		_limit_sensor_sub[left_limit_id].updated() &&
		_limit_sensor_sub[left_limit_id].copy(&limit_msg)) {
		bool was_active = _left_limit_active;
		_left_limit_active = limit_msg.state;

		// Emergency stop if limit just became active
		if (limit_msg.state && !was_active) {
			emergency_stop();
			PX4_WARN("Servo emergency stop due to left limit sensor activation");
		}
		// Log recovery if limit was released
		else if (!limit_msg.state && was_active) {
			PX4_INFO("Left limit sensor released, servo movement allowed again");
		}
	}

	// Check right limit sensor (HBridge pattern with bounds checking)
	uint8_t right_limit_id = get_right_limit_id();
	if (right_limit_id != 255 && right_limit_id < _limit_sensor_sub.size() &&
		_limit_sensor_sub[right_limit_id].updated() &&
		_limit_sensor_sub[right_limit_id].copy(&limit_msg)) {
		bool was_active = _right_limit_active;
		_right_limit_active = limit_msg.state;

		// Emergency stop if limit just became active
		if (limit_msg.state && !was_active) {
			emergency_stop();
			PX4_WARN("Servo emergency stop due to right limit sensor activation");
		}
		// Log recovery if limit was released
		else if (!limit_msg.state && was_active) {
			PX4_INFO("Right limit sensor released, servo movement allowed again");
		}
	}

	// Update legacy safety state for backward compatibility
	_safety_stop_active = _left_limit_active || _right_limit_active;
	if (_safety_stop_active) {
		_active_limit_function = _left_limit_active ? get_left_limit_id() : get_right_limit_id();
	} else {
		_active_limit_function = 255;
	}
}

bool ST3215Servo::check_servo_safety(float goal_position)
{
	// Check if goal position is within configured limits
	float min_pos = _param_min_position.get();
	float max_pos = _param_max_position.get();

	if (goal_position < min_pos || goal_position > max_pos) {
		PX4_WARN("Goal position %.2f outside limits [%.2f, %.2f]",
			 (double)goal_position, (double)min_pos, (double)max_pos);
		return false;
	}

	// Check direction-specific limit sensors (HBridge pattern)
	// Determine rotation direction based on current vs goal position
	float position_delta = goal_position - _current_position;

	if (fabsf(position_delta) > 0.01f) {  // Only check if there's significant movement
		bool rotating_left = position_delta > 0;  // Positive delta = left rotation

		// Check the specific limit sensor for this direction
		if (rotating_left && get_left_limit_id() != 255 && get_left_limit_id() < _limit_sensor_sub.size() && _left_limit_active) {
			PX4_WARN("Rotation blocked by left limit sensor (ID %d)", get_left_limit_id());
			return false;
		}

		if (!rotating_left && get_right_limit_id() != 255 && get_right_limit_id() < _limit_sensor_sub.size() && _right_limit_active) {
			PX4_WARN("Rotation blocked by right limit sensor (ID %d)", get_right_limit_id());
			return false;
		}
	}

	return true;
}

void ST3215Servo::emergency_stop()
{
	uint8_t servo_id = _param_servo_id.get();

	if (servo_id > 0) {
		// Immediately disable torque to stop movement
		if (set_torque_enable(servo_id, false)) {
			_servo_enabled = false;
			PX4_WARN("Servo %d emergency stopped - torque disabled", servo_id);
		} else {
			PX4_ERR("Failed to emergency stop servo %d", servo_id);
		}
	}
}

extern "C" __EXPORT int st3215_servo_main(int argc, char *argv[])
{
	return ST3215Servo::main(argc, argv);
}
