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
 * @file vla_proxy.cpp
 * @author PX4 Development Team
 *
 * VLA (Vision-Language-Action) proxy driver implementation
 * Follows ST3215 servo driver pattern for robust UART communication
 */

#include "vla_proxy.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <cstring>
#include <errno.h>
#include <sys/ioctl.h>

VLAProxy::VLAProxy(const char *serial_port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": loop")),
	_comms_error_perf(perf_alloc(PC_COUNT, MODULE_NAME": comm_err"))
{
	strncpy(_port_name, serial_port, sizeof(_port_name) - 1);
	_port_name[sizeof(_port_name) - 1] = '\0';
}

VLAProxy::~VLAProxy()
{
	if (_uart >= 0) {
		::close(_uart);
		_uart = -1;
	}

	perf_free(_loop_perf);
	perf_free(_comms_error_perf);
}

bool VLAProxy::init()
{
	// Start work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	PX4_INFO("VLA Proxy driver started on %s", _port_name);
	return true;
}

bool VLAProxy::configure_port()
{
	// Close existing connection if open
	if (_uart >= 0) {
		::close(_uart);
		_uart = -1;
	}

	// Open serial port
	PX4_INFO("Opening VLA serial port %s...", _port_name);
	_uart = ::open(_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart < 0) {
		PX4_ERR("Failed to open %s: %s", _port_name, strerror(errno));
		return false;
	}

	PX4_INFO("VLA serial port opened successfully (fd=%d)", _uart);

	// Configure port settings
	struct termios uart_config;

	if (tcgetattr(_uart, &uart_config) != 0) {
		PX4_ERR("Error getting VLA serial port attributes: %s", strerror(errno));
		::close(_uart);
		_uart = -1;
		return false;
	}

	// Get baudrate parameter
	int32_t baudrate = _param_baudrate.get();

	if (baudrate <= 0) {
		baudrate = 115200;  // Default
	}

	PX4_INFO("Configuring VLA baudrate: %ld", (long)baudrate);

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
	uart_config.c_cc[VTIME] = 1;     // Wait for up to 0.1s
	uart_config.c_cc[VMIN] = 0;      // No minimum number of characters

	if (tcsetattr(_uart, TCSANOW, &uart_config) != 0) {
		PX4_ERR("Error setting VLA serial port attributes: %s", strerror(errno));
		::close(_uart);
		_uart = -1;
		return false;
	}

	// Clear any existing data
	tcflush(_uart, TCIOFLUSH);

	return true;
}

void VLAProxy::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	perf_begin(_loop_perf);

	// Update parameters
	updateParams();

	// Check if VLA is enabled
	if (!_param_vla_enabled.get()) {
		perf_end(_loop_perf);
		return;
	}

	// Configure serial port if not already configured
	if (_uart < 0) {
		if (!configure_port()) {
			if (_param_debug_level.get() > 0) {
				PX4_WARN("Failed to configure VLA serial port, will retry later");
			}
			perf_end(_loop_perf);
			return;
		}
	}

	// Send robot status at configured rate
	hrt_abstime now = hrt_absolute_time();
	uint32_t status_interval_us = static_cast<uint32_t>(1000000.f / math::max(_param_status_rate.get(), 1.0f));

	if (now - _last_status_sent > status_interval_us) {
		if (send_robot_status()) {
			_last_status_sent = now;
		}
	}

	// Process incoming trajectory commands
	if (receive_trajectory_commands()) {
		_last_waypoint_received = now;
	}

	// Check connection status
	if (now - _last_connection_check > CONNECTION_CHECK_INTERVAL_MS * 1000) {
		_last_connection_check = now;

		// Check if we haven't received waypoints for too long
		if (_last_waypoint_received > 0 &&
		    (now - _last_waypoint_received) > static_cast<hrt_abstime>(_param_timeout_ms.get() * 1000)) {
			if (_vla_active) {
				PX4_WARN("VLA timeout - no waypoints received for %llu ms",
				         (now - _last_waypoint_received) / 1000);
				_vla_active = false;
			}
		}

		// Reset error counter if connection is good
		if (_connection_ok && _consecutive_errors > 0) {
			_consecutive_errors = 0;
		}
	}

	// Publish all buffered waypoints when we have a full batch or timeout
	if (_waypoint_buffer_count > 0) {
		// Check if first waypoint time has arrived or buffer is full
		if (_waypoint_buffer[0].timestamp_us <= now || _waypoint_buffer_count >= WAYPOINT_BUFFER_SIZE) {
			publish_trajectory_batch();
		}
	}

	perf_end(_loop_perf);
}

uint8_t VLAProxy::calculate_checksum(const uint8_t *data, size_t length)
{
	uint8_t checksum = 0;
	for (size_t i = 0; i < length; i++) {
		checksum ^= data[i];
	}
	return checksum;
}

bool VLAProxy::send_packet(const uint8_t *data, size_t length)
{
	if (_uart < 0 || !data || length == 0) {
		return false;
	}

	// Clear input buffer before sending to avoid stale data
	tcflush(_uart, TCIFLUSH);

	// Send the packet
	ssize_t bytes_written = ::write(_uart, data, length);
	if (bytes_written != (ssize_t)length) {
		if (_param_debug_level.get() > 0) {
			PX4_ERR("VLA write failed: expected %zu, wrote %zd, error: %s",
			        length, bytes_written, strerror(errno));
		}
		perf_count(_comms_error_perf);
		_consecutive_errors++;
		return false;
	}

	// Wait for transmission to complete
	tcdrain(_uart);
	return true;
}

bool VLAProxy::receive_packet(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms)
{
	if (_uart < 0 || !buffer || buffer_size == 0) {
		return false;
	}

	hrt_abstime start_time = hrt_absolute_time();
	size_t bytes_received = 0;

	// Read header first (4 bytes: HEADER1, HEADER2, TYPE, LENGTH)
	const size_t header_size = 4;

	while (bytes_received < header_size && bytes_received < buffer_size) {
		// Check for timeout
		if (hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			if (_param_debug_level.get() > 1) {
				PX4_WARN("VLA header timeout after %lu ms, got %zu bytes", (unsigned long)timeout_ms, bytes_received);
			}
			return false;
		}

		ssize_t bytes_read = ::read(_uart, buffer + bytes_received, buffer_size - bytes_received);
		if (bytes_read > 0) {
			bytes_received += bytes_read;
		} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			if (_param_debug_level.get() > 0) {
				PX4_ERR("VLA read error: %s", strerror(errno));
			}
			perf_count(_comms_error_perf);
			return false;
		}

		// Small delay to prevent CPU spinning
		usleep(500);
	}

	// Check if we have valid header
	if (bytes_received < header_size) {
		if (_param_debug_level.get() > 1) {
			PX4_WARN("VLA insufficient header bytes: got %zu, need %zu", bytes_received, header_size);
		}
		return false;
	}

	// Validate header
	if (buffer[0] != VLA_HEADER1 || buffer[1] != VLA_HEADER2) {
		if (_param_debug_level.get() > 0) {
			PX4_ERR("VLA invalid header: 0x%02X 0x%02X (expected 0xAA 0x55)", buffer[0], buffer[1]);
		}
		perf_count(_comms_error_perf);
		return false;
	}

	// Parse packet length from header
	uint8_t packet_length = buffer[3];

	// Calculate total expected packet size
	// Format: HEADER1(1) + HEADER2(1) + TYPE(1) + LENGTH(1) + DATA(LENGTH) + CHECKSUM(1)
	size_t total_expected = 4 + packet_length + 1;

	if (total_expected > buffer_size) {
		if (_param_debug_level.get() > 0) {
			PX4_ERR("VLA packet too large: expected %zu bytes, buffer only %zu", total_expected, buffer_size);
		}
		return false;
	}

	// Read remaining bytes
	while (bytes_received < total_expected && bytes_received < buffer_size) {
		if (hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			if (_param_debug_level.get() > 1) {
				PX4_WARN("VLA data timeout after %lu ms, got %zu/%zu bytes",
				         (unsigned long)timeout_ms, bytes_received, total_expected);
			}
			return false;
		}

		ssize_t bytes_read = ::read(_uart, buffer + bytes_received, buffer_size - bytes_received);
		if (bytes_read > 0) {
			bytes_received += bytes_read;
		} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			if (_param_debug_level.get() > 0) {
				PX4_ERR("VLA read error: %s", strerror(errno));
			}
			perf_count(_comms_error_perf);
			return false;
		}

		// Small delay to prevent CPU spinning
		usleep(500);
	}

	// Verify checksum
	uint8_t calculated_checksum = calculate_checksum(&buffer[2], bytes_received - 3);
	uint8_t received_checksum = buffer[bytes_received - 1];

	if (calculated_checksum != received_checksum) {
		if (_param_debug_level.get() > 0) {
			PX4_ERR("VLA checksum mismatch: calc=0x%02X, recv=0x%02X",
			        calculated_checksum, received_checksum);
		}
		perf_count(_comms_error_perf);
		_consecutive_errors++;
		return false;
	}

	return true;
}

void VLAProxy::collect_robot_status(WheelloaderStatus &status)
{
	status.timestamp = hrt_absolute_time();

	// Get arming state from actuator_armed
	actuator_armed_s actuator_armed;
	if (_actuator_armed_sub.copy(&actuator_armed)) {
		status.armed = actuator_armed.armed ? 1 : 0;
	}

	// Get operation mode from operation_mode_status (authoritative source)
	operation_mode_status_s mode_status;
	if (_operation_mode_status_sub.copy(&mode_status)) {
		status.operation_mode = mode_status.current_mode;
	}

	// Get position and velocity
	vehicle_local_position_s local_pos;
	if (_vehicle_local_position_sub.copy(&local_pos)) {
		status.position[0] = local_pos.x;
		status.position[1] = local_pos.y;
		status.position[2] = local_pos.z;
		status.velocity[0] = local_pos.vx;
		status.velocity[1] = local_pos.vy;
		status.velocity[2] = local_pos.vz;
	}

	// Get attitude (quaternion)
	vehicle_attitude_s attitude;
	if (_vehicle_attitude_sub.copy(&attitude)) {
		status.quaternion[0] = attitude.q[0];
		status.quaternion[1] = attitude.q[1];
		status.quaternion[2] = attitude.q[2];
		status.quaternion[3] = attitude.q[3];
	}

	// Get angular velocity
	vehicle_angular_velocity_s angular_vel;
	if (_vehicle_angular_velocity_sub.copy(&angular_vel)) {
		status.angular_velocity[0] = angular_vel.xyz[0];
		status.angular_velocity[1] = angular_vel.xyz[1];
		status.angular_velocity[2] = angular_vel.xyz[2];
	}

	// Get boom status
	boom_status_s boom_status;
	if (_boom_status_sub.copy(&boom_status)) {
		status.boom_angle = boom_status.angle;
		status.boom_velocity = boom_status.velocity;
		status.boom_load = boom_status.load;
		status.boom_motor_current = boom_status.motor_current;
		status.boom_state = boom_status.state;
	} else {
		// Default values if boom status unavailable
		status.boom_angle = 0.0f;
		status.boom_velocity = 0.0f;
		status.boom_load = 0.0f;
		status.boom_motor_current = 0.0f;
		status.boom_state = 0;
	}

	// Get bucket status
	bucket_status_s bucket_status;
	if (_bucket_status_sub.copy(&bucket_status)) {
		status.bucket_angle = bucket_status.bucket_angle;
		status.bucket_ground_angle = bucket_status.ground_angle;
		status.bucket_actuator_length = bucket_status.actuator_length;
		status.bucket_velocity = bucket_status.velocity;
		status.bucket_motor_current = bucket_status.motor_current;
		status.bucket_state = bucket_status.state;
		status.estimated_load_kg = bucket_status.estimated_load_kg;
	} else {
		// Default values if bucket status unavailable
		status.bucket_angle = 0.0f;
		status.bucket_ground_angle = 0.0f;
		status.bucket_actuator_length = 0.0f;
		status.bucket_velocity = 0.0f;
		status.bucket_motor_current = 0.0f;
		status.bucket_state = 0;
		status.estimated_load_kg = 0.0f;
	}

	// Get drivetrain status (front and rear)
	for (int i = 0; i < 2; i++) {
		drivetrain_status_s drivetrain;
		if (_drivetrain_status_subs[i].copy(&drivetrain)) {
			status.drivetrain_speeds[i] = drivetrain.current_speed_rpm;
		} else {
			status.drivetrain_speeds[i] = 0.0f;
		}
	}

	// Get steering status
	steering_status_s steering;
	if (_steering_status_sub.copy(&steering)) {
		status.chassis_steering_angle = steering.steering_angle_deg;
		status.chassis_state = steering.control_mode;
		status.chassis_health = steering.is_healthy ? 1 : 0;
	} else {
		status.chassis_steering_angle = 0.0f;
		status.chassis_state = 0;
		status.chassis_health = 0;
	}

	// Get traction status
	traction_status_s traction;
	if (_traction_status_sub.copy(&traction)) {
		status.chassis_traction_mu = traction.overall_traction_quality;
		status.chassis_mode = traction.intervention_level;
	} else {
		status.chassis_traction_mu = 0.0f;
		status.chassis_mode = 0;
	}

	// Calculate linear velocity (already have local_pos from earlier)
	status.chassis_linear_velocity = sqrtf(status.velocity[0] * status.velocity[0] + status.velocity[1] * status.velocity[1]);

	// Chassis angular velocity is the yaw rate (already fetched earlier in angular_velocity[2])
	status.chassis_angular_velocity = status.angular_velocity[2];

	// Get battery voltage from battery_status topic
	battery_status_s battery;
	if (_battery_status_sub.copy(&battery)) {
		status.battery_voltage = battery.voltage_v;
	} else {
		status.battery_voltage = 0.0f;
	}
}

bool VLAProxy::send_robot_status()
{
	WheelloaderStatus status;
	collect_robot_status(status);

	// Create packet: HEADER1 + HEADER2 + TYPE + LENGTH + DATA + CHECKSUM
	uint8_t packet[sizeof(WheelloaderStatus) + 6];  // 4 header bytes + data + 1 checksum
	size_t packet_size = 0;

	packet[packet_size++] = VLA_HEADER1;
	packet[packet_size++] = VLA_HEADER2;
	packet[packet_size++] = VLA_MSG_STATUS;
	packet[packet_size++] = sizeof(WheelloaderStatus);

	// Copy status data
	memcpy(&packet[packet_size], &status, sizeof(WheelloaderStatus));
	packet_size += sizeof(WheelloaderStatus);

	// Calculate and add checksum
	uint8_t checksum = calculate_checksum(&packet[2], packet_size - 2);
	packet[packet_size++] = checksum;

	return send_packet(packet, packet_size);
}

bool VLAProxy::receive_trajectory_commands()
{
	// Buffer sized for 32 waypoints batch: 32 * ~80 bytes + overhead = ~3KB
	uint8_t buffer[3072];

	if (!receive_packet(buffer, sizeof(buffer), PACKET_TIMEOUT_MS)) {
		return false;
	}

	// Parse message type and length
	uint8_t msg_type = buffer[2];
	uint8_t msg_length = buffer[3];

	// Only handle waypoint messages for now
	if (msg_type != VLA_MSG_WAYPOINT) {
		if (_param_debug_level.get() > 0) {
			PX4_WARN("Unsupported VLA message type: 0x%02X", msg_type);
		}
		return false;
	}

	// Calculate number of waypoints in this batch
	size_t num_waypoints = msg_length / sizeof(VLAWaypoint);
	
	if (msg_length % sizeof(VLAWaypoint) != 0) {
		if (_param_debug_level.get() > 0) {
			PX4_ERR("VLA batch size invalid: %d bytes is not multiple of waypoint size %zu",
			        msg_length, sizeof(VLAWaypoint));
		}
		return false;
	}

	if (num_waypoints > WAYPOINT_BUFFER_SIZE) {
		if (_param_debug_level.get() > 0) {
			PX4_WARN("VLA batch too large: %zu waypoints, max %zu", num_waypoints, WAYPOINT_BUFFER_SIZE);
		}
		num_waypoints = WAYPOINT_BUFFER_SIZE;  // Truncate to buffer size
	}

	// Clear existing buffer and load new batch
	_waypoint_buffer_count = 0;

	// Parse and validate all waypoints in the batch
	for (size_t i = 0; i < num_waypoints; i++) {
		VLAWaypoint waypoint;
		memcpy(&waypoint, &buffer[4 + i * sizeof(VLAWaypoint)], sizeof(VLAWaypoint));

		if (!validate_waypoint(waypoint)) {
			if (_param_debug_level.get() > 0) {
				PX4_WARN("VLA waypoint %zu validation failed, skipping", i);
			}
			continue;
		}

		_waypoint_buffer[_waypoint_buffer_count++] = waypoint;
	}

	if (_waypoint_buffer_count > 0) {
		_vla_active = true;

		if (_param_debug_level.get() > 1) {
			PX4_INFO("VLA batch received: %zu waypoints buffered", _waypoint_buffer_count);
		}
	} else {
		if (_param_debug_level.get() > 0) {
			PX4_WARN("VLA batch contained no valid waypoints");
		}
	}

	return true;
}

bool VLAProxy::validate_waypoint(const VLAWaypoint &waypoint)
{
	// Basic sanity checks
	if (waypoint.timestamp_us == 0) {
		return false;
	}

	// Check for reasonable position values (within ±1000 meters)
	for (int i = 0; i < 3; i++) {
		if (!std::isfinite(waypoint.position[i]) || fabsf(waypoint.position[i]) > 1000.0f) {
			return false;
		}
		if (!std::isfinite(waypoint.velocity[i]) || fabsf(waypoint.velocity[i]) > 100.0f) {
			return false;
		}
		if (!std::isfinite(waypoint.acceleration[i]) || fabsf(waypoint.acceleration[i]) > 100.0f) {
			return false;
		}
	}

	// Check orientation values (reasonable Euler angles)
	for (int i = 0; i < 3; i++) {
		if (!std::isfinite(waypoint.orientation[i]) || fabsf(waypoint.orientation[i]) > 2.0f * static_cast<float>(M_PI)) {
			return false;
		}
		if (!std::isfinite(waypoint.angular_velocity[i]) || fabsf(waypoint.angular_velocity[i]) > 10.0f) {
			return false;
		}
		if (!std::isfinite(waypoint.angular_acceleration[i]) || fabsf(waypoint.angular_acceleration[i]) > 100.0f) {
			return false;
		}
	}

	return true;
}

void VLAProxy::publish_trajectory_batch()
{
	vla_trajectory_s traj{};

	traj.timestamp = hrt_absolute_time();
	traj.trajectory_type = vla_trajectory_s::TRAJ_TYPE_BUCKET_6DOF;
	traj.frame_id = vla_trajectory_s::FRAME_LOCAL;
	traj.num_points = math::min(_waypoint_buffer_count, static_cast<size_t>(32));  // VLA trajectory supports max 32 points
	traj.current_point_index = 0;

	// Fill trajectory with all buffered waypoints
	for (size_t i = 0; i < traj.num_points; i++) {
		const VLAWaypoint &wp = _waypoint_buffer[i];

		traj.point_timestamps[i] = wp.timestamp_us;
		traj.bucket_x[i] = wp.position[0];
		traj.bucket_y[i] = wp.position[1];
		traj.bucket_z[i] = wp.position[2];

		// Convert Euler angles to quaternion
		float roll = wp.orientation[0];
		float pitch = wp.orientation[1];
		float yaw = wp.orientation[2];
		float cr = cosf(roll * 0.5f);
		float sr = sinf(roll * 0.5f);
		float cp = cosf(pitch * 0.5f);
		float sp = sinf(pitch * 0.5f);
		float cy = cosf(yaw * 0.5f);
		float sy = sinf(yaw * 0.5f);

		traj.bucket_qw[i] = cr * cp * cy + sr * sp * sy;
		traj.bucket_qx[i] = sr * cp * cy - cr * sp * sy;
		traj.bucket_qy[i] = cr * sp * cy + sr * cp * sy;
		traj.bucket_qz[i] = cr * cp * sy - sr * sp * cy;
	}

	// Control flags
	traj.hold_last_point = false;
	traj.smooth_transition = true;

	_vla_trajectory_pub.publish(traj);

	if (_param_debug_level.get() > 1) {
		PX4_INFO("Published VLA trajectory batch with %u points", traj.num_points);
	}

	// Clear buffer after publishing
	_waypoint_buffer_count = 0;
	_waypoint_buffer_head = 0;
}

int VLAProxy::print_status()
{
	PX4_INFO("VLA Proxy Driver Status:");
	PX4_INFO("  Port: %s", _port_name);
	PX4_INFO("  UART FD: %d", _uart);
	PX4_INFO("  Connection OK: %s", _connection_ok ? "YES" : "NO");
	PX4_INFO("  VLA Active: %s", _vla_active ? "YES" : "NO");
	PX4_INFO("  Consecutive Errors: %d", _consecutive_errors);
	PX4_INFO("  Waypoint Buffer Count: %zu", _waypoint_buffer_count);
	PX4_INFO("  Last Status Sent: %llu ms ago",
	         (_last_status_sent > 0) ? (hrt_absolute_time() - _last_status_sent) / 1000 : 0);
	PX4_INFO("  Last Waypoint Received: %llu ms ago",
	         (_last_waypoint_received > 0) ? (hrt_absolute_time() - _last_waypoint_received) / 1000 : 0);

	// Print only essential performance counters
	perf_print_counter(_loop_perf);
	perf_print_counter(_comms_error_perf);

	return 0;
}

int VLAProxy::task_spawn(int argc, char *argv[])
{
	const char *serial_port = "/dev/ttyS3";  // Default port

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			serial_port = myoptarg;
			break;
		case '?':
			print_usage("Unknown option");
			return -1;
		default:
			break;
		}
	}

	VLAProxy *instance = new VLAProxy(serial_port);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("VLA Proxy alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int VLAProxy::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (argc > 0 && !strcmp(argv[0], "status")) {
		return get_instance()->print_status();
	}

	print_usage("Unknown command");
	return 1;
}

int VLAProxy::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
VLA (Vision-Language-Action) proxy driver.

This driver acts as a bridge between PX4 and a VLA model, handling:
- Sending robot status to VLA via UART using robust packet protocol
- Receiving 6DOF trajectory commands from VLA
- Publishing trajectory setpoints for bucket control

### Implementation
The driver uses a reliable packet-based protocol with headers, checksums, and timeouts.
Follows the ST3215 servo driver pattern for robust UART communication.

### Examples
Start the driver on a specific serial port:
$ vla_proxy start -d /dev/ttyS3

Show driver status:
$ vla_proxy status

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("vla_proxy", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<device>", "Serial port", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Show driver status");

	return 0;
}

extern "C" __EXPORT int vla_proxy_main(int argc, char *argv[]);

int vla_proxy_main(int argc, char *argv[])
{
	return VLAProxy::main(argc, argv);
}
