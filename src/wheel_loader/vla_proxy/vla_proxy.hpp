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
 * @file vla_proxy.hpp
 * @author PX4 Development Team
 *
 * VLA (Vision-Language-Action) proxy driver header
 * Follows ST3215 servo driver pattern for robust UART communication
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_status.h>
// #include <uORB/topics/chassis_status.h>  // Message not yet defined
// #include <uORB/topics/vla_trajectory_setpoint.h>  // Message not yet defined
// #include <uORB/topics/mode_status.h>  // Message not yet defined

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>

using namespace time_literals;

class VLAProxy : public ModuleBase<VLAProxy>,
		 public ModuleParams,
		 public px4::ScheduledWorkItem
{
public:
	VLAProxy(const char *serial_port = "/dev/ttyS3");
	~VLAProxy() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool init();

private:
	void Run() override;

	// Serial communication (following ST3215 pattern)
	bool configure_port();
	bool send_packet(const uint8_t *data, size_t length);
	bool receive_packet(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms);
	uint8_t calculate_checksum(const uint8_t *data, size_t length);

	// VLA communication protocol
	bool send_robot_status();
	bool receive_trajectory_commands();
	void process_vla_message();

	// Wheel loader status structure (optimized for UART transmission)
	struct __attribute__((packed)) WheelloaderStatus {
		uint64_t timestamp;

		// Base platform state
		float position[3];        // x, y, z in meters
		float velocity[3];        // vx, vy, vz in m/s
		float quaternion[4];      // w, x, y, z
		float angular_velocity[3]; // roll_rate, pitch_rate, yaw_rate in rad/s
		uint8_t armed;           // 0=disarmed, 1=armed
		uint8_t operation_mode;  // Operation mode
		float battery_voltage;    // Battery voltage in volts

		// Joint states - Boom
		float boom_angle;         // boom angle in radians
		float boom_velocity;      // boom angular velocity (rad/s)
		float boom_load;          // current load on boom (normalized 0-1)
		float boom_motor_current; // boom motor current (A)
		uint8_t boom_state;       // boom state

		// Joint states - Bucket
		float bucket_angle;       // bucket angle relative to boom (radians)
		float bucket_ground_angle; // bucket angle relative to ground (radians)
		float bucket_actuator_length; // current actuator length in mm
		float bucket_velocity;    // actuator velocity (mm/s)
		float bucket_motor_current; // bucket motor current (A)
		uint8_t bucket_state;     // bucket state
		float estimated_load_kg;  // estimated load in bucket (kg)

		// Chassis state
		float chassis_linear_velocity;  // forward/backward velocity [m/s]
		float chassis_angular_velocity; // turning rate [rad/s]
		float chassis_steering_angle;   // articulation angle [rad]
		float wheel_speeds[4];    // individual wheel speeds [rad/s] FL,FR,RL,RR
		float chassis_traction_mu; // friction coefficient estimate
		uint8_t chassis_mode;     // current control mode
		uint8_t chassis_state;    // current chassis state
		uint8_t chassis_health;   // chassis health status
	};

	// VLA trajectory waypoint for 6DOF bucket control (optimized for UART)
	struct __attribute__((packed)) VLAWaypoint {
		uint64_t timestamp_us;
		float position[3];         // x, y, z position in meters
		float orientation[3];      // roll, pitch, yaw angles in radians
		float velocity[3];         // linear velocity (vx, vy, vz) in m/s
		float angular_velocity[3]; // angular velocity in rad/s
		float acceleration[3];     // linear acceleration in m/s²
		float angular_acceleration[3]; // angular acceleration in rad/s²
	};

	// VLA Protocol constants
	static constexpr uint8_t VLA_HEADER1 = 0xAA;
	static constexpr uint8_t VLA_HEADER2 = 0x55;
	static constexpr uint8_t VLA_MSG_STATUS = 0x01;    // Robot status message
	static constexpr uint8_t VLA_MSG_WAYPOINT = 0x02;  // Trajectory waypoint
	static constexpr uint8_t VLA_MSG_COMMAND = 0x03;   // VLA command
	static constexpr uint8_t VLA_MSG_ACK = 0x04;       // Acknowledgment

	// Helper functions
	void publish_trajectory_setpoint(const VLAWaypoint &waypoint);
	void collect_robot_status(WheelloaderStatus &status);
	bool validate_waypoint(const VLAWaypoint &waypoint);

	// Serial port
	int _uart{-1};
	char _port_name[32];

	// uORB topics
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	// uORB::Subscription _mode_status_sub{ORB_ID(mode_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
	uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};
	uORB::Subscription _chassis_status_sub{ORB_ID(chassis_status)};
	// uORB::Publication<vla_trajectory_setpoint_s> _vla_trajectory_pub{ORB_ID(vla_trajectory_setpoint)};

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _comms_error_perf;

	// Communication state
	bool _connection_ok{false};
	bool _vla_active{false};
	int _consecutive_errors{0};
	hrt_abstime _last_status_sent{0};
	hrt_abstime _last_waypoint_received{0};
	hrt_abstime _last_connection_check{0};

	// Waypoint buffer (circular buffer for better performance)
	static constexpr size_t WAYPOINT_BUFFER_SIZE = 10;
	VLAWaypoint _waypoint_buffer[WAYPOINT_BUFFER_SIZE];
	size_t _waypoint_buffer_head{0};
	size_t _waypoint_buffer_tail{0};
	size_t _waypoint_buffer_count{0};

	// Timing constants
	static constexpr unsigned SCHEDULE_INTERVAL = 20000;  // 50Hz update rate
	static constexpr uint32_t PACKET_TIMEOUT_MS = 10;
	static constexpr uint32_t STATUS_SEND_INTERVAL_MS = 50;  // Send status at 20Hz
	static constexpr uint32_t CONNECTION_CHECK_INTERVAL_MS = 1000;  // Check connection every 1s

	// Module parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VLA_ENABLED>) _param_vla_enabled,
		(ParamInt<px4::params::VLA_BAUDRATE>) _param_baudrate,
		(ParamFloat<px4::params::VLA_STATUS_RATE>) _param_status_rate,
		(ParamInt<px4::params::VLA_TIMEOUT_MS>) _param_timeout_ms,
		(ParamInt<px4::params::VLA_DEBUG>) _param_debug_level
	)
};
