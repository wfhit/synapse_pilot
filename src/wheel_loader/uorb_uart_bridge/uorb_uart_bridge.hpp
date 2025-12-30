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
 * @file uorb_uart_bridge.hpp
 * @author PX4 Development Team
 *
 * Refactored uORB UART Bridge module for X7+ main board
 * Uses improved distributed uORB protocol and dynamic topic registry
 * Provides robust communication with multiple NXT controller boards
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/distributed_uorb/distributed_uorb_protocol.hpp>
#include <lib/distributed_uorb/distributed_uorb_topic_registry.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>

// uORB topic includes
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/boom_control_setpoint.h>
#include <uORB/topics/tilt_control_setpoint.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/sensor_limit_switch.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/vehicle_status.h>

#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

using namespace time_literals;
using namespace distributed_uorb;

/**
 * Connection information for each remote node
 */
struct RemoteNodeConnection {
	NodeId node_id;
	int uart_fd;
	char device_path[32];
	bool is_connected;
	hrt_abstime last_heartbeat;
	hrt_abstime last_message;
	uint16_t tx_sequence;
	uint16_t last_rx_sequence;
	uint32_t tx_packets;
	uint32_t rx_packets;
	uint32_t tx_errors;
	uint32_t rx_errors;

	// Fixed-size circular buffer for transmission queue
	static constexpr size_t MAX_PACKET_SIZE = 256;
	static constexpr size_t TX_QUEUE_SIZE = 50;
	uint8_t tx_buffer[TX_QUEUE_SIZE][MAX_PACKET_SIZE];
	uint16_t tx_packet_sizes[TX_QUEUE_SIZE];
	size_t tx_queue_head;
	size_t tx_queue_tail;
	size_t tx_queue_count;
};

/**
 * Topic subscription/publication management
 */
struct TopicHandler {
	uint16_t topic_id;
	const orb_metadata *meta;
	uORB::Subscription *subscription;
	orb_advert_t publication;
	hrt_abstime last_published;
	uint32_t publish_count;
	bool is_outgoing; // true for X7+ → NXT, false for NXT → X7+
};

class UorbUartBridge : public ModuleBase<UorbUartBridge>,
		       public ModuleParams,
		       public px4::ScheduledWorkItem
{
public:
	UorbUartBridge();
	~UorbUartBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool init();

private:
	void Run() override;

	// Connection management
	bool configure_connections();
	bool open_connection(RemoteNodeConnection &conn);
	void close_connection(RemoteNodeConnection &conn);
	void check_connections();
	void send_heartbeat();

	// Message processing
	void process_outgoing_messages();
	void process_incoming_messages();
	void handle_received_frame(RemoteNodeConnection &conn, const ProtocolFrame *frame);

	// Topic management
	bool setup_topic_handlers();
	void cleanup_topic_handlers();
	TopicHandler *find_topic_handler(uint16_t topic_id);
	bool publish_to_local_topic(uint16_t topic_id, uint8_t instance, const void *data, size_t data_size);

	// Communication utilities
	bool send_frame(RemoteNodeConnection &conn, const uint8_t *frame_data, size_t frame_size);
	bool receive_frames(RemoteNodeConnection &conn);
	void handle_communication_error(RemoteNodeConnection &conn, const char *error_msg);

	// TX Queue management (fixed-size circular buffer)
	bool tx_queue_push(RemoteNodeConnection &conn, const uint8_t *data, size_t size);
	bool tx_queue_pop(RemoteNodeConnection &conn, uint8_t *data, size_t *size);
	bool tx_queue_empty(const RemoteNodeConnection &conn) const;
	bool tx_queue_full(const RemoteNodeConnection &conn) const;
	void tx_queue_init(RemoteNodeConnection &conn);

	// Statistics and diagnostics
	void update_statistics();
	void print_connection_status(const RemoteNodeConnection &conn) const;

	// Remote node connections (fixed-size arrays for embedded system)
	static constexpr size_t MAX_CONNECTIONS = 4; // Support up to 4 remote nodes
	RemoteNodeConnection _connections[MAX_CONNECTIONS];
	size_t _connection_count{0};

	// Topic handlers
	static constexpr size_t MAX_TOPIC_HANDLERS = 32; // Support up to 32 topics
	TopicHandler _topic_handlers[MAX_TOPIC_HANDLERS];
	size_t _topic_handler_count{0};

	// Protocol utilities
	FrameBuilder _frame_builder;
	FrameParser _frame_parser;

	// Communication state
	hrt_abstime _last_heartbeat_time{0};
	hrt_abstime _last_statistics_time{0};
	bool _all_connections_ready{false};

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _comms_error_perf;
	perf_counter_t _packet_tx_perf;
	perf_counter_t _packet_rx_perf;
	perf_counter_t _bytes_tx_perf;
	perf_counter_t _bytes_rx_perf;

	// Communication statistics
	struct {
		uint32_t tx_packets{0};
		uint32_t rx_packets{0};
		uint32_t tx_errors{0};
		uint32_t rx_errors{0};
	} _stats;

	// Timing constants
	static constexpr unsigned SCHEDULE_INTERVAL = 10_ms;  // 10ms for responsiveness
	static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
	static constexpr uint32_t CONNECTION_TIMEOUT_MS = 3000;
	static constexpr uint32_t STATISTICS_INTERVAL_MS = 10000;

	// Module parameters (≤16 chars per coding style)
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::UORB_BR_EN>) _param_enable,
		(ParamInt<px4::params::UORB_BR_BAUD>) _param_baudrate,
		(ParamBool<px4::params::UORB_BR_STATS>) _param_enable_stats,
		(ParamInt<px4::params::UORB_BR_F_PORT>) _param_front_port,
		(ParamInt<px4::params::UORB_BR_R_PORT>) _param_rear_port
	)
};

extern "C" __EXPORT int uorb_uart_bridge_main(int argc, char *argv[]);
