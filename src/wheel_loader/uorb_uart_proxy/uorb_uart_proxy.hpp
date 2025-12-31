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
 * @file uorb_uart_proxy.hpp
 * @author PX4 Development Team
 *
 * Refactored uORB UART Proxy module for NXT controller boards
 * Uses improved distributed uORB protocol and dynamic topic registry
 * Provides transparent uORB messaging with X7+ main board via UART
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
 * Topic subscription/publication management for NXT proxy
 */
struct ProxyTopicHandler {
	uint16_t topic_id;
	const orb_metadata *meta;
	uORB::Subscription *subscription;
	orb_advert_t publication;
	uint8_t instance;
	hrt_abstime last_updated;
	bool is_outgoing; // true for NXT → X7+, false for X7+ → NXT
};

class UorbUartProxy : public ModuleBase<UorbUartProxy>,
		      public ModuleParams,
		      public px4::ScheduledWorkItem
{
public:
	UorbUartProxy();
	~UorbUartProxy() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool init();

private:
	void Run() override;

	// Connection management
	bool configure_uart();
	void check_connection();
	void send_heartbeat();

	// Message processing
	void process_outgoing_messages();
	void process_incoming_messages();
	void handle_received_frame(const ProtocolFrame *frame);

	// Topic management
	bool setup_topic_handlers();
	void cleanup_topic_handlers();
	ProxyTopicHandler *find_topic_handler(uint16_t topic_id, uint8_t instance);
	bool publish_to_local_topic(uint16_t topic_id, uint8_t instance, const void *data, size_t data_size);

	// Communication utilities
	bool send_frame(const uint8_t *frame_data, size_t frame_size);
	bool receive_frames();
	void handle_communication_error(const char *error_msg);

	// Board type detection and filtering
	NodeId detect_board_type();
	bool is_topic_relevant_for_node(uint16_t topic_id, NodeId node_id);

	// UART connection
	int _uart_fd{-1};
	char _device_path[32];
	bool _is_connected{false};

	// Node identification
	NodeId _node_id{NodeId::RESERVED};

	// Topic handlers (fixed-size array for embedded system)
	static constexpr size_t MAX_TOPIC_HANDLERS = 24; // Support up to 24 topics per NXT node
	ProxyTopicHandler _topic_handlers[MAX_TOPIC_HANDLERS];
	size_t _topic_handler_count{0};

	// Protocol utilities
	FrameBuilder _frame_builder;
	FrameParser _frame_parser;

	// Communication state
	hrt_abstime _last_heartbeat_time{0};
	hrt_abstime _last_message_time{0};
	uint16_t _tx_sequence{0};
	uint16_t _last_rx_sequence{0};

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
	static constexpr unsigned SCHEDULE_INTERVAL = 10000;  // 10ms for responsiveness
	static constexpr uint32_t HEARTBEAT_INTERVAL_MS = 1000;
	static constexpr uint32_t CONNECTION_TIMEOUT_MS = 3000;
	static constexpr size_t MAX_TX_QUEUE_SIZE = 30;

	// Module parameters (≤16 chars per coding style)
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::UORB_PX_EN>) _param_enable,
		(ParamInt<px4::params::UORB_PX_BAUD>) _param_baudrate,
		(ParamBool<px4::params::UORB_PX_STATS>) _param_enable_stats,
		(ParamInt<px4::params::UORB_PX_NODE_ID>) _param_node_id,
		(ParamInt<px4::params::UORB_PX_UART>) _param_uart_device
	)
};

extern "C" __EXPORT int uorb_uart_proxy_main(int argc, char *argv[]);
