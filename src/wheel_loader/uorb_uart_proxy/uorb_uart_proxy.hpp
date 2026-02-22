/****************************************************************************
 *
 *   Copyright (c) 2024-2025 PX4 Development Team. All rights reserved.
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
 * uORB UART Proxy - Forwards uORB topics over UART to/from main board
 *
 * Runs on NXT-Dual controller boards (front/rear) and communicates with
 * uorb_uart_bridge on the X7+ main board.
 *
 * Key features:
 * - Persistent subscriptions (no subscription-every-cycle bug)
 * - RX frame accumulation (handles partial frames across reads)
 * - Static topic tables (no global registry overhead)
 * - Lazy incoming publications (created on first frame received)
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <lib/distributed_uorb/distributed_uorb_protocol.hpp>
#include <lib/distributed_uorb/distributed_uorb_topic_registry.hpp>

// Outgoing topics (NXT -> X7+)
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/sensor_limit_switch.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/hbridge_status.h>

// Incoming topics (X7+ -> NXT)
#include <uORB/topics/boom_control_setpoint.h>
#include <uORB/topics/tilt_control_setpoint.h>

using namespace time_literals;
using namespace distributed_uorb;

class UorbUartProxy : public ModuleBase<UorbUartProxy>, public ModuleParams
{
public:
	UorbUartProxy();
	~UorbUartProxy() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();
	void run();

private:
	static int run_trampoline(int argc, char *argv[]);

	// UART management
	bool open_uart();
	void close_uart();

	// Outgoing (NXT -> X7+): persistent subscriptions
	struct OutgoingSub {
		uORB::Subscription sub;
		uint16_t topic_id;
		const char *name;
	};
	static constexpr size_t MAX_OUT_SUBS = 16;
	OutgoingSub _out_subs[MAX_OUT_SUBS]{};
	size_t _out_sub_count{0};

	void setup_outgoing_subscriptions();
	void send_outgoing();

	// Incoming (X7+ -> NXT): lazy publications
	struct IncomingPub {
		orb_advert_t handle{nullptr};
		const orb_metadata *meta{nullptr};
		uint16_t topic_id{0};
		uint8_t instance{0};
	};
	static constexpr size_t MAX_IN_PUBS = 8;
	IncomingPub _in_pubs[MAX_IN_PUBS]{};
	size_t _in_pub_count{0};

	void receive_incoming();
	void parse_rx_buffer();
	void dispatch_frame(const ProtocolFrame *frame, const uint8_t *payload);
	void publish_incoming(uint16_t topic_id, uint8_t instance, const uint8_t *data, size_t size);
	const orb_metadata *get_topic_meta(uint16_t topic_id);

	// Time synchronization (via heartbeat)
	void process_heartbeat_time_sync(const HeartbeatPayload &hb);
	int64_t get_time_offset() const { return _time_offset_us; }
	bool is_time_synced() const { return _time_synced; }

	int64_t _time_offset_us{0};     // Clock offset: master_time = local_time + offset
	bool _time_synced{false};
	uint8_t _time_sync_quality{0};
	hrt_abstime _last_time_sync_rx{0};

	// Master heartbeat echo state (for time sync round-trip)
	int64_t _master_ts_t1{0};       // Last received master heartbeat ts_t1
	int64_t _master_hb_rx_time{0};  // Local time when master heartbeat was received
	uint8_t _master_ts_seq{0};      // Last received master ts_seq

	// Heartbeat
	void send_heartbeat();

	// Protocol
	FrameBuilder _frame_builder;
	FrameParser _frame_parser;
	uint16_t _tx_seq{0};

	// Buffers (class members, not stack)
	static constexpr size_t TX_BUF_SIZE = 600;
	static constexpr size_t RX_BUF_SIZE = 1024;
	static constexpr size_t MSG_BUF_SIZE = 256;
	uint8_t _tx_buf[TX_BUF_SIZE];
	uint8_t _rx_buf[RX_BUF_SIZE];
	size_t _rx_len{0};
	uint8_t _msg_buf[MSG_BUF_SIZE];

	// UART state
	int _uart_fd{-1};
	char _uart_device[32]{};

	// Timing
	hrt_abstime _last_heartbeat_time{0};
	hrt_abstime _last_rx_time{0};
	static constexpr hrt_abstime HEARTBEAT_INTERVAL = 1_s;

	// Statistics
	uint32_t _tx_frames{0};
	uint32_t _rx_frames{0};
	uint32_t _tx_bytes{0};
	uint32_t _rx_bytes{0};
	uint32_t _tx_errors{0};
	uint32_t _rx_errors{0};
	uint32_t _parse_errors{0};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamBool<px4::params::UORB_PX_EN>) _param_enable,
		(ParamInt<px4::params::UORB_PX_UART>) _param_uart_port,
		(ParamInt<px4::params::UORB_PX_BAUD>) _param_baud,
		(ParamInt<px4::params::UORB_PX_NODE_ID>) _param_node_id,
		(ParamBool<px4::params::UORB_PX_STATS>) _param_stats
	)
};
