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

#include "uorb_uart_proxy.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>
#include <drivers/drv_hrt.h>

UorbUartProxy::UorbUartProxy() :
	ModuleParams(nullptr)
{
}

UorbUartProxy::~UorbUartProxy()
{
	close_uart();
}

bool UorbUartProxy::init()
{
	if (!_param_enable.get()) {
		PX4_INFO("Proxy disabled (UORB_PX_EN=0)");
		return false;
	}

	// Open UART
	if (!open_uart()) {
		PX4_ERR("Failed to open UART");
		return false;
	}

	// Setup outgoing subscriptions
	setup_outgoing_subscriptions();

	PX4_INFO("Initialized on %s at %ld baud, node_id=%ld",
		 _uart_device, (long)_param_baud.get(), (long)_param_node_id.get());

	return true;
}

bool UorbUartProxy::open_uart()
{
	// Build device path from parameter
	int32_t uart_port = _param_uart_port.get();
	snprintf(_uart_device, sizeof(_uart_device), "/dev/ttyS%ld", (long)uart_port);

	// Open UART
	_uart_fd = ::open(_uart_device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		PX4_ERR("Failed to open %s: %d", _uart_device, errno);
		return false;
	}

	// Configure UART
	struct termios uart_config;

	if (tcgetattr(_uart_fd, &uart_config) != 0) {
		PX4_ERR("tcgetattr failed: %d", errno);
		::close(_uart_fd);
		_uart_fd = -1;
		return false;
	}

	// Raw mode
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL | INLCR | PARMRK | INPCK | ISTRIP | IXON);
	uart_config.c_oflag &= ~(OCRNL | ONLCR | ONLRET | ONOCR | OFILL | OPOST);
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
	uart_config.c_cflag &= ~(CSIZE | PARENB | CSTOPB);
	uart_config.c_cflag |= CS8;

	// Set baud rate
	int32_t baud = _param_baud.get();
	speed_t speed;

	switch (baud) {
	case 9600:   speed = B9600;   break;
	case 19200:  speed = B19200;  break;
	case 38400:  speed = B38400;  break;
	case 57600:  speed = B57600;  break;
	case 115200: speed = B115200; break;
	case 230400: speed = B230400; break;
	case 460800: speed = B460800; break;
	case 921600: speed = B921600; break;

	default:
		PX4_ERR("Unsupported baud rate: %ld", (long)baud);
		::close(_uart_fd);
		_uart_fd = -1;
		return false;
	}

	cfsetispeed(&uart_config, speed);
	cfsetospeed(&uart_config, speed);

	// Non-blocking reads
	uart_config.c_cc[VMIN] = 0;
	uart_config.c_cc[VTIME] = 0;

	if (tcsetattr(_uart_fd, TCSANOW, &uart_config) != 0) {
		PX4_ERR("tcsetattr failed: %d", errno);
		::close(_uart_fd);
		_uart_fd = -1;
		return false;
	}

	return true;
}

void UorbUartProxy::close_uart()
{
	if (_uart_fd >= 0) {
		::close(_uart_fd);
		_uart_fd = -1;
	}
}

void UorbUartProxy::setup_outgoing_subscriptions()
{
	// Helper to add a subscription
	auto add_sub = [this](const orb_metadata * meta, uint8_t instance, uint16_t topic_id, const char *name) {
		if (_out_sub_count >= MAX_OUT_SUBS) {
			PX4_ERR("Too many outgoing subscriptions");
			return;
		}

		OutgoingSub &sub = _out_subs[_out_sub_count++];
		sub.sub = uORB::Subscription(meta, instance);
		sub.topic_id = topic_id;
		sub.name = name;
	};

	// Status topics (instance 0)
	add_sub(ORB_ID(boom_status), 0, static_cast<uint16_t>(TopicIdRange::BOOM_STATUS), "boom_status");
	add_sub(ORB_ID(bucket_status), 0, static_cast<uint16_t>(TopicIdRange::BUCKET_STATUS), "bucket_status");
	add_sub(ORB_ID(steering_status), 0, static_cast<uint16_t>(TopicIdRange::STEERING_STATUS), "steering_status");

	// Multi-instance sensor topics
	// sensor_limit_switch: 4 instances
	for (uint8_t i = 0; i < 4; i++) {
		add_sub(ORB_ID(sensor_limit_switch), i, static_cast<uint16_t>(TopicIdRange::SENSOR_LIMIT_SWITCH),
			"sensor_limit_switch");
	}

	// sensor_quad_encoder: 2 instances
	for (uint8_t i = 0; i < 2; i++) {
		add_sub(ORB_ID(sensor_quad_encoder), i, static_cast<uint16_t>(TopicIdRange::SENSOR_QUAD_ENCODER),
			"sensor_quad_encoder");
	}

	// hbridge_status: 2 instances
	for (uint8_t i = 0; i < 2; i++) {
		add_sub(ORB_ID(hbridge_status), i, static_cast<uint16_t>(TopicIdRange::HBRIDGE_STATUS), "hbridge_status");
	}

	PX4_INFO("Setup %zu outgoing subscriptions", _out_sub_count);
}

void UorbUartProxy::run()
{
	while (!should_exit()) {
		send_outgoing();
		receive_incoming();

		hrt_abstime now = hrt_absolute_time();

		if (now - _last_heartbeat_time >= HEARTBEAT_INTERVAL) {
			send_heartbeat();
			_last_heartbeat_time = now;
		}

		usleep(10000); // 10ms
	}
}

void UorbUartProxy::send_outgoing()
{
	for (size_t i = 0; i < _out_sub_count; i++) {
		OutgoingSub &sub = _out_subs[i];

		if (sub.sub.updated()) {
			// Copy message data
			if (!sub.sub.copy(_msg_buf)) {
				continue;
			}

			// Get message size from metadata
			const orb_metadata *meta = sub.sub.get_topic();

			if (meta == nullptr) {
				continue;
			}

			size_t msg_size = meta->o_size;

			if (msg_size > MSG_BUF_SIZE) {
				PX4_ERR("Message too large: %zu > %zu", msg_size, MSG_BUF_SIZE);
				continue;
			}

			// Build frame
			uint8_t instance = sub.sub.get_instance();

			// Determine source node ID from parameter
			NodeId source_node;
			int32_t node_id_param = _param_node_id.get();

			if (node_id_param == static_cast<int32_t>(NodeId::NXT_FRONT)) {
				source_node = NodeId::NXT_FRONT;
			} else if (node_id_param == static_cast<int32_t>(NodeId::NXT_REAR)) {
				source_node = NodeId::NXT_REAR;
			} else {
				source_node = NodeId::RESERVED;
			}

			size_t frame_size = _frame_builder.build_data_frame(
						    _tx_buf, TX_BUF_SIZE,
						    sub.topic_id, instance,
						    source_node, NodeId::BROADCAST,
						    _msg_buf, msg_size
					    );

			if (frame_size == 0) {
				PX4_ERR("Failed to build frame for topic_id=0x%04x", sub.topic_id);
				continue;
			}

			// Send frame
			ssize_t written = ::write(_uart_fd, _tx_buf, frame_size);

			if (written != (ssize_t)frame_size) {
				_rx_errors++;
			} else {
				_tx_frames++;
				_tx_bytes += frame_size;
			}
		}
	}
}

void UorbUartProxy::receive_incoming()
{
	if (_uart_fd < 0) {
		return;
	}

	// Read available data into RX buffer
	size_t space = RX_BUF_SIZE - _rx_len;

	if (space == 0) {
		// Buffer full, discard oldest data
		PX4_WARN("RX buffer full, discarding data");
		_rx_len = 0;
		_rx_errors++;
		return;
	}

	ssize_t n = ::read(_uart_fd, _rx_buf + _rx_len, space);

	if (n > 0) {
		_rx_len += n;
		_rx_bytes += n;
		_last_rx_time = hrt_absolute_time();

		// Parse accumulated data
		parse_rx_buffer();
	}
}

void UorbUartProxy::parse_rx_buffer()
{
	while (_rx_len > 0) {
		// Look for sync bytes (0xFF 0xFE)
		size_t sync_pos = 0;
		bool found_sync = false;

		for (size_t i = 0; i < _rx_len - 1; i++) {
			if (_rx_buf[i] == 0xFF && _rx_buf[i + 1] == 0xFE) {
				sync_pos = i;
				found_sync = true;
				break;
			}
		}

		if (!found_sync) {
			// No sync found, keep last byte in case it's the first sync byte
			if (_rx_len > 1) {
				_rx_buf[0] = _rx_buf[_rx_len - 1];
				_rx_len = 1;
			}

			return;
		}

		// Discard data before sync
		if (sync_pos > 0) {
			memmove(_rx_buf, _rx_buf + sync_pos, _rx_len - sync_pos);
			_rx_len -= sync_pos;
		}

		// Need at least 6 bytes to read frame_length field
		// ProtocolFrame: sync1(1) + sync2(1) + protocol_version(1) + message_type(1) + frame_length(2)
		if (_rx_len < 6) {
			return;  // Wait for more data
		}

		// Read frame_length (bytes 4-5, little-endian uint16_t)
		uint16_t frame_length = _rx_buf[4] | (_rx_buf[5] << 8);

		// Validate frame_length
		static constexpr size_t MIN_FRAME_SIZE = 22;  // 18-byte header + 4-byte CRC
		static constexpr size_t MAX_FRAME_SIZE = 534; // 18 + 512 + 4

		if (frame_length < MIN_FRAME_SIZE || frame_length > MAX_FRAME_SIZE) {
			// Invalid frame length, skip sync bytes and resync
			memmove(_rx_buf, _rx_buf + 2, _rx_len - 2);
			_rx_len -= 2;
			_parse_errors++;
			continue;
		}

		// Wait for complete frame
		if (_rx_len < frame_length) {
			return;  // Need more data
		}

		// Try to parse frame
		const ProtocolFrame *frame = _frame_parser.parse_frame(_rx_buf, frame_length);

		if (frame != nullptr) {
			// Valid frame, dispatch it
			const uint8_t *payload = _rx_buf + sizeof(ProtocolFrame);
			dispatch_frame(frame, payload);

			_rx_frames++;

			// Consume frame
			memmove(_rx_buf, _rx_buf + frame_length, _rx_len - frame_length);
			_rx_len -= frame_length;
		} else {
			// Parse failed, skip sync bytes and resync
			memmove(_rx_buf, _rx_buf + 2, _rx_len - 2);
			_rx_len -= 2;
			_parse_errors++;
		}
	}
}

void UorbUartProxy::dispatch_frame(const ProtocolFrame *frame, const uint8_t *payload)
{
	switch (frame->message_type) {
	case static_cast<uint8_t>(MessageType::DATA):
		publish_incoming(frame->topic_id, frame->instance, payload, frame->payload_length);
		break;

	case static_cast<uint8_t>(MessageType::HEARTBEAT):
		// Just update last RX time (already done in receive_incoming)
		break;

	default:
		// Unknown message type
		break;
	}
}

void UorbUartProxy::publish_incoming(uint16_t topic_id, uint8_t instance, const uint8_t *data, size_t size)
{
	// Get topic metadata
	const orb_metadata *meta = get_topic_meta(topic_id);

	if (meta == nullptr) {
		return;
	}

	// Validate size
	if (size != meta->o_size) {
		PX4_WARN("Size mismatch for topic_id=0x%04x: got %zu, expected %zu", topic_id, size, meta->o_size);
		return;
	}

	// Find or create publication
	IncomingPub *pub = nullptr;

	for (size_t i = 0; i < _in_pub_count; i++) {
		if (_in_pubs[i].topic_id == topic_id && _in_pubs[i].instance == instance) {
			pub = &_in_pubs[i];
			break;
		}
	}

	if (pub == nullptr) {
		// Create new publication
		if (_in_pub_count >= MAX_IN_PUBS) {
			PX4_ERR("Too many incoming publications");
			return;
		}

		pub = &_in_pubs[_in_pub_count++];
		pub->topic_id = topic_id;
		pub->instance = instance;
		pub->meta = meta;

		// Advertise with specific instance
		int instance_copy = instance;
		pub->handle = orb_advertise_multi(meta, data, &instance_copy);

		if (pub->handle == nullptr) {
			PX4_ERR("Failed to advertise topic_id=0x%04x instance=%u", topic_id, instance);
			_in_pub_count--;
			return;
		}

		PX4_INFO("Created publication for topic_id=0x%04x instance=%u", topic_id, instance);
	} else {
		// Publish to existing handle
		orb_publish(meta, pub->handle, data);
	}
}

const orb_metadata *UorbUartProxy::get_topic_meta(uint16_t topic_id)
{
	// Map topic IDs to metadata
	switch (static_cast<TopicIdRange>(topic_id)) {
	case TopicIdRange::BOOM_CONTROL_SETPOINT:
		return ORB_ID(boom_control_setpoint);

	case TopicIdRange::TILT_CONTROL_SETPOINT:
		return ORB_ID(tilt_control_setpoint);

	default:
		return nullptr;
	}
}

void UorbUartProxy::send_heartbeat()
{
	// Determine source node ID from parameter
	NodeId source_node;
	int32_t node_id_param = _param_node_id.get();

	if (node_id_param == static_cast<int32_t>(NodeId::NXT_FRONT)) {
		source_node = NodeId::NXT_FRONT;
	} else if (node_id_param == static_cast<int32_t>(NodeId::NXT_REAR)) {
		source_node = NodeId::NXT_REAR;
	} else {
		source_node = NodeId::RESERVED;
	}

	HeartbeatPayload hb{};
	hb.timestamp = hrt_absolute_time();
	hb.uptime_ms = hrt_absolute_time() / 1000;
	hb.system_health = 100;
	hb.cpu_load = 0;
	hb.free_memory_kb = 0;
	hb.tx_packets = _tx_frames;
	hb.rx_packets = _rx_frames;
	hb.tx_errors = 0;
	hb.rx_errors = _rx_errors;

	size_t frame_size = _frame_builder.build_heartbeat_frame(
				    _tx_buf, TX_BUF_SIZE,
				    source_node, hb,
				    _tx_seq++
			    );

	if (frame_size > 0) {
		ssize_t written = ::write(_uart_fd, _tx_buf, frame_size);

		if (written == (ssize_t)frame_size) {
			_tx_frames++;
			_tx_bytes += frame_size;
		}
	}
}

int UorbUartProxy::print_status()
{
	PX4_INFO("uORB UART Proxy Status");
	PX4_INFO("  Device: %s", _uart_device);
	PX4_INFO("  Node ID: %ld", (long)_param_node_id.get());
	PX4_INFO("  Baud: %ld", (long)_param_baud.get());
	PX4_INFO("  UART FD: %d", _uart_fd);

	hrt_abstime now = hrt_absolute_time();
	hrt_abstime rx_age = now - _last_rx_time;
	bool connected = (rx_age < 3_s);

	PX4_INFO("  Connected: %s (last RX: %.1fs ago)", connected ? "yes" : "no", (double)rx_age / 1e6);

	PX4_INFO("Memory usage:");
	size_t out_subs_mem = sizeof(_out_subs);
	size_t in_pubs_mem = sizeof(_in_pubs);
	size_t buffers_mem = sizeof(_tx_buf) + sizeof(_rx_buf) + sizeof(_msg_buf);
	size_t total_mem = sizeof(*this);
	PX4_INFO("  Class size: %zu bytes", total_mem);
	PX4_INFO("  Outgoing subs: %zu bytes (%zu used)", out_subs_mem, _out_sub_count * sizeof(OutgoingSub));
	PX4_INFO("  Incoming pubs: %zu bytes (%zu used)", in_pubs_mem, _in_pub_count * sizeof(IncomingPub));
	PX4_INFO("  Buffers: %zu bytes (TX=%zu, RX=%zu, MSG=%zu)", buffers_mem,
		 sizeof(_tx_buf), sizeof(_rx_buf), sizeof(_msg_buf));
	PX4_INFO("  RX buffer fill: %zu / %zu bytes (%.1f%%)", _rx_len, sizeof(_rx_buf),
		 100.0 * _rx_len / sizeof(_rx_buf));

	PX4_INFO("Statistics:");
	PX4_INFO("  TX: %lu frames, %lu bytes", (unsigned long)_tx_frames, (unsigned long)_tx_bytes);
	PX4_INFO("  RX: %lu frames, %lu bytes", (unsigned long)_rx_frames, (unsigned long)_rx_bytes);
	PX4_INFO("  Errors: RX=%lu, Parse=%lu", (unsigned long)_rx_errors, (unsigned long)_parse_errors);

	PX4_INFO("Outgoing subscriptions: %zu", _out_sub_count);

	for (size_t i = 0; i < _out_sub_count; i++) {
		const OutgoingSub &sub = _out_subs[i];
		PX4_INFO("  [%zu] %s (0x%04x) instance=%u", i, sub.name, sub.topic_id, sub.sub.get_instance());
	}

	PX4_INFO("Incoming publications: %zu", _in_pub_count);

	for (size_t i = 0; i < _in_pub_count; i++) {
		const IncomingPub &pub = _in_pubs[i];
		PX4_INFO("  [%zu] topic_id=0x%04x instance=%u", i, pub.topic_id, pub.instance);
	}

	return 0;
}

int UorbUartProxy::task_spawn(int argc, char *argv[])
{
	_task_id = px4_task_spawn_cmd("uorb_uart_proxy",
				      SCHED_DEFAULT,
				      SCHED_PRIORITY_DEFAULT,
				      4096,
				      (px4_main_t)&UorbUartProxy::run_trampoline,
				      (char *const *)argv);

	if (_task_id < 0) {
		_task_id = -1;
		return -errno;
	}

	return 0;
}

int UorbUartProxy::run_trampoline(int argc, char *argv[])
{
	UorbUartProxy *instance = new UorbUartProxy();

	if (!instance) {
		PX4_ERR("alloc failed");
		return -1;
	}

	_object.store(instance);

	if (instance->init()) {
		instance->run();
	}

	_object.store(nullptr);
	_task_id = -1;
	delete instance;
	return 0;
}

int UorbUartProxy::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int UorbUartProxy::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
uORB UART Proxy for NXT controller boards.

Forwards uORB topics over UART to/from the X7+ main board using the
distributed uORB protocol.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uorb_uart_proxy", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int uorb_uart_proxy_main(int argc, char *argv[])
{
	return UorbUartProxy::main(argc, argv);
}
