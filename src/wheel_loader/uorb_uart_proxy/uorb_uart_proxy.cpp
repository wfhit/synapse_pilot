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
 * @file uorb_uart_proxy.cpp
 * @author PX4 Development Team
 *
 * Refactored uORB UART Proxy implementation for NXT controller boards
 * Uses improved distributed uORB protocol for robust communication
 */

#include "uorb_uart_proxy.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_hrt.h>
#include <cstring>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace distributed_uorb;

UorbUartProxy::UorbUartProxy() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_comms_error_perf(perf_alloc(PC_COUNT, MODULE_NAME": comm_errors")),
	_packet_tx_perf(perf_alloc(PC_COUNT, MODULE_NAME": tx_packets")),
	_packet_rx_perf(perf_alloc(PC_COUNT, MODULE_NAME": rx_packets")),
	_bytes_tx_perf(perf_alloc(PC_COUNT, MODULE_NAME": tx_bytes")),
	_bytes_rx_perf(perf_alloc(PC_COUNT, MODULE_NAME": rx_bytes"))
{
}

UorbUartProxy::~UorbUartProxy()
{
	cleanup_topic_handlers();

	if (_uart_fd >= 0) {
		close(_uart_fd);
		_uart_fd = -1;
	}

	perf_free(_loop_perf);
	perf_free(_comms_error_perf);
	perf_free(_packet_tx_perf);
	perf_free(_packet_rx_perf);
	perf_free(_bytes_tx_perf);
	perf_free(_bytes_rx_perf);
}

bool UorbUartProxy::init()
{
	// Detect board type based on parameter or auto-detection
	_node_id = detect_board_type();

	if (_node_id == NodeId::RESERVED) {
		PX4_ERR("Failed to detect board type");
		return false;
	}

	// Configure UART device path
	snprintf(_device_path, sizeof(_device_path), "/dev/ttyS%ld", _param_uart_device.get());

	if (!configure_uart()) {
		PX4_ERR("Failed to configure UART");
		return false;
	}

	if (!setup_topic_handlers()) {
		PX4_ERR("Failed to setup topic handlers");
		return false;
	}

	ScheduleOnInterval(SCHEDULE_INTERVAL);
	PX4_INFO("uORB UART Proxy initialized for node %d", static_cast<int>(_node_id));
	return true;
}

NodeId UorbUartProxy::detect_board_type()
{
	// Use parameter if set explicitly
	int node_param = _param_node_id.get();

	if (node_param == static_cast<int>(NodeId::NXT_FRONT)) {
		return NodeId::NXT_FRONT;

	} else if (node_param == static_cast<int>(NodeId::NXT_REAR)) {
		return NodeId::NXT_REAR;
	}

	// Auto-detection based on hardware characteristics or configuration
	// For now, default to FRONT if not specified
	PX4_WARN("Node ID not specified, defaulting to NXT_FRONT");
	return NodeId::NXT_FRONT;
}

bool UorbUartProxy::configure_uart()
{
	if (_is_connected) {
		return true;
	}

	// Close existing connection if open
	if (_uart_fd >= 0) {
		close(_uart_fd);
		_uart_fd = -1;
	}

	PX4_INFO("Opening UART %s...", _device_path);
	_uart_fd = open(_device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart_fd < 0) {
		handle_communication_error("Failed to open UART");
		return false;
	}

	PX4_INFO("UART opened successfully (fd=%d)", _uart_fd);

	struct termios uart_config;

	if (tcgetattr(_uart_fd, &uart_config) != 0) {
		PX4_ERR("Error getting UART attributes: %s", strerror(errno));
		close(_uart_fd);
		_uart_fd = -1;
		return false;
	}

	// Get baudrate parameter (default to 115200)
	int32_t baudrate = _param_baudrate.get();

	if (baudrate <= 0) {
		baudrate = 115200;  // Default baudrate
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

	if (tcsetattr(_uart_fd, TCSANOW, &uart_config) != 0) {
		PX4_ERR("Error setting UART attributes: %s", strerror(errno));
		close(_uart_fd);
		_uart_fd = -1;
		return false;
	}

	// Clear any existing data
	tcflush(_uart_fd, TCIOFLUSH);

	_is_connected = true;
	PX4_INFO("Configured UART on %s", _device_path);
	return true;
}

void UorbUartProxy::Run()
{
	if (should_exit()) {
		return;
	}

	perf_begin(_loop_perf);

	// Check and maintain connection
	check_connection();

	// Process messages in both directions
	process_outgoing_messages();
	process_incoming_messages();

	// Send periodic heartbeat
	send_heartbeat();

	perf_end(_loop_perf);
}

void UorbUartProxy::check_connection()
{
	if (!_is_connected) {
		configure_uart();
	}

	// Check for connection timeout
	const hrt_abstime now = hrt_absolute_time();

	if (_last_message_time > 0 &&
	    (now - _last_message_time) > (CONNECTION_TIMEOUT_MS * 1000)) {

		PX4_WARN("Connection timeout, attempting reconnect");
		_is_connected = false;

		if (_uart_fd >= 0) {
			close(_uart_fd);
			_uart_fd = -1;
		}

		configure_uart();
	}
}

void UorbUartProxy::send_heartbeat()
{
	if (!_is_connected) {
		return;
	}

	const hrt_abstime now = hrt_absolute_time();

	if ((now - _last_heartbeat_time) >= (HEARTBEAT_INTERVAL_MS * 1000)) {
		// Create heartbeat payload
		distributed_uorb::HeartbeatPayload payload{};
		payload.timestamp = hrt_absolute_time();
		payload.uptime_ms = static_cast<uint32_t>(hrt_absolute_time() / 1000);
		payload.system_health = 100;  // Assume healthy
		payload.cpu_load = 50;        // Placeholder
		payload.free_memory_kb = 64;  // Placeholder
		payload.tx_packets = _stats.tx_packets;
		payload.rx_packets = _stats.rx_packets;
		payload.tx_errors = _stats.tx_errors;
		payload.rx_errors = _stats.rx_errors;

		size_t frame_size = _frame_builder.build_heartbeat_frame(
					    _tx_buffer,
					    IO_BUFFER_SIZE,
					    _node_id,
					    payload,
					    _tx_sequence++
				    );

		if (frame_size > 0) {
			send_frame(_tx_buffer, frame_size);
			perf_count(_packet_tx_perf);
		}

		_last_heartbeat_time = now;
	}
}

bool UorbUartProxy::setup_topic_handlers()
{
	// Initialize topic registry with relevant topics for this node type
	using namespace distributed_uorb;

	// Register topics based on node type
	if (_node_id == NodeId::NXT_FRONT || _node_id == NodeId::NXT_REAR) {
		// Common topics for both NXT nodes
		g_topic_registry.register_topic("boom_status", ORB_ID(boom_status));
		g_topic_registry.register_topic("bucket_status", ORB_ID(bucket_status));
		g_topic_registry.register_topic("steering_status", ORB_ID(steering_status));
		g_topic_registry.register_topic("sensor_limit_switch", ORB_ID(sensor_limit_switch));
		g_topic_registry.register_topic("sensor_quad_encoder", ORB_ID(sensor_quad_encoder));
		g_topic_registry.register_topic("hbridge_status", ORB_ID(hbridge_status));

		// Incoming command topics
		g_topic_registry.register_topic("boom_control_setpoint", ORB_ID(boom_control_setpoint));
		g_topic_registry.register_topic("tilt_control_setpoint", ORB_ID(tilt_control_setpoint));
	}

	PX4_INFO("Registered %zu topics in distributed registry", g_topic_registry.get_topic_count());
	return true;
}

void UorbUartProxy::cleanup_topic_handlers()
{
	for (size_t i = 0; i < _topic_handler_count; i++) {
		if (_topic_handlers[i].subscription != nullptr) {
			delete _topic_handlers[i].subscription;
			_topic_handlers[i].subscription = nullptr;
		}

		if (_topic_handlers[i].publication != nullptr) {
			orb_unadvertise(_topic_handlers[i].publication);
			_topic_handlers[i].publication = nullptr;
		}
	}

	_topic_handler_count = 0;
}

void UorbUartProxy::process_outgoing_messages()
{
	if (!_is_connected) {
		return;
	}

	// Get all registered topics for sending status to X7+
	static constexpr size_t MAX_LOCAL_TOPICS = 16;
	const TopicInfo *topics[MAX_LOCAL_TOPICS];
	size_t topic_count = g_topic_registry.get_all_topics(topics, MAX_LOCAL_TOPICS);

	// Send status information from NXT to X7+
	for (size_t i = 0; i < topic_count; i++) {
		const TopicInfo *topic_info = topics[i];

		if (!is_topic_relevant_for_node(topic_info->topic_id, _node_id)) {
			continue;
		}

		// Skip topics larger than our buffer
		if (topic_info->meta->o_size > MAX_TOPIC_DATA_SIZE) {
			continue;
		}

		// Use stack-minimal subscription check
		uORB::Subscription sub(topic_info->meta);

		// Check for new data to send upstream
		if (sub.updated()) {
			if (sub.copy(_topic_data)) {
				size_t frame_size = _frame_builder.build_data_frame(
							    _tx_buffer, IO_BUFFER_SIZE,
							    topic_info->topic_id, 0,
							    _node_id, NodeId::X7_MAIN,
							    _topic_data, topic_info->meta->o_size,
							    Priority::NORMAL, Reliability::BEST_EFFORT,
							    _tx_sequence++);

				if (frame_size > 0) {
					send_frame(_tx_buffer, frame_size);
					_stats.tx_packets++;
					perf_count(_packet_tx_perf);
					perf_count(_bytes_tx_perf);
				}
			}
		}
	}
}

void UorbUartProxy::process_incoming_messages()
{
	if (_is_connected) {
		receive_frames();
	}
}

bool UorbUartProxy::is_topic_relevant_for_node(uint16_t topic_id, NodeId node_id)
{
	const TopicInfo *topic_info = g_topic_registry.get_topic_info(topic_id);

	if (topic_info == nullptr || topic_info->meta == nullptr) {
		return false;
	}

	const char *topic_name = topic_info->meta->o_name;

	// Topics that NXT nodes send to X7+ (outgoing from NXT perspective)
	if (strcmp(topic_name, "boom_status") == 0 ||
	    strcmp(topic_name, "bucket_status") == 0 ||
	    strcmp(topic_name, "steering_status") == 0 ||
	    strcmp(topic_name, "sensor_limit_switch") == 0 ||
	    strcmp(topic_name, "sensor_quad_encoder") == 0 ||
	    strcmp(topic_name, "hbridge_status") == 0) {
		return true;
	}

	// Topics that X7+ sends to NXT nodes (incoming from NXT perspective)
	if (strcmp(topic_name, "boom_control_setpoint") == 0 ||
	    strcmp(topic_name, "tilt_control_setpoint") == 0) {
		return true;
	}

	return false;
}

bool UorbUartProxy::send_frame(const uint8_t *frame_data, size_t frame_size)
{
	if (!_is_connected || _uart_fd < 0) {
		return false;
	}

	ssize_t written = write(_uart_fd, frame_data, frame_size);

	if (written != static_cast<ssize_t>(frame_size)) {
		_stats.tx_errors++;
		handle_communication_error("UART write failed");
		return false;
	}

	return true;
}

bool UorbUartProxy::receive_frames()
{
	if (!_is_connected || _uart_fd < 0) {
		return false;
	}

	ssize_t bytes_read = read(_uart_fd, _rx_buffer, IO_BUFFER_SIZE);

	if (bytes_read > 0) {
		_last_message_time = hrt_absolute_time();
		perf_count(_bytes_rx_perf);

		// Parse received data for complete frames
		for (ssize_t i = 0; i < bytes_read;) {
			const ProtocolFrame *frame = _frame_parser.parse_frame(&_rx_buffer[i], bytes_read - i);

			if (frame != nullptr) {
				handle_received_frame(frame);
				_stats.rx_packets++;
				perf_count(_packet_rx_perf);
				// Calculate frame size including header and footer
				size_t frame_size = sizeof(ProtocolFrame) + frame->payload_length + sizeof(uint32_t);
				i += frame_size;

			} else {
				i++; // Skip this byte and continue parsing
			}
		}

		return true;
	}

	return false;
}

void UorbUartProxy::handle_received_frame(const ProtocolFrame *frame)
{
	if (frame == nullptr) {
		return;
	}

	switch (static_cast<uint8_t>(frame->message_type)) {
	case static_cast<uint8_t>(MessageType::DATA): {
			// Publish received command data to local uORB
			publish_to_local_topic(frame->topic_id, frame->instance, frame->payload, frame->payload_length);
			break;
		}

	case static_cast<uint8_t>(MessageType::HEARTBEAT): {
			// Connection is alive, no action needed
			break;
		}

	case static_cast<uint8_t>(MessageType::ACK): {
			// Handle acknowledgment if needed
			break;
		}

	default:
		PX4_WARN("Unknown message type: %d", static_cast<int>(frame->message_type));
		break;
	}
}

bool UorbUartProxy::publish_to_local_topic(uint16_t topic_id, uint8_t instance, const void *data, size_t data_size)
{
	const TopicInfo *topic_info = g_topic_registry.get_topic_info(topic_id);

	if (topic_info == nullptr || topic_info->meta == nullptr) {
		PX4_WARN("Unknown topic ID: %d", topic_id);
		return false;
	}

	const orb_metadata *meta = topic_info->meta;

	if (data_size != meta->o_size) {
		PX4_WARN("Size mismatch for topic %s: expected %zu, got %zu", meta->o_name, meta->o_size, data_size);
		return false;
	}

	// Find or create topic handler
	ProxyTopicHandler *handler = find_topic_handler(topic_id, instance);

	if (handler == nullptr) {
		// Create new handler if we have space
		if (_topic_handler_count < MAX_TOPIC_HANDLERS) {
			ProxyTopicHandler *new_handler = &_topic_handlers[_topic_handler_count];
			new_handler->topic_id = topic_id;
			new_handler->meta = meta;
			new_handler->subscription = nullptr;
			int instance_int = static_cast<int>(instance);
			new_handler->publication = orb_advertise_multi(meta, data, &instance_int);
			new_handler->instance = instance;
			new_handler->last_updated = hrt_absolute_time();
			new_handler->is_outgoing = false;
			_topic_handler_count++;

		} else {
			PX4_WARN("Maximum topic handlers reached (%zu)", MAX_TOPIC_HANDLERS);
			return false;
		}

	} else {
		// Update existing publication
		if (handler->publication != nullptr) {
			orb_publish(meta, handler->publication, data);
			handler->last_updated = hrt_absolute_time();
		}
	}

	return true;
}

ProxyTopicHandler *UorbUartProxy::find_topic_handler(uint16_t topic_id, uint8_t instance)
{
	for (size_t i = 0; i < _topic_handler_count; i++) {
		if (_topic_handlers[i].topic_id == topic_id && _topic_handlers[i].instance == instance) {
			return &_topic_handlers[i];
		}
	}

	return nullptr;
}

void UorbUartProxy::handle_communication_error(const char *error_msg)
{
	_stats.tx_errors++;
	perf_count(_comms_error_perf);
	PX4_WARN("Communication error: %s", error_msg);
}

int UorbUartProxy::print_status()
{
	PX4_INFO("uORB UART Proxy status:");
	PX4_INFO("  Node ID: %d", static_cast<int>(_node_id));
	PX4_INFO("  Connected: %s", _is_connected ? "YES" : "NO");
	PX4_INFO("  Device: %s", _device_path);
	PX4_INFO("  TX packets: %lu (errors: %lu)", (unsigned long)_stats.tx_packets, (unsigned long)_stats.tx_errors);
	PX4_INFO("  RX packets: %lu (errors: %lu)", (unsigned long)_stats.rx_packets, (unsigned long)_stats.rx_errors);
	PX4_INFO("  Topic handlers: %zu", _topic_handler_count);

	return 0;
}

int UorbUartProxy::task_spawn(int argc, char *argv[])
{
	UorbUartProxy *instance = new UorbUartProxy();

	if (instance == nullptr) {
		PX4_ERR("Failed to allocate instance");
		return PX4_ERROR;
	}

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (!instance->init()) {
		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		PX4_ERR("Failed to initialize");
		return PX4_ERROR;
	}

	return PX4_OK;
}

int UorbUartProxy::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = UorbUartProxy::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

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
uORB UART Proxy for NXT controller boards in Wheel Loader Robot.

This module runs on NXT-Dual controller boards and provides transparent
uORB messaging to/from the X7+ main board via UART. It uses an improved
distributed uORB protocol with CRC32 validation and board-type awareness.

### Examples
Start the proxy:
$ uorb_uart_proxy start

Check status:
$ uorb_uart_proxy status

Stop the proxy:
$ uorb_uart_proxy stop
	)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("uorb_uart_proxy", "communication");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	return 0;
}

extern "C" __EXPORT int uorb_uart_proxy_main(int argc, char *argv[])
{
	return UorbUartProxy::main(argc, argv);
}
