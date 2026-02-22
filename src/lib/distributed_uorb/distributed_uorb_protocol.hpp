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
 * @file distributed_uorb_protocol.hpp
 * @author PX4 Development Team
 *
 * Distributed uORB protocol definitions and utilities
 * Shared between uorb_uart_bridge and uorb_uart_proxy modules
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <cstdint>
#include <cstddef>

namespace distributed_uorb
{

// Protocol version
static constexpr uint8_t PROTOCOL_VERSION = 2;

// Frame sync bytes for packet detection
static constexpr uint8_t SYNC_BYTE1 = 0xFF;
static constexpr uint8_t SYNC_BYTE2 = 0xFE;

// Node identifiers
enum class NodeId : uint8_t {
	BROADCAST = 0,
	X7_MAIN = 1,
	NXT_FRONT = 2,
	NXT_REAR = 3,
	RESERVED = 255
};

// Message types
enum class MessageType : uint8_t {
	DATA = 0x01,         // Topic data message
	SUBSCRIBE = 0x02,    // Subscribe request
	ADVERTISE = 0x03,    // Advertise topic
	HEARTBEAT = 0x04,    // Keep-alive message
	TIME_SYNC = 0x05,    // Time synchronization
	TOPIC_INFO = 0x06,   // Topic information exchange
	ERROR = 0x07,        // Error notification
	ACK = 0x08,          // Acknowledgment
	DISCOVERY = 0x09     // Node/topic discovery
};

// Priority levels
enum class Priority : uint8_t {
	LOW = 0,
	NORMAL = 1,
	HIGH = 2,
	CRITICAL = 3
};

// Reliability levels
enum class Reliability : uint8_t {
	BEST_EFFORT = 0,     // No delivery guarantee
	RELIABLE = 1,        // At least once delivery
	GUARANTEED = 2       // Exactly once delivery
};

// Compression types
enum class Compression : uint8_t {
	NONE = 0,
	ZLIB = 1,
	LZ4 = 2
};

// Error flags
enum class ErrorFlags : uint8_t {
	NONE = 0x00,
	CHECKSUM_FAILED = 0x01,
	TIMEOUT = 0x02,
	BUFFER_OVERFLOW = 0x04,
	PROTOCOL_ERROR = 0x08,
	NODE_OFFLINE = 0x10,
	TOPIC_NOT_FOUND = 0x20,
	PERMISSION_DENIED = 0x40,
	UNKNOWN_ERROR = 0x80
};

// Maximum payload size (optimized for common uORB messages)
static constexpr size_t MAX_PAYLOAD_SIZE = 512;

// Timing constants
static constexpr uint32_t DEFAULT_HEARTBEAT_INTERVAL_MS = 1000;
static constexpr uint32_t DEFAULT_CONNECTION_TIMEOUT_MS = 3000;
static constexpr uint32_t DEFAULT_ACK_TIMEOUT_MS = 100;
static constexpr uint32_t DEFAULT_RETRY_INTERVAL_MS = 50;
static constexpr uint8_t DEFAULT_MAX_RETRIES = 3;

/**
 * Wire protocol frame structure
 * Total overhead: 20 bytes + payload
 */
struct __attribute__((packed)) ProtocolFrame {
	// Frame header (6 bytes)
	uint8_t sync1;              // Sync byte 1 (0xFF)
	uint8_t sync2;              // Sync byte 2 (0xFE)
	uint8_t protocol_version;   // Protocol version
	uint8_t message_type;       // MessageType enum
	uint16_t frame_length;      // Total frame length including header

	// Message metadata (8 bytes)
	uint16_t topic_id;          // uORB topic identifier
	uint8_t instance;           // Multi-instance topic instance
	uint8_t source_node;        // Source NodeId
	uint8_t dest_node;          // Destination NodeId
	uint16_t sequence_num;      // Sequence number
	uint8_t flags;              // Combination of priority, reliability, compression

	// Quality of service (4 bytes)
	uint16_t payload_length;    // Actual payload data length
	uint16_t ttl_ms;            // Time-to-live in milliseconds

	// Payload data (variable length, max MAX_PAYLOAD_SIZE)
	uint8_t payload[0];

	// Footer (CRC32) follows payload data
};

/**
 * Message flags bit layout:
 * Bits 0-1: Priority (Priority enum)
 * Bits 2-3: Reliability (Reliability enum)
 * Bits 4-5: Compression (Compression enum)
 * Bits 6-7: Reserved
 */
inline uint8_t encode_flags(Priority priority, Reliability reliability, Compression compression)
{
	return static_cast<uint8_t>(priority) |
	       (static_cast<uint8_t>(reliability) << 2) |
	       (static_cast<uint8_t>(compression) << 4);
}

inline void decode_flags(uint8_t flags, Priority &priority, Reliability &reliability, Compression &compression)
{
	priority = static_cast<Priority>(flags & 0x03);
	reliability = static_cast<Reliability>((flags >> 2) & 0x03);
	compression = static_cast<Compression>((flags >> 4) & 0x03);
}

/**
 * Heartbeat message payload
 *
 * Time sync is piggybacked on heartbeat for reliability.
 * Uses NTP-like 4-timestamp algorithm:
 *   1. Master heartbeat carries ts_t1 (master send time) + ts_seq
 *   2. Slave stores ts_t1 and records ts_t2 (local receive time)
 *   3. Slave heartbeat echoes ts_t1, ts_t2, and sets ts_t3 (slave send time)
 *   4. Master receives, records t4, computes offset and RTT
 *   5. Master sends computed offset/quality in next heartbeat to slave
 */
struct __attribute__((packed)) HeartbeatPayload {
	uint64_t timestamp;         // Current timestamp
	uint32_t uptime_ms;         // Node uptime in milliseconds
	uint8_t system_health;      // System health status (0-100%)
	uint8_t cpu_load;           // CPU load percentage
	uint16_t free_memory_kb;    // Available memory in KB
	uint32_t tx_packets;        // Total transmitted packets
	uint32_t rx_packets;        // Total received packets
	uint32_t tx_errors;         // Transmission error count
	uint32_t rx_errors;         // Reception error count
	// Time sync (piggybacked on heartbeat)
	int64_t ts_t1;              // Master send time (master sets, slave echoes)
	int64_t ts_t2;              // Slave receive time of master heartbeat
	int64_t ts_t3;              // Slave send time of this heartbeat
	int64_t ts_offset_us;       // Computed clock offset (master to slave)
	uint32_t ts_rtt_us;         // Computed RTT (master to slave)
	uint8_t ts_quality;         // Sync quality 0-100 (master to slave)
	uint8_t ts_seq;             // Sequence counter
};

/**
 * Topic information message payload
 */
struct __attribute__((packed)) TopicInfoPayload {
	uint16_t topic_id;          // Topic identifier
	uint8_t instance;           // Instance number
	char topic_name[32];        // Topic name string
	uint16_t message_size;      // Message structure size
	uint8_t publisher_count;    // Number of publishers
	uint8_t subscriber_count;   // Number of subscribers
	uint32_t publish_rate_hz;   // Publishing rate in Hz
	uint64_t last_published;    // Last publish timestamp
};

/**
 * Time synchronization payload
 *
 * Uses a 4-timestamp NTP-like round trip algorithm:
 *   1. Master sends TIME_SYNC_REQUEST with t1 (master send time)
 *   2. Slave receives, records t2 (slave receive time), replies with
 *      TIME_SYNC_RESPONSE carrying {t1, t2, t3} where t3 = slave send time
 *   3. Master receives, records t4 (master receive time), computes:
 *      - Round-trip time:  rtt    = (t4 - t1) - (t3 - t2)
 *      - Clock offset:     offset = ((t2 - t1) + (t3 - t4)) / 2
 *   4. Master sends TIME_SYNC_OFFSET with computed offset to slave
 *   5. Slave stores offset for timestamp correction
 */
struct __attribute__((packed)) TimeSyncPayload {
	uint8_t phase;              // 0=request, 1=response, 2=offset
	uint8_t reserved;
	int64_t t1;                 // Master send time (us)
	int64_t t2;                 // Slave receive time (us)
	int64_t t3;                 // Slave send time (us)
	int64_t offset_us;          // Computed clock offset (us), master−slave
	uint32_t rtt_us;            // Round-trip time (us)
	uint8_t quality;            // Sync quality 0-100 (filtered)
	uint8_t sequence_id;        // Matches request→response→offset
};

static constexpr uint8_t TIME_SYNC_PHASE_REQUEST  = 0;
static constexpr uint8_t TIME_SYNC_PHASE_RESPONSE = 1;
static constexpr uint8_t TIME_SYNC_PHASE_OFFSET   = 2;

// Time sync constants
static constexpr uint32_t TIME_SYNC_INTERVAL_MS = 2000;   // Send request every 2s
static constexpr uint32_t TIME_SYNC_RTT_MAX_US = 500000;  // Discard if RTT > 500ms (accounts for UART queuing)
static constexpr uint8_t TIME_SYNC_FILTER_SIZE = 8;       // Median filter window

/**
 * Error notification payload
 */
struct __attribute__((packed)) ErrorPayload {
	uint8_t error_code;         // ErrorFlags enum
	uint16_t failed_sequence;   // Sequence number that failed
	uint16_t topic_id;          // Related topic ID (if applicable)
	char error_message[64];     // Human-readable error description
};

/**
 * CRC32 calculation for data integrity
 */
uint32_t calculate_crc32(const void *data, size_t length);

/**
 * Frame validation
 */
bool validate_frame(const ProtocolFrame *frame, size_t total_length);

/**
 * Frame builder helper class
 */
class FrameBuilder
{
public:
	FrameBuilder() = default;
	~FrameBuilder() = default;

	/**
	 * Build a data frame
	 */
	size_t build_data_frame(uint8_t *buffer, size_t buffer_size,
				uint16_t topic_id, uint8_t instance,
				NodeId source, NodeId dest,
				const void *payload, size_t payload_size,
				Priority priority = Priority::NORMAL,
				Reliability reliability = Reliability::BEST_EFFORT,
				uint16_t sequence = 0, uint16_t ttl_ms = 1000);

	/**
	 * Build a heartbeat frame
	 */
	size_t build_heartbeat_frame(uint8_t *buffer, size_t buffer_size,
				     NodeId source, const HeartbeatPayload &payload,
				     uint16_t sequence = 0);

	/**
	 * Build a time sync frame
	 */
	size_t build_time_sync_frame(uint8_t *buffer, size_t buffer_size,
				     NodeId source, NodeId dest,
				     const TimeSyncPayload &payload,
				     uint16_t sequence = 0);

	/**
	 * Build an error frame
	 */
	size_t build_error_frame(uint8_t *buffer, size_t buffer_size,
				 NodeId source, NodeId dest,
				 const ErrorPayload &payload,
				 uint16_t sequence = 0);

private:
	uint16_t _next_sequence{1};
};

/**
 * Frame parser helper class
 */
class FrameParser
{
public:
	FrameParser() = default;
	~FrameParser() = default;

	/**
	 * Parse and validate a frame from buffer
	 */
	const ProtocolFrame *parse_frame(const uint8_t *buffer, size_t buffer_size);

	/**
	 * Get payload data from parsed frame
	 */
	const void *get_payload(const ProtocolFrame *frame) const;

	/**
	 * Get payload size from parsed frame
	 */
	size_t get_payload_size(const ProtocolFrame *frame) const;

private:
	bool validate_crc(const ProtocolFrame *frame, size_t total_length) const;
};

} // namespace distributed_uorb
