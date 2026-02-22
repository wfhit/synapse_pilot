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
 * @file distributed_uorb_topic_registry.hpp
 * @author PX4 Development Team
 *
 * Distributed uORB topic registry for dynamic topic mapping
 * Maps topic names to IDs and handles topic metadata
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <uORB/uORB.h>
#include <cstdint>
#include <cstring>

namespace distributed_uorb
{

// Maximum number of topics that can be registered
static constexpr size_t MAX_REGISTERED_TOPICS = 256;

// Topic ID ranges
enum class TopicIdRange : uint16_t {
	// System topics (0x0000-0x00FF)
	SYSTEM_BASE = 0x0000,
	HEARTBEAT = 0x0000,
	TIME_SYNC = 0x0001,
	NODE_INFO = 0x0002,

	// Control topics (0x0100-0x01FF)
	CONTROL_BASE = 0x0100,
	ACTUATOR_OUTPUTS = 0x0100,
	VEHICLE_STATUS = 0x0101,
	BOOM_CONTROL_SETPOINT = 0x0103,
	TILT_CONTROL_SETPOINT = 0x0104,
	STEERING_SETPOINT = 0x0105,
	LOAD_LAMP_COMMAND = 0x0106,
	TRACTION_SETPOINT = 0x0107,

	// Sensor topics (0x0200-0x02FF)
	SENSOR_BASE = 0x0200,
	SENSOR_LIMIT_SWITCH = 0x0200,
	SENSOR_QUAD_ENCODER = 0x0202,
	HBRIDGE_STATUS = 0x0203,

	// Status topics (0x0300-0x03FF)
	STATUS_BASE = 0x0300,
	BOOM_STATUS = 0x0300,
	BUCKET_STATUS = 0x0301,
	STEERING_STATUS = 0x0302,
	WHEEL_LOADER_STATUS = 0x0303,

	// Custom topics (0x1000-0xFFFF)
	CUSTOM_BASE = 0x1000,

	// Invalid topic
	INVALID = 0xFFFF
};

/**
 * Topic registration information
 */
struct TopicInfo {
	uint16_t topic_id;              // Unique topic identifier
	char name[32];                  // Topic name string
	const orb_metadata *meta;       // uORB metadata pointer
	size_t message_size;            // Message structure size
	bool is_distributed;            // True if topic should be distributed
	uint32_t publish_rate_hz;       // Expected publishing rate
	uint8_t priority;               // Distribution priority (0-3)
	bool requires_acknowledgment;   // True if reliable delivery needed
};

/**
 * Topic registry class for managing distributed topics
 */
class TopicRegistry
{
public:
	TopicRegistry() = default;
	~TopicRegistry() = default;

	/**
	 * Initialize the registry with default topics
	 */
	bool init();

	/**
	 * Register a topic for distribution
	 */
	bool register_topic(const char *name, const orb_metadata *meta,
			    bool is_distributed = true, uint32_t publish_rate_hz = 10,
			    uint8_t priority = 1, bool requires_ack = false);

	/**
	 * Get topic ID by name
	 */
	uint16_t get_topic_id(const char *name) const;

	/**
	 * Get topic info by ID
	 */
	const TopicInfo *get_topic_info(uint16_t topic_id) const;

	/**
	 * Get topic info by name
	 */
	const TopicInfo *get_topic_info(const char *name) const;

	/**
	 * Check if topic should be distributed
	 */
	bool is_distributed_topic(uint16_t topic_id) const;

	/**
	 * Get all registered topics
	 */
	size_t get_all_topics(const TopicInfo **topics, size_t max_topics) const;

	/**
	 * Get topic info by index (for stack-safe iteration)
	 */
	const TopicInfo *get_topic_by_index(size_t index) const;

	/**
	 * Get number of registered topics
	 */
	size_t get_topic_count() const { return _topic_count; }

	/**
	 * Get topic name by ID (returns nullptr if not found)
	 */
	const char *get_topic_name(uint16_t topic_id) const;

	/**
	 * Print registry statistics
	 */
	void print_statistics() const;

private:
	TopicInfo _topics[MAX_REGISTERED_TOPICS];
	size_t _topic_count{0};
	uint16_t _next_custom_id{static_cast<uint16_t>(TopicIdRange::CUSTOM_BASE)};

	/**
	 * Register default system topics
	 */
	void register_default_topics();

	/**
	 * Find free topic slot
	 */
	int find_free_slot() const;

	/**
	 * Generate topic ID from name hash
	 */
	uint16_t generate_topic_id(const char *name) const;
};

/**
 * Global topic registry instance
 */
extern TopicRegistry g_topic_registry;

} // namespace distributed_uorb
