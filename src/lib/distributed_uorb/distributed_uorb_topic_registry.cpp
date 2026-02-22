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
 * @file distributed_uorb_topic_registry.cpp
 * @author PX4 Development Team
 *
 * Distributed uORB topic registry implementation
 */

#include "distributed_uorb_topic_registry.hpp"
#include <px4_platform_common/log.h>

// Include basic distributed topic headers that are likely to exist
// If specific topics are missing, they can be registered dynamically later

namespace distributed_uorb
{

// Global registry instance
TopicRegistry g_topic_registry;

bool TopicRegistry::init()
{
	_topic_count = 0;
	_next_custom_id = static_cast<uint16_t>(TopicIdRange::CUSTOM_BASE);

	register_default_topics();

	PX4_INFO("Topic registry initialized with %zu topics", _topic_count);
	return true;
}

void TopicRegistry::register_default_topics()
{
	// Default topics are registered by application code that uses the registry
	// This allows flexibility for different wheel loader configurations
	PX4_DEBUG("Default topics registration skipped - will be registered by application");
}

bool TopicRegistry::register_topic(const char *name, const orb_metadata *meta,
				   bool is_distributed, uint32_t publish_rate_hz,
				   uint8_t priority, bool requires_ack)
{
	if (_topic_count >= MAX_REGISTERED_TOPICS) {
		PX4_ERR("Topic registry full, cannot register %s", name);
		return false;
	}

	if (strlen(name) >= sizeof(TopicInfo::name)) {
		PX4_ERR("Topic name too long: %s", name);
		return false;
	}

	// Check if topic already registered
	if (get_topic_id(name) != static_cast<uint16_t>(TopicIdRange::INVALID)) {
		PX4_WARN("Topic %s already registered", name);
		return false;
	}

	int slot = find_free_slot();

	if (slot < 0) {
		PX4_ERR("No free slot for topic %s", name);
		return false;
	}

	TopicInfo &info = _topics[slot];
	info.topic_id = generate_topic_id(name);
	strncpy(info.name, name, sizeof(info.name) - 1);
	info.name[sizeof(info.name) - 1] = '\0';
	info.meta = meta;
	info.message_size = meta ? meta->o_size : 0;
	info.is_distributed = is_distributed;
	info.publish_rate_hz = publish_rate_hz;
	info.priority = priority;
	info.requires_acknowledgment = requires_ack;

	_topic_count++;

	PX4_DEBUG("Registered topic: %s (ID: 0x%04X, size: %zu)", name, info.topic_id, info.message_size);
	return true;
}

uint16_t TopicRegistry::get_topic_id(const char *name) const
{
	for (size_t i = 0; i < _topic_count; ++i) {
		if (strcmp(_topics[i].name, name) == 0) {
			return _topics[i].topic_id;
		}
	}

	return static_cast<uint16_t>(TopicIdRange::INVALID);
}

const TopicInfo *TopicRegistry::get_topic_info(uint16_t topic_id) const
{
	for (size_t i = 0; i < _topic_count; ++i) {
		if (_topics[i].topic_id == topic_id) {
			return &_topics[i];
		}
	}

	return nullptr;
}

const TopicInfo *TopicRegistry::get_topic_info(const char *name) const
{
	for (size_t i = 0; i < _topic_count; ++i) {
		if (strcmp(_topics[i].name, name) == 0) {
			return &_topics[i];
		}
	}

	return nullptr;
}

bool TopicRegistry::is_distributed_topic(uint16_t topic_id) const
{
	const TopicInfo *info = get_topic_info(topic_id);
	return info ? info->is_distributed : false;
}

size_t TopicRegistry::get_all_topics(const TopicInfo **topics, size_t max_topics) const
{
	size_t count = 0;

	for (size_t i = 0; i < _topic_count && count < max_topics; ++i) {
		topics[count++] = &_topics[i];
	}

	return count;
}

const char *TopicRegistry::get_topic_name(uint16_t topic_id) const
{
	const TopicInfo *info = get_topic_info(topic_id);
	return info ? info->name : nullptr;
}

const TopicInfo *TopicRegistry::get_topic_by_index(size_t index) const
{
	if (index < _topic_count) {
		return &_topics[index];
	}

	return nullptr;
}

void TopicRegistry::print_statistics() const
{
	PX4_INFO("Topic Registry Statistics:");
	PX4_INFO("  Total topics: %zu/%zu", _topic_count, MAX_REGISTERED_TOPICS);
	PX4_INFO("  Next custom ID: 0x%04X", _next_custom_id);

	size_t distributed_count = 0;

	for (size_t i = 0; i < _topic_count; ++i) {
		if (_topics[i].is_distributed) {
			distributed_count++;
		}
	}

	PX4_INFO("  Distributed topics: %zu", distributed_count);

	// Show first few topics as example
	PX4_INFO("  Registered topics:");
	size_t show_count = _topic_count > 10 ? 10 : _topic_count;

	for (size_t i = 0; i < show_count; ++i) {
		const TopicInfo &info = _topics[i];
		PX4_INFO("    %s (0x%04X, %zuB, %s)", info.name, info.topic_id,
			 info.message_size, info.is_distributed ? "distributed" : "local");
	}

	if (_topic_count > show_count) {
		PX4_INFO("    ... and %zu more", _topic_count - show_count);
	}
}

int TopicRegistry::find_free_slot() const
{
	for (size_t i = 0; i < MAX_REGISTERED_TOPICS; ++i) {
		if (_topics[i].topic_id == 0) { // Uninitialized slot
			return static_cast<int>(i);
		}
	}

	return -1;
}

uint16_t TopicRegistry::generate_topic_id(const char *name) const
{
	// Map known topics to predefined IDs
	struct {
		const char *name;
		TopicIdRange id;
	} predefined_topics[] = {
		{"actuator_outputs", TopicIdRange::ACTUATOR_OUTPUTS},
		{"vehicle_status", TopicIdRange::VEHICLE_STATUS},
		{"boom_control_setpoint", TopicIdRange::BOOM_CONTROL_SETPOINT},
		{"tilt_control_setpoint", TopicIdRange::TILT_CONTROL_SETPOINT},
		{"steering_setpoint", TopicIdRange::STEERING_SETPOINT},
		{"load_lamp_command", TopicIdRange::LOAD_LAMP_COMMAND},
		{"sensor_limit_switch", TopicIdRange::SENSOR_LIMIT_SWITCH},
		{"sensor_quad_encoder", TopicIdRange::SENSOR_QUAD_ENCODER},
		{"hbridge_status", TopicIdRange::HBRIDGE_STATUS},
		{"boom_status", TopicIdRange::BOOM_STATUS},
		{"bucket_status", TopicIdRange::BUCKET_STATUS},
		{"steering_status", TopicIdRange::STEERING_STATUS},
	};

	for (const auto &topic : predefined_topics) {
		if (strcmp(name, topic.name) == 0) {
			return static_cast<uint16_t>(topic.id);
		}
	}

	// For custom topics, use hash-based ID generation
	uint16_t hash = 0;

	for (const char *p = name; *p; ++p) {
		hash = (hash * 31) + *p;
	}

	// Ensure ID is in custom range
	uint16_t id = static_cast<uint16_t>(TopicIdRange::CUSTOM_BASE) + (hash & 0x0FFF);

	// Check for conflicts
	while (get_topic_info(id) != nullptr) {
		id = (id + 1) | static_cast<uint16_t>(TopicIdRange::CUSTOM_BASE);
	}

	return id;
}

} // namespace distributed_uorb
