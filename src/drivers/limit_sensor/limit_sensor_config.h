#pragma once

#include <stdint.h>
#include <px4_platform_common/px4_config.h>

// Limit sensor instance configuration
struct limit_sensor_config_t {
	uint8_t instance_id;          // Instance number (0-based)
	uint8_t function;             // LimitFunction enum value
	uint32_t gpio_pin_1;          // Primary GPIO pin
	uint32_t gpio_pin_2;          // Secondary GPIO pin (0 if not used)
	bool inverted;                // True if switch logic is inverted
	bool redundancy_enabled;      // True if redundancy checking is enabled
	const char *name;             // Human-readable name
};

// Board-specific limit sensor configuration (provided by drivers_board)
__EXPORT extern const limit_sensor_config_t g_limit_sensor_config[];
