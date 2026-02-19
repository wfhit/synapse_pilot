/**
 * @file hbridge_config.h
 *
 * H-Bridge driver board configuration structure
 */

#pragma once

#include <stdint.h>

#define HBRIDGE_MAX_INSTANCES 2

/**
 * @brief Per-instance H-Bridge configuration
 * Each instance controls one H-bridge channel
 */
struct hbridge_config_t {
	uint8_t instance_id;        ///< Instance identifier (0, 1, etc.)
	const char *name;           ///< Human readable name
	bool enabled;               ///< Whether this channel is enabled
	int pwm_ch;                 ///< PWM channel number
	uint32_t dir_gpio;          ///< Direction control GPIO
};

/**
 * @brief Manager configuration for shared resources
 */
struct hbridge_manager_config_t {
	uint32_t enable_gpio;       ///< Shared enable GPIO (0 = not used)
};

// Board-specific configurations (defined in board files)
extern hbridge_config_t hbridge_configs[HBRIDGE_MAX_INSTANCES];
extern hbridge_manager_config_t hbridge_manager_config;
