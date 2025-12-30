#pragma once

#include <stdint.h>
#include <stdbool.h>

// Maximum number of encoder instances supported
#define ENCODER_MAX_INSTANCES 4

// Simplified raw data structure from platform
// Optimized for memory alignment and atomic operations
typedef struct {
	uint64_t timestamp;         // High-resolution timestamp (8 bytes, 8-byte aligned)
	int64_t counter;           // Raw encoder counter value (8 bytes, 8-byte aligned)
} __attribute__((packed, aligned(8))) encoder_raw_data_t;

// Encoder hardware configuration
typedef struct {
	uint32_t gpio_a;                // GPIO pin for channel A
	uint32_t gpio_b;                // GPIO pin for channel B
	uint16_t overflow_count;        // Counter overflow/wraparound value (0 = no auto-reset)
} quad_encoder_config_t;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize all quad encoder instances
 *
 * This function initializes all encoder instances according to their configurations.
 * Must be called once during board initialization before using any other functions.
 *
 * @return 0 on success, negative error code on failure
 */
int quad_encoder_init(void);

/**
 * @brief Start a quad encoder instance (enable interrupts)
 *
 * Enables interrupts and starts data collection for the specified encoder.
 * The encoder must have been initialized with quad_encoder_init() first.
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 */
int quad_encoder_start(uint8_t encoder_id);

/**
 * @brief Stop a quad encoder instance (disable interrupts)
 *
 * Disables interrupts and stops data collection for the specified encoder.
 *
 * @param encoder_id Encoder instance ID
 * @return 0 on success, negative error code on failure
 */
int quad_encoder_stop(uint8_t encoder_id);

/**
 * @brief Get raw encoder data from platform
 *
 * Gets the latest raw encoder data (timestamp, counter, direction) from the platform.
 * This is a simplified interface where all processing is done in the module.
 *
 * @param encoder_id Encoder instance ID
 * @param raw_data Pointer to store raw encoder data
 * @return true if data was retrieved successfully, false otherwise
 */
bool quad_encoder_get_raw_data(uint8_t encoder_id, encoder_raw_data_t *raw_data);

/**
 * @brief Set encoder overflow count
 *
 * Configures when the encoder counter automatically wraps around.
 * When the counter reaches this value, it will reset to 0 (forward direction)
 * or to (overflow_count - 1) for reverse direction.
 *
 * @param encoder_id Encoder instance ID
 * @param overflow_count Counter overflow value (0 = no auto-reset)
 * @return 0 on success, negative error code on failure
 */
int quad_encoder_set_overflow_count(uint8_t encoder_id, uint16_t overflow_count);

/**
 * @brief Get encoder overflow count
 *
 * Gets the current encoder overflow count setting.
 *
 * @param encoder_id Encoder instance ID
 * @param overflow_count Pointer to store the current overflow count
 * @return 0 on success, negative error code on failure
 */
int quad_encoder_get_overflow_count(uint8_t encoder_id, uint16_t *overflow_count);

#ifdef __cplusplus
}
#endif
