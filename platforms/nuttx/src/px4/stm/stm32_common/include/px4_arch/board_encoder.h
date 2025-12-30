#pragma once

#include <px4_arch/quad_encoder.h>

/**
 * @brief Board encoder interface
 *
 * This header defines the interface between the platform encoder hardware layer
 * and board-specific encoder configurations. Boards must provide the data arrays
 * and the platform provides the interface functions.
 */

/**
 * @brief Board-specific encoder configuration data
 *
 * These arrays must be provided by each board that supports encoders.
 * The arrays are sized using BOARD_NUM_QUAD_ENCODERS from board_config.h
 */
__EXPORT extern const quad_encoder_config_t g_board_encoder_configs[];
__EXPORT extern const char *g_board_encoder_names[];

/**
 * @brief Board interface functions
 *
 * These functions provide access to board-specific encoder configuration
 * and are implemented by the platform layer using board data.
 */

/**
 * @brief Get board encoder configuration
 *
 * @param encoder_id Encoder instance ID
 * @return Pointer to encoder configuration, or NULL if invalid
 */
const quad_encoder_config_t *board_get_encoder_config(uint8_t encoder_id);

/**
 * @brief Get maximum number of encoders supported by board
 *
 * @return Number of encoders available on this board
 */
uint8_t board_get_max_encoders(void);

/**
 * @brief Get encoder name for debugging/logging
 *
 * @param encoder_id Encoder instance ID
 * @return Encoder name string, or "Unknown" if invalid
 */
const char *board_get_encoder_name(uint8_t encoder_id);
