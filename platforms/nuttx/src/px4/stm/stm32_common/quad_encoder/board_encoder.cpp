#include <px4_arch/quad_encoder.h>
#include <px4_arch/board_encoder.h>
#include <px4_platform_common/px4_config.h>
#include "board_config.h"

// Forward declarations for board-specific encoder arrays
extern const quad_encoder_config_t g_board_encoder_configs[];
extern const char *g_board_encoder_names[];

// Board encoder configuration functions (common implementation)
const quad_encoder_config_t *board_get_encoder_config(uint8_t encoder_id)
{
	if (encoder_id >= board_get_max_encoders()) {
		return nullptr;
	}
	return &g_board_encoder_configs[encoder_id];
}

uint8_t board_get_max_encoders(void)
{
	return BOARD_NUM_QUAD_ENCODERS;
}

const char *board_get_encoder_name(uint8_t encoder_id)
{
	if (encoder_id >= board_get_max_encoders()) {
		return "Unknown";
	}
	return g_board_encoder_names[encoder_id];
}
