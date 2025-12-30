#include <px4_arch/quad_encoder.h>
#include <px4_arch/board_encoder.h>
#include <board_config.h>

// Board encoder configurations
const quad_encoder_config_t g_board_encoder_configs[BOARD_NUM_QUAD_ENCODERS] = {
	{
		.gpio_a = QUAD_ENCODER_A_GPIO,
		.gpio_b = QUAD_ENCODER_B_GPIO,
		.overflow_count = 0,  // 0 = no auto-reset
	}
};

// Board encoder names
const char *g_board_encoder_names[BOARD_NUM_QUAD_ENCODERS] = {
	"Rear Wheel Motor"
};
