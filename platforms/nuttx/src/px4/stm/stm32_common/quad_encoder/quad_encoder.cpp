/****************************************************************************
 *
 * Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file quad_encoder.cpp
 *
 * STM32 Simplified Quadrature Encoder Driver for NuttX/STM32 platforms.
 *
 * This driver provides a simplified quadrature encoder interface that:
 * - Provides only raw data (timestamp, counter, direction)
 * - Lets the module handle all processing
 * - Uses minimal ISRs for real-time performance
 * - Supports multiple encoder instances
 *
 * @author PX4 Development Team
 */

#include <px4_arch/quad_encoder.h>
#include <px4_arch/board_encoder.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <drivers/drv_hrt.h>

#include <nuttx/config.h>
#include <nuttx/irq.h>
#include <arch/board/board.h>
#include <chip.h>
#include <stm32_gpio.h>

#include <cstring>
#include <cerrno>
#include <syslog.h>

/**
 * @brief Simplified encoder state structure
 */
struct encoder_state_t {
	// Configuration
	quad_encoder_config_t config;
	bool is_initialized;
	bool is_active;

	// GPIO pins
	uint32_t gpio_a;
	uint32_t gpio_b;

	// Current state
	uint8_t last_ab_state;
	int64_t counter;
	uint64_t last_timestamp;

	// Raw data for module
	encoder_raw_data_t raw_data;
	volatile bool data_updated;
};

/**
 * @brief Global encoder instances
 */
static encoder_state_t g_encoders[ENCODER_MAX_INSTANCES];
static bool g_quad_encoder_initialized = false;

/**
 * @brief Forward declarations
 */
static int encoder_isr_wrapper_0(int irq, void *context, void *arg);
static int encoder_isr_wrapper_1(int irq, void *context, void *arg);
static int encoder_isr_wrapper_2(int irq, void *context, void *arg);
static int encoder_isr_wrapper_3(int irq, void *context, void *arg);

// ISR function pointers array
static int (*g_isr_wrappers[ENCODER_MAX_INSTANCES])(int, void *, void *) = {
	encoder_isr_wrapper_0,
	encoder_isr_wrapper_1,
	encoder_isr_wrapper_2,
	encoder_isr_wrapper_3
};

/**
 * @brief Generic encoder ISR processing
 */
static void process_encoder_interrupt(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return;
	}

	encoder_state_t *encoder = &g_encoders[encoder_id];

	if (!encoder->is_active) {
		return;
	}

	// Get current timestamp
	uint64_t timestamp = hrt_absolute_time();

	// Read current pin states
	bool pin_a = stm32_gpioread(encoder->gpio_a);
	bool pin_b = stm32_gpioread(encoder->gpio_b);

	// Build current AB state
	uint8_t current_ab_state = (pin_b << 1) | pin_a;

	// Quadrature decoding state machine
	int8_t direction = 0;

	switch ((encoder->last_ab_state << 2) | current_ab_state) {
	case 0x01: case 0x07: case 0x08: case 0x0E: // Forward
		direction = 1;
		break;

	case 0x02: case 0x04: case 0x0B: case 0x0D: // Reverse
		direction = -1;
		break;

	default: // No change or invalid
		break;
	}

	// Update position
	if (direction != 0) {
		encoder->counter += direction;
		encoder->last_timestamp = timestamp;

		// Handle counter wrap-around at overflow boundary
		if (encoder->config.overflow_count > 0) {
			if (encoder->counter >= encoder->config.overflow_count) {
				encoder->counter = 0;

			} else if (encoder->counter < 0) {
				encoder->counter = encoder->config.overflow_count - 1;
			}
		}

		// Update raw data for module access
		encoder->raw_data.timestamp = timestamp;
		encoder->raw_data.counter = encoder->counter;

		encoder->data_updated = true;
	}

	encoder->last_ab_state = current_ab_state;
}

/**
 * @brief ISR wrapper functions for each encoder instance
 */
static int encoder_isr_wrapper_0(int irq, void *context, void *arg)
{
	process_encoder_interrupt(0);
	return 0;
}

static int encoder_isr_wrapper_1(int irq, void *context, void *arg)
{
	process_encoder_interrupt(1);
	return 0;
}

static int encoder_isr_wrapper_2(int irq, void *context, void *arg)
{
	process_encoder_interrupt(2);
	return 0;
}

static int encoder_isr_wrapper_3(int irq, void *context, void *arg)
{
	process_encoder_interrupt(3);
	return 0;
}

/**
 * @brief Initialize all quad encoder instances
 */
int quad_encoder_init(void)
{
	if (g_quad_encoder_initialized) {
		return 0; // Already initialized
	}

	// Initialize all encoder instances
	for (uint8_t i = 0; i < ENCODER_MAX_INSTANCES; i++) {
		encoder_state_t *encoder = &g_encoders[i];
		memset(encoder, 0, sizeof(encoder_state_t));

		// Get board configuration for this encoder
		const quad_encoder_config_t *config = board_get_encoder_config(i);

		if (config != nullptr) {
			encoder->config = *config;
			encoder->gpio_a = config->gpio_a;
			encoder->gpio_b = config->gpio_b;
			encoder->is_initialized = true;

			// Initialize state
			encoder->counter = 0;
			encoder->last_ab_state = 0;
			encoder->data_updated = false;

			// Initialize raw data
			encoder->raw_data.timestamp = 0;
			encoder->raw_data.counter = 0;
		}
	}

	g_quad_encoder_initialized = true;
	return 0;
}

/**
 * @brief Start a quad encoder instance (enable interrupts)
 */
int quad_encoder_start(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -EINVAL;
	}

	if (!g_quad_encoder_initialized) {
		return -ENOTCONN;
	}

	encoder_state_t *encoder = &g_encoders[encoder_id];

	if (!encoder->is_initialized) {
		return -ENOTCONN;
	}

	if (encoder->is_active) {
		return 0; // Already active
	}

	// Configure GPIO pins as inputs with pull-up
	stm32_configgpio(encoder->gpio_a | GPIO_PULLUP);
	stm32_configgpio(encoder->gpio_b | GPIO_PULLUP);

	// Read initial pin states
	bool pin_a = stm32_gpioread(encoder->gpio_a);
	bool pin_b = stm32_gpioread(encoder->gpio_b);

	encoder->last_ab_state = (pin_b << 1) | pin_a;

	// Set up interrupts on both edges for both pins
	int ret = stm32_gpiosetevent(encoder->gpio_a, true, true, true,
				     g_isr_wrappers[encoder_id], nullptr);

	if (ret < 0) {
		return ret;
	}

	ret = stm32_gpiosetevent(encoder->gpio_b, true, true, true,
				 g_isr_wrappers[encoder_id], nullptr);

	if (ret < 0) {
		// Clean up A pin interrupt
		stm32_gpiosetevent(encoder->gpio_a, false, false, false, nullptr, nullptr);
		return ret;
	}

	encoder->is_active = true;
	return 0;
}

/**
 * @brief Stop a quad encoder instance (disable interrupts)
 */
int quad_encoder_stop(uint8_t encoder_id)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -EINVAL;
	}

	encoder_state_t *encoder = &g_encoders[encoder_id];

	if (!encoder->is_active) {
		return 0; // Already stopped
	}

	// Disable interrupts
	stm32_gpiosetevent(encoder->gpio_a, false, false, false, nullptr, nullptr);
	stm32_gpiosetevent(encoder->gpio_b, false, false, false, nullptr, nullptr);

	encoder->is_active = false;
	return 0;
}

/**
 * @brief Get raw encoder data from platform
 */
bool quad_encoder_get_raw_data(uint8_t encoder_id, encoder_raw_data_t *raw_data)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES || raw_data == nullptr) {
		return false;
	}

	encoder_state_t *encoder = &g_encoders[encoder_id];

	if (!encoder->is_active) {
		return false;
	}

	// Copy raw data in a thread-safe manner
	irqstate_t flags = enter_critical_section();
	memcpy(raw_data, &encoder->raw_data, sizeof(encoder_raw_data_t));
	bool data_valid = encoder->data_updated;
	encoder->data_updated = false; // Clear update flag
	leave_critical_section(flags);

	return data_valid;
}

/**
 * @brief Set encoder overflow count
 */
int quad_encoder_set_overflow_count(uint8_t encoder_id, uint16_t overflow_count)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES) {
		return -EINVAL;
	}

	if (!g_quad_encoder_initialized) {
		return -ENOTCONN;
	}

	encoder_state_t *encoder = &g_encoders[encoder_id];

	if (!encoder->is_initialized) {
		return -ENOTCONN;
	}

	// Update overflow count in a thread-safe manner
	irqstate_t flags = enter_critical_section();
	encoder->config.overflow_count = overflow_count;
	leave_critical_section(flags);

	return 0;
}

/**
 * @brief Get encoder overflow count
 */
int quad_encoder_get_overflow_count(uint8_t encoder_id, uint16_t *overflow_count)
{
	if (encoder_id >= ENCODER_MAX_INSTANCES || overflow_count == nullptr) {
		return -EINVAL;
	}

	if (!g_quad_encoder_initialized) {
		return -ENOTCONN;
	}

	encoder_state_t *encoder = &g_encoders[encoder_id];

	if (!encoder->is_initialized) {
		return -ENOTCONN;
	}

	// Read overflow count in a thread-safe manner
	irqstate_t flags = enter_critical_section();
	*overflow_count = encoder->config.overflow_count;
	leave_critical_section(flags);

	return 0;
}

/** @} */
