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
 * @file board_hbridge_config.cpp
 *
 * Board-specific H-Bridge configuration for NXT-Dual-WL-Rear
 * This board manages left and right wheel drive
 */

#include <board_config.h>
#include <drivers/hbridge/hbridge_config.h>

#ifdef BOARD_HAS_HBRIDGE_CONFIG

// Channel configurations for rear board (left and right wheels)
hbridge_config_t hbridge_configs[HBRIDGE_MAX_INSTANCES] = {
	// Channel 0 - Left wheel control via PWM 2
	{
		.instance_id = 0,
		.name = "boom_motor",
		.enabled = true,
		.pwm_ch = 0,
		.dir_gpio = DRV8701_LEFT_DIR_GPIO,   // PE14 - Rear wheel direction
		.dir_reverse = false
	},
	// Channel 1 - Right wheel control via PWM 3
	{
		.instance_id = 1,
		.name = "rear_wheel",
		.enabled = true,
		.pwm_ch = 1,
		.dir_gpio = DRV8701_RIGHT_DIR_GPIO,  // PE13 - Boom direction
		.dir_reverse = false
	}
};

// Manager configuration for shared resources
hbridge_manager_config_t hbridge_manager_config = {
	.enable_gpio = DRV8701_ENABLE_GPIO  // PE7 - Shared enable GPIO for all channels
};

#endif // BOARD_HAS_HBRIDGE_CONFIG
