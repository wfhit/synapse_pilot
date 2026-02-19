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
 * @file wk2132_board.c
 * @author PX4 Development Team
 * @brief Board-specific WK2132 initialization
 */

#include <nuttx/config.h>
#include <sys/types.h>
#include <syslog.h>
#include <stdio.h>
#include <errno.h>

#include <px4_platform_common/px4_config.h>
#include "board_config.h"

#ifdef BOARD_HAS_WK2132

#include <px4_arch/wk2132.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use configuration from board_config.h */
#define BOARD_WK2132_I2C_BUS     WK2132_I2C_BUS
#define BOARD_WK2132_I2C_BASE_ADDR    WK2132_BASE_ADDR
#define BOARD_WK2132_BASE_TTY    10      /* Start at /dev/ttyS10 */
#define BOARD_WK2132_NUM_PORTS   WK2132_NUM_PORTS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief Initialize WK2132 serial ports
 *
 * This function should be called during board initialization to set up
 * the WK2132 I2C-to-UART bridge chip and register its ports as serial devices.
 * The I2C address is automatically calculated based on J1 jumper configuration.
 *
 * @return OK on success, negative error code on failure
 */
int wk2132_board_init(void)
{
	int ret;

	syslog(LOG_INFO, "WK2132: Initializing I2C-UART bridge on bus %d", BOARD_WK2132_I2C_BUS);
	syslog(LOG_INFO, "WK2132: J1 jumper config IA1=%d, IA0=%d -> I2C addr 0x%02x",
	       WK2132_J1_IA1, WK2132_J1_IA0, BOARD_WK2132_I2C_BASE_ADDR);

	/* Register WK2132 serial devices */
	ret = wk2132_register_devices(BOARD_WK2132_I2C_BUS,
				      BOARD_WK2132_I2C_BASE_ADDR,
				      BOARD_WK2132_BASE_TTY,
				      BOARD_WK2132_NUM_PORTS);

	if (ret < 0) {
		syslog(LOG_ERR, "WK2132: Failed to initialize (%d)", ret);
		syslog(LOG_ERR, "WK2132: Check J1 jumper setting and I2C connections");
		return ret;
	}

	syslog(LOG_INFO, "WK2132: Successfully initialized %d ports starting at /dev/ttyS%d",
	       BOARD_WK2132_NUM_PORTS, BOARD_WK2132_BASE_TTY);

	return OK;
}

/**
 * @brief Get WK2132 device path for a specific port
 *
 * @param port_num  Port number (0-3)
 * @param path      Buffer to store device path
 * @param path_len  Length of path buffer
 * @return          OK on success, negative on error
 */
int wk2132_get_device_path(int port_num, char *path, size_t path_len)
{
	if (port_num < 0 || port_num >= BOARD_WK2132_NUM_PORTS) {
		return -EINVAL;
	}

	snprintf(path, path_len, "/dev/ttyS%d", BOARD_WK2132_BASE_TTY + port_num);
	return OK;
}

#endif /* BOARD_HAS_WK2132 */
