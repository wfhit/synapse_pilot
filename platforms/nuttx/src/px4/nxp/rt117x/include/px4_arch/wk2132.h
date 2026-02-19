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

#pragma once

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <termios.h>

#include <nuttx/config.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/serial/serial.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/semaphore.h>

__BEGIN_DECLS

/* WK2132 Register Definitions - Global Registers */
#define WK2132_GENA     0x00  /* Global Enable Register */
#define WK2132_GRST     0x01  /* Global Reset Register */
#define WK2132_GMUT     0x02  /* Global Master IRQ Enable */
#define WK2132_GIER     0x10  /* Global IRQ Enable Register */
#define WK2132_GIFR     0x11  /* Global IRQ Flag Register */

/* WK2132 Register Definitions - Page Select */
#define WK2132_SPAGE    0x03  /* Sub-page Select Register - (C1,C0)0011 */

/* WK2132 Register Definitions - Sub-page 0 (SPAGE0) */
#define WK2132_SCR      0x04  /* Sub-channel Control Register - (C1,C0)0100 */
#define WK2132_LCR      0x05  /* Line Control Register - (C1,C0)0101 */
#define WK2132_FCR      0x06  /* FIFO Control Register - (C1,C0)0110 */
#define WK2132_SIER     0x07  /* Sub-channel IRQ Enable Register - (C1,C0)0111 */
#define WK2132_SIFR     0x08  /* Sub-channel IRQ Flag Register - (C1,C0)1000 */
#define WK2132_TFCNT    0x09  /* TX FIFO Count Register - (C1,C0)1001 */
#define WK2132_RFCNT    0x0A  /* RX FIFO Count Register - (C1,C0)1010 */
#define WK2132_FSR      0x0B  /* FIFO Status Register - (C1,C0)1011 */
#define WK2132_LSR      0x0C  /* Line Status Register - (C1,C0)1100 */
#define WK2132_FDAT     0x0D  /* FIFO Data Register - (C1,C0)1101 */

/* WK2132 Register Definitions - Sub-page 1 (SPAGE1) */
#define WK2132_BAUD1    0x04  /* Baud Rate Control Register 1 - (C1,C0)0100 */
#define WK2132_BAUD0    0x05  /* Baud Rate Control Register 0 - (C1,C0)0101 */
#define WK2132_PRES     0x06  /* Prescaler Register - (C1,C0)0110 */
#define WK2132_RFTL     0x07  /* RX FIFO Trigger Level Register - (C1,C0)0111 */
#define WK2132_TFTL     0x08  /* TX FIFO Trigger Level Register - (C1,C0)1000 */

/* Page Select Values */
#define WK2132_SPAGE_PAGE0  0x00  /* Select PAGE0 (UART control registers) */
#define WK2132_SPAGE_PAGE1  0x01  /* Select PAGE1 (Baud rate and FIFO trigger registers) */

/* GENA Register Bits */
#define WK2132_GENA_UT4EN   0x08  /* UART4 Enable */
#define WK2132_GENA_UT3EN   0x04  /* UART3 Enable */
#define WK2132_GENA_UT2EN   0x02  /* UART2 Enable */
#define WK2132_GENA_UT1EN   0x01  /* UART1 Enable */

/* GRST Register Bits - Global Reset */
#define WK2132_GRST_UT4SLEEP 0x80  /* UART4 Sleep Mode Control (0: Wake up, 1: Sleep) */
#define WK2132_GRST_UT3SLEEP 0x40  /* UART3 Sleep Mode Control (0: Wake up, 1: Sleep) */
#define WK2132_GRST_UT2SLEEP 0x20  /* UART2 Sleep Mode Control (0: Wake up, 1: Sleep) */
#define WK2132_GRST_UT1SLEEP 0x10  /* UART1 Sleep Mode Control (0: Wake up, 1: Sleep) */
#define WK2132_GRST_UT4RST  0x08  /* UART4 Reset */
#define WK2132_GRST_UT3RST  0x04  /* UART3 Reset */
#define WK2132_GRST_UT2RST  0x02  /* UART2 Reset */
#define WK2132_GRST_UT1RST  0x01  /* UART1 Reset */

/* GIER/GIFR Register Bits */
#define WK2132_GIFR_UT4INT   0x08  /* UART4 Interrupt Flag (0: No interrupt, 1: Has interrupt) */
#define WK2132_GIFR_UT3INT   0x04  /* UART3 Interrupt Flag (0: No interrupt, 1: Has interrupt) */
#define WK2132_GIFR_UT2INT   0x02  /* UART2 Interrupt Flag (0: No interrupt, 1: Has interrupt) */
#define WK2132_GIFR_UT1INT   0x01  /* UART1 Interrupt Flag (0: No interrupt, 1: Has interrupt) */

/* SPAGE Register Bits - Sub-page Select Register */
#define WK2132_SPAGE_PAGE    0x01  /* PAGE Select bit (0: PAGE0, 1: PAGE1) */

/* SCR Register Bits */
#define WK2132_SCR_SLEEPEN   0x04  /* Sleep Enable (0: Disable, 1: Enable) */
#define WK2132_SCR_TXEN      0x02  /* TX Enable (0: Disable, 1: Enable) */
#define WK2132_SCR_RXEN      0x01  /* RX Enable (0: Disable, 1: Enable) */

/* LCR Register Bits */
#define WK2132_LCR_BREAK    0x20  /* Break Line Output Enable (0: Normal output, 1: Line Break output) */
#define WK2132_LCR_IREN     0x10  /* IrDA SIR Enable (0: Disable, 1: Enable) */
#define WK2132_LCR_PAEN     0x08  /* Parity Enable (0: No parity, 1: Has parity) */
#define WK2132_LCR_PAM1     0x04  /* Parity Mode bit 1 */
#define WK2132_LCR_PAM0     0x02  /* Parity Mode bit 0 (00: Odd, 01: Even, 10: Mark, 11: Space) */
#define WK2132_LCR_STPL     0x01  /* Stop Bit Length (0: 1 bit, 1: 2 bits) */
#define WK2132_LCR_STB      WK2132_LCR_STPL  /* Alias for compatibility */

/* FCR Register Bits */
#define WK2132_FCR_TFTRIG1  0x80  /* TX FIFO Trigger Level bit 1 */
#define WK2132_FCR_TFTRIG0  0x40  /* TX FIFO Trigger Level bit 0 (TFTRIG[1:0]: 00=8Byte, 01=16Byte, 10=24Byte, 11=30Byte) */
#define WK2132_FCR_RFTRIG1  0x20  /* RX FIFO Trigger Level bit 1 */
#define WK2132_FCR_RFTRIG0  0x10  /* RX FIFO Trigger Level bit 0 (RFTRIG[1:0]: 00=8Byte, 01=16Byte, 10=24Byte, 11=28Byte) */
#define WK2132_FCR_TFEN     0x08  /* TX FIFO Enable (0: Disable, 1: Enable) */
#define WK2132_FCR_RFEN     0x04  /* RX FIFO Enable (0: Disable, 1: Enable) */
#define WK2132_FCR_TFRST    0x02  /* TX FIFO Reset (0: No effect, 1: Reset TX FIFO) */
#define WK2132_FCR_RFRST    0x01  /* RX FIFO Reset (0: No effect, 1: Reset RX FIFO) */

/* SIER/SIFR Register Bits */
#define WK2132_SIER_FERR    0x80  /* Frame Error Interrupt Enable */
#define WK2132_SIER_CTS     0x40  /* CTS Interrupt Enable */
#define WK2132_SIER_RTS     0x20  /* RTS Interrupt Enable */
#define WK2132_SIER_XOFF    0x10  /* XOFF Interrupt Enable */
#define WK2132_SIER_TFEMPTY 0x08  /* TX FIFO Empty Interrupt Enable */
#define WK2132_SIER_TFTRIG  0x04  /* TX FIFO Trigger Interrupt Enable */
#define WK2132_SIER_RXOVT   0x02  /* RX FIFO Overflow/Timeout Interrupt Enable */
#define WK2132_SIER_RFTRIG  0x01  /* RX FIFO Trigger Interrupt Enable */

/* Aliases for compatibility with _IEN suffix naming */
#define WK2132_SIER_FERR_IEN    WK2132_SIER_FERR
#define WK2132_SIER_CTS_IEN     WK2132_SIER_CTS
#define WK2132_SIER_RTS_IEN     WK2132_SIER_RTS
#define WK2132_SIER_XOFF_IEN    WK2132_SIER_XOFF
#define WK2132_SIER_TFEMPTY_IEN WK2132_SIER_TFEMPTY
#define WK2132_SIER_TFTRIG_IEN  WK2132_SIER_TFTRIG
#define WK2132_SIER_RXOUT_IEN   WK2132_SIER_RXOVT
#define WK2132_SIER_RFTRIG_IEN  WK2132_SIER_RFTRIG

/* FSR Register Bits */
#define WK2132_FSR_RFOE     0x80  /* RX FIFO Overflow Error (0: No OE error, 1: Has OE error) */
#define WK2132_FSR_RFBI     0x40  /* RX FIFO Break Interrupt (0: No Line Break, 1: Has Line Break) */
#define WK2132_FSR_RFFE     0x20  /* RX FIFO Frame Error (0: No FE error, 1: Has FE error) */
#define WK2132_FSR_RFPE     0x10  /* RX FIFO Parity Error (0: No PE error, 1: Has PE error) */
#define WK2132_FSR_RDAT     0x08  /* RX FIFO Data Available (0: No data in RX FIFO, 1: Has data in RX FIFO) */
#define WK2132_FSR_TDAT     0x04  /* TX FIFO Data Available (0: No data in TX FIFO, 1: Has data in TX FIFO) */
#define WK2132_FSR_TFULL    0x02  /* TX FIFO Full (0: TX FIFO not full, 1: TX FIFO full) */
#define WK2132_FSR_TBUSY    0x01  /* TX Busy (0: TX not busy, 1: TX busy) */

/* LSR Register Bits */
#define WK2132_LSR_OE       0x08  /* Overrun Error (0: No OE error, 1: Has OE error) */
#define WK2132_LSR_BI       0x04  /* Break Interrupt (0: No Line Break error, 1: Has Line Break error) */
#define WK2132_LSR_FE       0x02  /* Framing Error (0: No FE error, 1: Has FE error) */
#define WK2132_LSR_PE       0x01  /* Parity Error (0: No PE error, 1: Has PE error) */

/* RFTL Register Values (SPAGE1) - RX FIFO Trigger Levels */
#define WK2132_RFTL_8           0x00  /* RX FIFO Trigger Level 8 bytes */
#define WK2132_RFTL_16          0x01  /* RX FIFO Trigger Level 16 bytes */
#define WK2132_RFTL_24          0x02  /* RX FIFO Trigger Level 24 bytes */
#define WK2132_RFTL_30          0x03  /* RX FIFO Trigger Level 30 bytes */

/* TFTL Register Values (SPAGE1) - TX FIFO Trigger Levels */
#define WK2132_TFTL_8           0x00  /* TX FIFO Trigger Level 8 bytes */
#define WK2132_TFTL_16          0x01  /* TX FIFO Trigger Level 16 bytes */
#define WK2132_TFTL_24          0x02  /* TX FIFO Trigger Level 24 bytes */
#define WK2132_TFTL_30          0x03  /* TX FIFO Trigger Level 30 bytes */

/* Configuration */
#define WK2132_CRYSTAL_FREQ     14745600  /* 14.7456 MHz crystal */
#define WK2132_MAX_PORTS        4         /* Maximum 4 UART ports */
#define WK2132_FIFO_SIZE        256       /* FIFO size */

/* Default I2C configuration */
#define WK2132_I2C_FREQUENCY    1000000   /* 1 MHz High-Speed I2C mode for maximum performance */
#define WK2132_DEFAULT_ADDR     0x10      /* Default I2C address */

/* I2C Address bit definitions */
#define WK2132_ADDR_REG_ACCESS  0x00      /* Register access (bit 0 = 0) */
#define WK2132_ADDR_FIFO_ACCESS 0x01      /* FIFO access (bit 0 = 1) */

/* Channel selection bits (bits 2-1) */
#define WK2132_ADDR_CH1         0x00      /* Channel 1 (bits 2-1 = 00) */
#define WK2132_ADDR_CH2         0x02      /* Channel 2 (bits 2-1 = 01) */
#define WK2132_ADDR_CH3         0x04      /* Channel 3 (bits 2-1 = 10) */
#define WK2132_ADDR_CH4         0x06      /* Channel 4 (bits 2-1 = 11) */

/* Macro to construct I2C address */
#define WK2132_MAKE_I2C_ADDR(base, channel, is_fifo) \
	((base) | (((channel) - 1) << 1) | ((is_fifo) ? WK2132_ADDR_FIFO_ACCESS : WK2132_ADDR_REG_ACCESS))

/* Device instance data */
struct wk2132_dev_s {
	FAR struct i2c_master_s *i2c;     /* I2C interface */
	uint8_t                  base_addr; /* I2C base address */
	uint8_t                  port;     /* UART port number (1-4) */
	uint32_t                 baud;     /* Configured baud rate */
	uint32_t                 parity;   /* Configured parity */
	uint32_t                 nbits;    /* Number of bits */
	bool                     stopbits2; /* Two stop bits */
	sem_t                    exclsem;  /* Mutual exclusion */
	bool                     enabled;  /* Port enabled flag */
};

/* Public Functions */

/**
 * Initialize a WK2132 UART port
 *
 * @param i2c        I2C master interface
 * @param base_addr  I2C base address of WK2132 chip
 * @param port       UART port number (1-4)
 * @return           UART device structure or NULL on failure
 */
FAR struct uart_dev_s *wk2132_uart_init(FAR struct i2c_master_s *i2c,
					uint8_t base_addr, uint8_t port);

/**
 * Register WK2132 serial devices as /dev/ttyS* devices
 *
 * @param i2c_bus       I2C bus number
 * @param i2c_base_addr I2C base address of WK2132 chip
 * @param base_tty      Base TTY number (e.g., 6 for /dev/ttyS6)
 * @param num_ports     Number of ports to register (1-4)
 * @return              OK on success, negative on failure
 */
int wk2132_register_devices(int i2c_bus, uint8_t i2c_base_addr,
			    int base_tty, int num_ports);

/**
 * Board-specific WK2132 initialization
 *
 * This function initializes the WK2132 I2C-to-UART bridge chip
 * using board-specific configuration parameters.
 *
 * @return              OK on success, negative error code on failure
 */
int wk2132_board_init(void);

/**
 * Get WK2132 device path for a specific port
 *
 * @param port_num      Port number (0-3)
 * @param path          Buffer to store device path
 * @param path_len      Length of path buffer
 * @return              OK on success, negative on error
 */
int wk2132_get_device_path(int port_num, char *path, size_t path_len);

__END_DECLS
