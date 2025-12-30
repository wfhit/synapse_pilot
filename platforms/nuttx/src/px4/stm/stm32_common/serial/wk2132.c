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
 * @file wk2132.c
 * @author PX4 Development Team
 * @brief WK2132 I2C to Quad UART serial driver for NuttX
 *
 * This driver provides serial port functionality for the WK2132 I2C to
 * Quad UART bridge chip, exposing up to 4 additional UART ports as
 * standard /dev/ttyS* devices.
 *
 * FIFO Operations (Based on I2C Timing Diagrams 11.2.3 and 11.2.4):
 * - FIFO write operations send data directly without register address prefix
 * - FIFO read operations read data directly without register address prefix
 * - Multi-byte FIFO operations are supported for improved efficiency
 * - Buffer size limitations are respected using TFCNT/RFCNT registers
 * - Maximum FIFO size is 256 bytes per channel
 */

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <termios.h>
#include <stdio.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/serial/serial.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/wqueue.h>

#include <arch/board/board.h>

/* Include appropriate STM32 I2C header based on variant */
#if defined(CONFIG_ARCH_CHIP_STM32H7)
#include <stm32_i2c.h>
#elif defined(CONFIG_ARCH_CHIP_STM32F7) || defined(CONFIG_ARCH_CHIP_STM32F4)
#include <stm32_i2c.h>
#elif defined(CONFIG_ARCH_CHIP_STM32F3) || defined(CONFIG_ARCH_CHIP_STM32F1)
#include <stm32_i2c.h>
#else
#error "WK2132: Unsupported STM32 variant"
#endif

#include <px4_arch/wk2132.h>

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int  wk2132_setup(FAR struct uart_dev_s *dev);
static void wk2132_shutdown(FAR struct uart_dev_s *dev);
static int  wk2132_attach(FAR struct uart_dev_s *dev);
static void wk2132_detach(FAR struct uart_dev_s *dev);
static int  wk2132_ioctl(FAR struct file *filep, int cmd, unsigned long arg);
static int  wk2132_receive(FAR struct uart_dev_s *dev, unsigned int *status);
static void wk2132_rxint(FAR struct uart_dev_s *dev, bool enable);
static bool wk2132_rxavailable(FAR struct uart_dev_s *dev);
static void wk2132_send(FAR struct uart_dev_s *dev, int ch);
static void wk2132_txint(FAR struct uart_dev_s *dev, bool enable);
static bool wk2132_txready(FAR struct uart_dev_s *dev);
static bool wk2132_txempty(FAR struct uart_dev_s *dev);

static int  wk2132_i2c_write_global(FAR struct wk2132_dev_s *priv, uint8_t reg,
                                     uint8_t value);
static int  wk2132_i2c_read_global(FAR struct wk2132_dev_s *priv, uint8_t reg,
                                    FAR uint8_t *value);
static int  wk2132_i2c_write_reg(FAR struct wk2132_dev_s *priv, uint8_t reg,
                                  uint8_t value);
static int  wk2132_i2c_read_reg(FAR struct wk2132_dev_s *priv, uint8_t reg,
                                 FAR uint8_t *value);

static int  wk2132_i2c_write_fifo(FAR struct wk2132_dev_s *priv, uint8_t value);
static int  wk2132_i2c_read_fifo(FAR struct wk2132_dev_s *priv, FAR uint8_t *value);

static int  wk2132_set_baud(FAR struct wk2132_dev_s *priv, uint32_t baud);
static int  wk2132_reconfigure(FAR struct uart_dev_s *dev);
static void wk2132_poll_worker(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* UART operations */
static const struct uart_ops_s g_wk2132_uart_ops =
{
  .setup          = wk2132_setup,
  .shutdown       = wk2132_shutdown,
  .attach         = wk2132_attach,
  .detach         = wk2132_detach,
  .ioctl          = wk2132_ioctl,
  .receive        = wk2132_receive,
  .rxint          = wk2132_rxint,
  .rxavailable    = wk2132_rxavailable,
  .send           = wk2132_send,
  .txint          = wk2132_txint,
  .txready        = wk2132_txready,
  .txempty        = wk2132_txempty,
};

/* Work queue for polling */
static struct work_s g_wk2132_poll_work;
static bool g_wk2132_poll_started = false;
static FAR struct uart_dev_s *g_wk2132_devices[WK2132_MAX_PORTS];
static int g_wk2132_device_count = 0;
static sem_t g_wk2132_poll_sem = SEM_INITIALIZER(1); /* Protect polling state */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/**
 * @brief Write to a global register via I2C
 *
 * Use this function for global registers that are not channel-specific:
 * - WK2132_GENA (Global Enable Register)
 * - WK2132_GRST (Global Reset Register)
 * - WK2132_GMUT (Global Master IRQ Enable)
 * - WK2132_GIER (Global IRQ Enable Register)
 * - WK2132_GIFR (Global IRQ Flag Register)
 */
static int wk2132_i2c_write_global(FAR struct wk2132_dev_s *priv, uint8_t reg,
                                    uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  uint8_t i2c_addr;
  int ret;

  /* Construct I2C address for global register:
   * Extract C1,C0 bits from register address (bits 5-4) and combine with base_addr */
  uint8_t c1c0_bits = (reg >> 4) & 0x03;  /* Extract C1,C0 from register */
  i2c_addr = priv->base_addr | (c1c0_bits << 1) | WK2132_ADDR_REG_ACCESS;

  /* Setup data buffer */
  buffer[0] = reg & 0x0F;  /* Use lower 4 bits as actual register address */
  buffer[1] = value;

  /* Setup I2C message */
  msg.frequency = WK2132_I2C_FREQUENCY;
  msg.addr      = i2c_addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  /* Perform the transfer */
  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Global write failed - reg=0x%02x, ret=%d\n", reg, ret);
    }
  return (ret >= 0) ? OK : ret;
}

/**
 * @brief Read from a global register via I2C
 *
 * Use this function for global registers that are not channel-specific:
 * - WK2132_GENA (Global Enable Register)
 * - WK2132_GRST (Global Reset Register)
 * - WK2132_GMUT (Global Master IRQ Enable)
 * - WK2132_GIER (Global IRQ Enable Register)
 * - WK2132_GIFR (Global IRQ Flag Register)
 */
static int wk2132_i2c_read_global(FAR struct wk2132_dev_s *priv, uint8_t reg,
                                   FAR uint8_t *value)
{
  struct i2c_msg_s msgs[2];
  uint8_t i2c_addr;
  int ret;

  /* Construct I2C address for global register:
   * Extract C1,C0 bits from register address (bits 5-4) and combine with base_addr */
  uint8_t c1c0_bits = (reg >> 4) & 0x03;  /* Extract C1,C0 from register */
  uint8_t reg_addr = reg & 0x0F;          /* Use lower 4 bits as actual register address */
  i2c_addr = priv->base_addr | (c1c0_bits << 1) | WK2132_ADDR_REG_ACCESS;

  /* Setup I2C write message for register address */
  msgs[0].frequency = WK2132_I2C_FREQUENCY;
  msgs[0].addr      = i2c_addr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = &reg_addr;
  msgs[0].length    = 1;

  /* Setup I2C read message for data */
  msgs[1].frequency = WK2132_I2C_FREQUENCY;
  msgs[1].addr      = i2c_addr;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = value;
  msgs[1].length    = 1;

  /* Perform the transfer */
  ret = I2C_TRANSFER(priv->i2c, msgs, 2);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Global read failed - reg=0x%02x, ret=%d\n", reg, ret);
    }
  return (ret >= 0) ? OK : ret;
}

/**
 * @brief Write a channel-specific register via I2C
 *
 * Use this function for channel-specific registers:
 * - WK2132_SPAGE (Sub-page Select Register)
 * - WK2132_SCR (Sub-channel Control Register)
 * - WK2132_LCR (Line Control Register)
 * - WK2132_FCR (FIFO Control Register)
 * - WK2132_SIER (Sub-channel IRQ Enable Register)
 * - WK2132_SIFR (Sub-channel IRQ Flag Register)
 * - WK2132_FSR (FIFO Status Register)
 * - WK2132_LSR (Line Status Register)
 * - And baud rate registers (BAUD1, BAUD0, PRES) in sub-page 1
 */
static int wk2132_i2c_write_reg(FAR struct wk2132_dev_s *priv, uint8_t reg,
                                 uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t buffer[2];
  uint8_t i2c_addr;
  int ret;

  /* Construct I2C address: base_addr in bits 6-3, channel in bits 2-1, reg access in bit 0 */
  i2c_addr = WK2132_MAKE_I2C_ADDR(priv->base_addr, priv->port, false);

  /* Setup data buffer */
  buffer[0] = reg;
  buffer[1] = value;

  /* Setup I2C message */
  msg.frequency = WK2132_I2C_FREQUENCY;
  msg.addr      = i2c_addr;
  msg.flags     = 0;
  msg.buffer    = buffer;
  msg.length    = 2;

  /* Perform the transfer */
  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/**
 * @brief Read a channel-specific register via I2C
 *
 * Use this function for channel-specific registers:
 * - WK2132_SPAGE (Sub-page Select Register)
 * - WK2132_SCR (Sub-channel Control Register)
 * - WK2132_LCR (Line Control Register)
 * - WK2132_FCR (FIFO Control Register)
 * - WK2132_SIER (Sub-channel IRQ Enable Register)
 * - WK2132_SIFR (Sub-channel IRQ Flag Register)
 * - WK2132_FSR (FIFO Status Register)
 * - WK2132_LSR (Line Status Register)
 * - And baud rate registers (BAUD1, BAUD0, PRES) in sub-page 1
 */
static int wk2132_i2c_read_reg(FAR struct wk2132_dev_s *priv, uint8_t reg,
                                FAR uint8_t *value)
{
  struct i2c_msg_s msgs[2];
  uint8_t i2c_addr;
  int ret;

  /* Construct I2C address: base_addr in bits 6-3, channel in bits 2-1, reg access in bit 0 */
  i2c_addr = WK2132_MAKE_I2C_ADDR(priv->base_addr, priv->port, false);

  /* Setup I2C write message for register address */
  msgs[0].frequency = WK2132_I2C_FREQUENCY;
  msgs[0].addr      = i2c_addr;
  msgs[0].flags     = 0;
  msgs[0].buffer    = &reg;
  msgs[0].length    = 1;

  /* Setup I2C read message for data */
  msgs[1].frequency = WK2132_I2C_FREQUENCY;
  msgs[1].addr      = i2c_addr;
  msgs[1].flags     = I2C_M_READ;
  msgs[1].buffer    = value;
  msgs[1].length    = 1;

  /* Perform the transfer */
  ret = I2C_TRANSFER(priv->i2c, msgs, 2);
  return (ret >= 0) ? OK : ret;
}

/**
 * @brief Write to FIFO via I2C (single byte)
 *
 * As shown in timing diagram 11.2.3, FIFO write operations send data directly
 * to the FIFO without specifying a register address.
 */
static int wk2132_i2c_write_fifo(FAR struct wk2132_dev_s *priv, uint8_t value)
{
  struct i2c_msg_s msg;
  uint8_t i2c_addr;
  int ret;

  /* Construct I2C address: base_addr in bits 6-3, channel in bits 2-1, FIFO access in bit 0 */
  i2c_addr = WK2132_MAKE_I2C_ADDR(priv->base_addr, priv->port, true);

  /* Setup I2C message - direct data write to FIFO as per timing diagram */
  msg.frequency = WK2132_I2C_FREQUENCY;
  msg.addr      = i2c_addr;
  msg.flags     = 0;
  msg.buffer    = &value;
  msg.length    = 1;

  /* Perform the transfer */
  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/**
 * @brief Read from FIFO via I2C (single byte)
 *
 * As shown in timing diagram 11.2.4, FIFO read operations read data directly
 * from the FIFO without specifying a register address.
 */
static int wk2132_i2c_read_fifo(FAR struct wk2132_dev_s *priv, FAR uint8_t *value)
{
  struct i2c_msg_s msg;
  uint8_t i2c_addr;
  int ret;

  /* Construct I2C address: base_addr in bits 6-3, channel in bits 2-1, FIFO access in bit 0 */
  i2c_addr = WK2132_MAKE_I2C_ADDR(priv->base_addr, priv->port, true);

  /* Setup I2C read message - direct data read from FIFO as per timing diagram */
  msg.frequency = WK2132_I2C_FREQUENCY;
  msg.addr      = i2c_addr;
  msg.flags     = I2C_M_READ;
  msg.buffer    = value;
  msg.length    = 1;

  /* Perform the transfer */
  ret = I2C_TRANSFER(priv->i2c, &msg, 1);
  return (ret >= 0) ? OK : ret;
}

/**
 * @brief Set baud rate for a UART port
 * Based on DFRobot implementation with fractional calculation
 */
static int wk2132_set_baud(FAR struct wk2132_dev_s *priv, uint32_t baud)
{
  uint16_t val_integer, val_decimal;
  uint8_t baud1, baud0, pres;
  int ret;

  syslog(LOG_DEBUG, "WK2132: Setting baud rate to %lu\n", (unsigned long)baud);

  /* DFRobot algorithm:
   * val_integer = FOSC/(baud * 16) - 1
   * val_decimal = (FOSC%(baud * 16))/(baud * 16) * 10 (for prescaler calculation)
   * Prescaler is calculated from decimal part
   */
  val_integer = (WK2132_CRYSTAL_FREQ / (baud * 16)) - 1;
  /* Calculate decimal part more accurately by scaling */
  uint32_t remainder = WK2132_CRYSTAL_FREQ % (baud * 16);
  val_decimal = (remainder * 10) / (baud * 16);

  /* Extract high and low bytes */
  baud1 = (uint8_t)(val_integer >> 8);
  baud0 = (uint8_t)(val_integer & 0x00FF);

  /* Calculate prescaler from decimal part */
  while (val_decimal > 0x0A)
    {
      val_decimal /= 0x0A;
    }
  pres = (uint8_t)val_decimal;

  /* Select sub-page 1 for baud rate configuration */
  ret = wk2132_i2c_write_reg(priv, WK2132_SPAGE, 1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to switch to sub-page 1 for port %d: %d\n", priv->port, ret);
      return ret;
    }

  /* Set baud rate registers */
  ret = wk2132_i2c_write_reg(priv, WK2132_BAUD1, baud1);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to write BAUD1 register for port %d: %d\n", priv->port, ret);
      return ret;
    }

  ret = wk2132_i2c_write_reg(priv, WK2132_BAUD0, baud0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to write BAUD0 register for port %d: %d\n", priv->port, ret);
      return ret;
    }

  ret = wk2132_i2c_write_reg(priv, WK2132_PRES, pres);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to write PRES register for port %d: %d\n", priv->port, ret);
      return ret;
    }

  /* Read back the baud rate registers for verification */
  uint8_t verify_reg;
  /* Simple verification - just check if we can read back any register to ensure I2C communication is working */
  ret = wk2132_i2c_read_reg(priv, WK2132_BAUD1, &verify_reg);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to verify baud rate registers for port %d: %d\n", priv->port, ret);
      goto errout;
    }

  /* Return to sub-page 0 */
  ret = wk2132_i2c_write_reg(priv, WK2132_SPAGE, 0);
  if (ret < 0)
    {
      return ret;
    }

  syslog(LOG_DEBUG, "WK2132: Baud rate %lu set successfully\n", (unsigned long)baud);

  return OK;

errout:
  /* Try to restore state on error */
  wk2132_i2c_write_reg(priv, WK2132_SPAGE, 0);  /* Return to page 0 */
  return ret;
}

/**
 * @brief Reconfigure UART port settings without disrupting polling
 * This function safely updates port settings while preserving enabled state
 */
static int wk2132_reconfigure(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t lcr = 0;
  int ret;
  bool was_enabled;

  syslog(LOG_INFO, "WK2132: Reconfiguring port %d\n", priv->port);

  /* Take exclusive access */
  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to get exclusive access for port %d\n", priv->port);
      return ret;
    }

  /* Save and temporarily disable port to prevent data loss during reconfiguration */
  was_enabled = priv->enabled;
  priv->enabled = false;

  /* Configure LCR based on new settings */
  if (priv->stopbits2)
    {
      lcr |= WK2132_LCR_STB;  /* 2 stop bits */
    }

  /* Configure parity according to datasheet and termios standards */
  switch (priv->parity)
    {
    case 0:  /* No parity */
      /* PAEN=0, no parity bits needed */
      break;
    case 1:  /* Odd parity */
      lcr |= WK2132_LCR_PAEN | WK2132_LCR_PAM0;  /* Enable parity, PAM1=0,PAM0=1 for odd */
      break;
    case 2:  /* Even parity */
      lcr |= WK2132_LCR_PAEN | WK2132_LCR_PAM1;  /* Enable parity, PAM1=1,PAM0=0 for even */
      break;
    default:
      syslog(LOG_WARNING, "WK2132: Invalid parity mode %lu, using no parity\n", (unsigned long)priv->parity);
      break;
    }

  ret = wk2132_i2c_write_reg(priv, WK2132_LCR, lcr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to write LCR register for port %d\n", priv->port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Set LCR=0x%02x for port %d\n", lcr, priv->port);

  /* Set baud rate */
  ret = wk2132_set_baud(priv, priv->baud);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to set baud rate %lu for port %d\n", (unsigned long)priv->baud, priv->port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Set baud rate %lu for port %d\n", (unsigned long)priv->baud, priv->port);

  /* Allow configuration to settle */
  usleep(10000);  /* 10ms delay */

  syslog(LOG_INFO, "WK2132: Reconfiguration completed successfully for port %d\n", priv->port);

  /* Restore enabled state */
  priv->enabled = was_enabled;

errout:
  /* Restore enabled state on error too */
  if (ret < 0)
    {
      priv->enabled = was_enabled;
    }
  nxsem_post(&priv->exclsem);
  return ret;
}

/**
 * @brief Polling worker function
 */
static void wk2132_poll_worker(FAR void *arg)
{
  int i;
  bool should_continue;

  /* Poll all registered devices */
  for (i = 0; i < g_wk2132_device_count; i++)
    {
      FAR struct uart_dev_s *dev = g_wk2132_devices[i];
      if (dev != NULL)
        {
          FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;

          /* Only poll enabled devices */
          if (priv->enabled)
            {
              /* Check for received data */
              if (wk2132_rxavailable(dev))
                {
                  uart_recvchars(dev);
                }

              /* Check for transmit ready */
              if (dev->xmit.head != dev->xmit.tail && wk2132_txready(dev))
                {
                  uart_xmitchars(dev);
                }
            }
        }
    }

  /* Check if we should continue polling (with proper synchronization) */
  nxsem_wait(&g_wk2132_poll_sem);
  should_continue = g_wk2132_poll_started;
  nxsem_post(&g_wk2132_poll_sem);

  /* Schedule next poll - reduced frequency to lower I2C burden */
  if (should_continue)
    {
      work_queue(LPWORK, &g_wk2132_poll_work, wk2132_poll_worker, NULL,
                 MSEC2TICK(5));  /* Poll every 5ms instead of 1ms to reduce I2C burden */
    }
}

/**
 * @brief Setup UART port
 * Based on DFRobot subSerialConfig implementation
 */
static int wk2132_setup(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t lcr = 0;
  uint8_t gena, grst;
  int ret;

  syslog(LOG_INFO, "WK2132: Setting up port %d\n", priv->port);

  /* Take exclusive access */
  ret = nxsem_wait(&priv->exclsem);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to get exclusive access for port %d\n", priv->port);
      return ret;
    }

  /* Step 1: Sub UART clock enable (DFRobot: subSerialGlobalRegEnable(subUartChannel, clock)) */
  syslog(LOG_DEBUG, "WK2132: Sub UART clock enable for port %d\n", priv->port);
  ret = wk2132_i2c_read_global(priv, WK2132_GENA, &gena);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to read GENA register for port %d\n", priv->port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Current GENA=0x%02x for port %d\n", gena, priv->port);

  /* Enable the UART port in global register */
  uint8_t gena_bit = 1 << (priv->port - 1);
  gena |= gena_bit;
  ret = wk2132_i2c_write_global(priv, WK2132_GENA, gena);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to write GENA register for port %d\n", priv->port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Enabled clock for port %d in GENA, new value=0x%02x\n", priv->port, gena);

  /* Allow clock to stabilize */
  usleep(50000);  /* 50ms delay */

  /* Step 2: Software reset sub UART (DFRobot: subSerialGlobalRegEnable(subUartChannel, rst)) */
  syslog(LOG_DEBUG, "WK2132: Software reset sub UART for port %d\n", priv->port);
  ret = wk2132_i2c_read_global(priv, WK2132_GRST, &grst);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to read GRST register for port %d\n", priv->port);
      goto errout;
    }

  uint8_t grst_bit = 1 << (priv->port - 1);
  grst |= grst_bit;
  ret = wk2132_i2c_write_global(priv, WK2132_GRST, grst);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to write GRST register for port %d\n", priv->port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Reset completed for port %d, GRST=0x%02x\n", priv->port, grst);

  /* Allow reset to complete */
  usleep(100000);  /* 100ms delay after reset */

//  /* Step 3: Sub UART global interrupt enable (DFRobot: subSerialGlobalRegEnable(subUartChannel, intrpt)) */
//  syslog(LOG_DEBUG, "WK2132: Sub UART global interrupt enable for port %d\n", priv->port);
//  ret = wk2132_i2c_read_global(priv, WK2132_GIER, &gier);
//  if (ret < 0)
//    {
//      syslog(LOG_ERR, "WK2132: Failed to read GIER register for port %d\n", priv->port);
//      goto errout;
//    }
//
//  uint8_t gier_bit = 1 << (priv->port - 1);
//  gier |= gier_bit;
//  ret = wk2132_i2c_write_global(priv, WK2132_GIER, gier);
//  if (ret < 0)
//    {
//      syslog(LOG_ERR, "WK2132: Failed to write GIER register for port %d\n", priv->port);
//      goto errout;
//    }
//
//  syslog(LOG_DEBUG, "WK2132: Global interrupt enabled for port %d, GIER=0x%02x\n", priv->port, gier);
//
//  /* Allow interrupt configuration to settle */
//  usleep(25000);  /* 25ms delay */

  /* Step 4: Sub UART page register setting (default PAGE0) */
  syslog(LOG_DEBUG, "WK2132: Sub UART page register setting (default PAGE0) for port %d\n", priv->port);
  ret = wk2132_i2c_write_reg(priv, WK2132_SPAGE, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to set page 0 for port %d\n", priv->port);
      goto errout;
    }

//  /* Step 5: Sub interrupt setting - Enable multiple interrupt sources like DFRobot */
//  syslog(LOG_DEBUG, "WK2132: Sub interrupt setting for port %d\n", priv->port);
//  uint8_t sier = WK2132_SIER_RFTRIG_IEN |   /* RX FIFO trigger interrupt */
//                 WK2132_SIER_RXOUT_IEN |    /* RX timeout interrupt */
//                 WK2132_SIER_TFTRIG_IEN |   /* TX FIFO trigger interrupt */
//                 WK2132_SIER_TFEMPTY_IEN |  /* TX FIFO empty interrupt */
//                 WK2132_SIER_FERR_IEN;      /* Frame error interrupt */
//
//  ret = wk2132_i2c_write_reg(priv, WK2132_SIER, sier);
//  if (ret < 0)
//    {
//      syslog(LOG_ERR, "WK2132: Failed to write SIER register for port %d\n", priv->port);
//      goto errout;
//    }
//
//  syslog(LOG_DEBUG, "WK2132: Set SIER=0x%02x for port %d\n", sier, priv->port);

  /* Step 6: Enable transmit/receive FIFO with reset (DFRobot style) */
  syslog(LOG_DEBUG, "WK2132: Enable transmit/receive FIFO for port %d\n", priv->port);

  /* DFRobot FCR configuration: {.rfRst = 0x01, .tfRst = 0x00, .rfEn = 0x01, .tfEn = 0x01, .rfTrig = 0x00, .tfTrig = 0x00} */
  uint8_t fcr_current = 0;

  /* Read current FCR value first (DFRobot subSerialRegConfig pattern) */
  ret = wk2132_i2c_read_reg(priv, WK2132_FCR, &fcr_current);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to read FCR register for port %d\n", priv->port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Current FCR=0x%02x for port %d\n", fcr_current, priv->port);

  /* Apply DFRobot FCR settings using OR operation like subSerialRegConfig */
  uint8_t fcr_settings = WK2132_FCR_RFRST |     /* Reset RX FIFO (rfRst = 1) */
                         WK2132_FCR_TFRST |     /* Reset TX FIFO (tfRst = 1) */
                         WK2132_FCR_RFEN |      /* Enable RX FIFO (rfEn = 1) */
                         WK2132_FCR_TFEN;       /* Enable TX FIFO (tfEn = 1) */
                         /* Trigger levels default to 0 (rfTrig = 0, tfTrig = 0) */

  ret = wk2132_i2c_write_reg(priv, WK2132_FCR, fcr_settings);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to write FCR register for port %d\n", priv->port);
      goto errout;
    }

  /* Read back for verification */
  uint8_t fcr_verify = 0;
  ret = wk2132_i2c_read_reg(priv, WK2132_FCR, &fcr_verify);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to verify FCR register for port %d\n", priv->port);
      goto errout;
    }
  syslog(LOG_DEBUG, "WK2132: Set FCR=0x%02x (was 0x%02x) for port %d\n",
         fcr_verify, fcr_current, priv->port);

  /* Allow FIFO configuration to settle */
  usleep(25000);  /* 25ms delay */

  /* Step 7: Sub UART LCR configuration */
  /* Configure LCR based on settings according to datasheet */
  /* Note: WK2132 supports only 8-bit data length (fixed) */
  /* LCR Register bits from datasheet image:
   * Bit 6: Reserved (0)
   * Bit 5: BREAK - Line Break output enable
   * Bit 4: IREN - IR mode enable
   * Bit 3: PAEN - Parity enable
   * Bit 2-1: PAM1-0 - Parity mode (10=Even, 01=Odd, 00=Reserved, 11=Mark/Space)
   * Bit 0: STB - Stop bits (0=1bit, 1=2bits)
   */

  /* Configure stop bits */
  if (priv->stopbits2)
    {
      lcr |= WK2132_LCR_STB;  /* 2 stop bits */
    }

  /* Configure parity according to datasheet and termios standards */
  switch (priv->parity)
    {
    case 0:  /* No parity */
      /* PAEN=0, no parity bits needed */
      break;
    case 1:  /* Odd parity */
      lcr |= WK2132_LCR_PAEN | WK2132_LCR_PAM0;  /* Enable parity, PAM1=0,PAM0=1 for odd */
      break;
    case 2:  /* Even parity */
      lcr |= WK2132_LCR_PAEN | WK2132_LCR_PAM1;  /* Enable parity, PAM1=1,PAM0=0 for even */
      break;
    default:
      syslog(LOG_WARNING, "WK2132: Invalid parity mode %lu, using no parity\n", (unsigned long)priv->parity);
      break;
    }

  ret = wk2132_i2c_write_reg(priv, WK2132_LCR, lcr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to write LCR register for port %d\n", priv->port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Set LCR=0x%02x for port %d\n", lcr, priv->port);

  /* Step 8: Sub UART baud rate configuration */
  /* Set baud rate */
  ret = wk2132_set_baud(priv, priv->baud);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to set baud rate %lu for port %d\n", (unsigned long)priv->baud, priv->port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Set baud rate %lu for port %d\n", (unsigned long)priv->baud, priv->port);

  /* Allow baud rate configuration to settle */
  usleep(50000);  /* 50ms delay */

  /* Step 9: Sub UART receive/transmit enable (DFRobot: sScrReg_t scr = {.rxEn = 0x01, .txEn = 0x01, .sleepEn = 0x00, .rsv = 0x00}) */
  syslog(LOG_DEBUG, "WK2132: Sub UART receive/transmit enable for port %d\n", priv->port);
  uint8_t scr = WK2132_SCR_RXEN | WK2132_SCR_TXEN;  /* Enable RX and TX, disable sleep mode */

  ret = wk2132_i2c_write_reg(priv, WK2132_SCR, scr);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to write SCR register for port %d\n", priv->port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Set SCR=0x%02x for port %d (RX/TX enabled)\n", scr, priv->port);

  /* Allow final configuration to settle */
  usleep(50000);  /* 50ms delay */

  syslog(LOG_INFO, "WK2132: Setup completed successfully for port %d\n", priv->port);

  /* Step 10: Enable polling for this port */
  /* Start polling if not already started (with proper synchronization) */
  nxsem_wait(&g_wk2132_poll_sem);
  if (!g_wk2132_poll_started)
    {
      g_wk2132_poll_started = true;
      ret = work_queue(LPWORK, &g_wk2132_poll_work, wk2132_poll_worker, NULL, 0);
      if (ret < 0)
        {
          syslog(LOG_ERR, "WK2132: Failed to start polling worker: %d\n", ret);
          g_wk2132_poll_started = false;  /* Reset on failure */
          nxsem_post(&g_wk2132_poll_sem);
          goto errout;
        }
      syslog(LOG_INFO, "WK2132: Started global polling\n");
    }
  nxsem_post(&g_wk2132_poll_sem);

  /* Mark device as enabled only after everything succeeds */
  priv->enabled = true;

errout:
  nxsem_post(&priv->exclsem);
  return ret;
}

/**
 * @brief Shutdown UART port
 */
static void wk2132_shutdown(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t gena;
  int i;
  bool any_enabled = false;

  /* Mark device as disabled first */
  priv->enabled = false;

  /* Disable all interrupts */
  wk2132_i2c_write_reg(priv, WK2132_SIER, 0x00);

  /* Disable the UART port in global register */
  if (wk2132_i2c_read_global(priv, WK2132_GENA, &gena) >= 0)
    {
      uint8_t gena_bit = 1 << (priv->port - 1);
      gena &= ~gena_bit;
      wk2132_i2c_write_global(priv, WK2132_GENA, gena);
    }

  /* Check if any devices are still enabled */
  nxsem_wait(&g_wk2132_poll_sem);
  for (i = 0; i < g_wk2132_device_count; i++)
    {
      if (g_wk2132_devices[i] != NULL)
        {
          FAR struct wk2132_dev_s *check_priv =
            (FAR struct wk2132_dev_s *)g_wk2132_devices[i]->priv;
          if (check_priv->enabled)
            {
              any_enabled = true;
              break;
            }
        }
    }

  /* If no devices are enabled, stop polling */
  if (!any_enabled && g_wk2132_poll_started)
    {
      g_wk2132_poll_started = false;
      work_cancel(LPWORK, &g_wk2132_poll_work);
      syslog(LOG_INFO, "WK2132: Stopped global polling - no devices enabled\n");
    }
  nxsem_post(&g_wk2132_poll_sem);
}

/**
 * @brief Attach interrupt (not used in polled mode)
 */
static int wk2132_attach(FAR struct uart_dev_s *dev)
{
  return OK;
}

/**
 * @brief Detach interrupt (not used in polled mode)
 */
static void wk2132_detach(FAR struct uart_dev_s *dev)
{
  /* Nothing to do */
}

/**
 * @brief IOCTL handler
 */
static int wk2132_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct inode *inode = filep->f_inode;
  FAR struct uart_dev_s *dev = inode->i_private;
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  int ret = OK;

  switch (cmd)
    {
    case TCGETS:
      {
        FAR struct termios *termiosp = (FAR struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Return current settings */
        termiosp->c_cflag = 0;

        /* WK2132 only supports 8-bit data according to datasheet */
        termiosp->c_cflag |= CS8;

        if (priv->stopbits2)
          {
            termiosp->c_cflag |= CSTOPB;
          }

        if (priv->parity == 1)
          {
            termiosp->c_cflag |= PARENB | PARODD;
          }
        else if (priv->parity == 2)
          {
            termiosp->c_cflag |= PARENB;
          }

        /* Set baud rate */
        cfsetispeed(termiosp, priv->baud);
        cfsetospeed(termiosp, priv->baud);
      }
      break;

    case TCSETS:
      {
        FAR struct termios *termiosp = (FAR struct termios *)arg;

        if (!termiosp)
          {
            ret = -EINVAL;
            break;
          }

        /* Extract new settings */
        priv->baud = cfgetispeed(termiosp);

        /* WK2132 only supports 8-bit data according to datasheet */
        priv->nbits = 8;

        priv->stopbits2 = (termiosp->c_cflag & CSTOPB) != 0;

        if (termiosp->c_cflag & PARENB)
          {
            priv->parity = (termiosp->c_cflag & PARODD) ? 1 : 2;
          }
        else
          {
            priv->parity = 0;
          }

        /* Reconfigure the UART safely without disrupting polling */
        ret = wk2132_reconfigure(dev);
        if (ret < 0)
          {
            syslog(LOG_ERR, "WK2132: Failed to reconfigure UART port %d: %d\n", priv->port, ret);
            /* On reconfigure failure, we should still return the error but the device remains usable with old settings */
          }
      }
      break;

    default:
      ret = -ENOTTY;
      break;
    }

  return ret;
}

/**
 * @brief Receive character
 */
static int wk2132_receive(FAR struct uart_dev_s *dev, unsigned int *status)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t fsr;
  uint8_t lsr;
  uint8_t rxdata;
  int ret;

  /* Check if device is enabled */
  if (!priv->enabled)
    {
      *status = 0;
      return -EIO;
    }

  /* Get FIFO status */
  ret = wk2132_i2c_read_reg(priv, WK2132_FSR, &fsr);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WK2132: Failed to read FSR for port %d: %d\n", priv->port, ret);
      *status = 0;
      return -EIO;
    }

  /* Check if data is actually available before reading */
  if ((fsr & WK2132_FSR_RDAT) == 0)
    {
      *status = fsr << 8;
      return -EAGAIN;  /* No data available */
    }

  /* Get line status */
  ret = wk2132_i2c_read_reg(priv, WK2132_LSR, &lsr);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WK2132: Failed to read LSR for port %d: %d\n", priv->port, ret);
      *status = fsr << 8;
      return -EIO;
    }

  *status = (fsr << 8) | lsr;

  /* Read data from FIFO */
  ret = wk2132_i2c_read_fifo(priv, &rxdata);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WK2132: Failed to read RX FIFO for port %d: %d\n", priv->port, ret);
      return -EIO;
    }

  return rxdata;
}

/**
 * @brief Enable/disable RX interrupt
 */
static void wk2132_rxint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t sier;
  int ret;

  /* Check if device is enabled */
  if (!priv->enabled)
    {
      return;
    }

  /* Read current interrupt enable register */
  ret = wk2132_i2c_read_reg(priv, WK2132_SIER, &sier);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WK2132: Failed to read SIER for port %d: %d\n", priv->port, ret);
      return;
    }

  /* Enable/disable RX interrupt */
  if (enable)
    {
      sier |= WK2132_SIER_RFTRIG_IEN;
    }
  else
    {
      sier &= ~WK2132_SIER_RFTRIG_IEN;
    }

  ret = wk2132_i2c_write_reg(priv, WK2132_SIER, sier);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WK2132: Failed to write SIER for port %d: %d\n", priv->port, ret);
    }
}

/**
 * @brief Check if RX data available
 */
static bool wk2132_rxavailable(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t fsr;

  if (!priv->enabled)
    {
      return false;
    }

  /* Check FIFO status register for available data */
  if (wk2132_i2c_read_reg(priv, WK2132_FSR, &fsr) < 0)
    {
      return false;
    }

  return (fsr & WK2132_FSR_RDAT) != 0;
}

/**
 * @brief Send character
 */
static void wk2132_send(FAR struct uart_dev_s *dev, int ch)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  int ret;

  /* Check if device is enabled */
  if (!priv->enabled)
    {
      return;
    }

  /* Write data to FIFO */
  ret = wk2132_i2c_write_fifo(priv, (uint8_t)ch);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WK2132: Failed to send character 0x%02x to port %d: %d\n",
             (uint8_t)ch, priv->port, ret);
    }
}

/**
 * @brief Enable/disable TX interrupt
 */
static void wk2132_txint(FAR struct uart_dev_s *dev, bool enable)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t sier;
  int ret;

  /* Check if device is enabled */
  if (!priv->enabled)
    {
      return;
    }

  /* Read current interrupt enable register */
  ret = wk2132_i2c_read_reg(priv, WK2132_SIER, &sier);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WK2132: Failed to read SIER for port %d: %d\n", priv->port, ret);
      return;
    }

  /* Enable/disable TX interrupt */
  if (enable)
    {
      sier |= WK2132_SIER_TFTRIG_IEN;
    }
  else
    {
      sier &= ~WK2132_SIER_TFTRIG_IEN;
    }

  ret = wk2132_i2c_write_reg(priv, WK2132_SIER, sier);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WK2132: Failed to write SIER for port %d: %d\n", priv->port, ret);
    }
}

/**
 * @brief Check if TX ready
 */
static bool wk2132_txready(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t fsr;

  if (!priv->enabled)
    {
      return false;
    }

  /* Check FIFO status register */
  if (wk2132_i2c_read_reg(priv, WK2132_FSR, &fsr) < 0)
    {
      return false;
    }

  /* Return true if TX FIFO is not full */
  return (fsr & WK2132_FSR_TFULL) == 0;
}

/**
 * @brief Check if TX empty
 */
static bool wk2132_txempty(FAR struct uart_dev_s *dev)
{
  FAR struct wk2132_dev_s *priv = (FAR struct wk2132_dev_s *)dev->priv;
  uint8_t fsr;

  if (!priv->enabled)
    {
      return true;
    }

  /* Check FIFO status register */
  if (wk2132_i2c_read_reg(priv, WK2132_FSR, &fsr) < 0)
    {
      return false;
    }

  /* Return true if TX is not busy and no data available */
  return ((fsr & WK2132_FSR_TBUSY) == 0) && ((fsr & WK2132_FSR_TDAT) == 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/**
 * @brief Initialize a WK2132 UART port
 */
FAR struct uart_dev_s *wk2132_uart_init(FAR struct i2c_master_s *i2c,
                                         uint8_t base_addr, uint8_t port)
{
  FAR struct wk2132_dev_s *priv;
  FAR struct uart_dev_s *dev;
  int ret;

  /* Enable debug logging for WK2132 driver */
  setlogmask(LOG_UPTO(LOG_DEBUG));

  syslog(LOG_INFO, "WK2132: Initializing port %d with base_addr 0x%02x\n", port, base_addr);

  /* Validate parameters */
  if (!i2c || port < 1 || port > WK2132_MAX_PORTS)
    {
      syslog(LOG_ERR, "WK2132: Invalid parameters - i2c=%p, port=%d (valid range 1-%d)\n",
             i2c, port, WK2132_MAX_PORTS);
      return NULL;
    }

  /* Allocate device structures */
  priv = (FAR struct wk2132_dev_s *)kmm_zalloc(sizeof(struct wk2132_dev_s));
  if (priv == NULL)
    {
      syslog(LOG_ERR, "WK2132: Failed to allocate private structure for port %d\n", port);
      return NULL;
    }

  syslog(LOG_DEBUG, "WK2132: Allocated private structure for port %d\n", port);

  dev = (FAR struct uart_dev_s *)kmm_zalloc(sizeof(struct uart_dev_s));
  if (dev == NULL)
    {
      syslog(LOG_ERR, "WK2132: Failed to allocate uart_dev_s structure for port %d\n", port);
      kmm_free(priv);
      return NULL;
    }

  syslog(LOG_DEBUG, "WK2132: Allocated uart_dev_s structure for port %d\n", port);

  /* Initialize private structure */
  priv->i2c       = i2c;
  priv->base_addr = base_addr;
  priv->port      = port;
  priv->baud      = 115200;  /* Default baud rate */
  priv->parity    = 0;       /* No parity */
  priv->nbits     = 8;       /* 8 data bits */
  priv->stopbits2 = false;   /* 1 stop bit */
  priv->enabled   = false;

  nxsem_init(&priv->exclsem, 0, 1);

  syslog(LOG_DEBUG, "WK2132: Initialized private structure - port=%d, baud=%lu, bits=%lu\n",
         priv->port, (unsigned long)priv->baud, priv->nbits);

  /* Initialize public structure */
  dev->ops      = &g_wk2132_uart_ops;
  dev->priv     = priv;
  dev->isconsole = false;

  /* Allocate RX/TX buffers - increased to 512 bytes for better performance */
  dev->xmit.size   = 512;
  dev->xmit.buffer = (FAR char *)kmm_malloc(dev->xmit.size);
  if (dev->xmit.buffer == NULL)
    {
      syslog(LOG_ERR, "WK2132: Failed to allocate TX buffer for port %d\n", port);
      goto errout;
    }

  dev->recv.size   = 512;
  dev->recv.buffer = (FAR char *)kmm_malloc(dev->recv.size);
  if (dev->recv.buffer == NULL)
    {
      syslog(LOG_ERR, "WK2132: Failed to allocate RX buffer for port %d\n", port);
      goto errout;
    }

  syslog(LOG_DEBUG, "WK2132: Allocated TX/RX buffers (512 bytes each) for port %d\n", port);

  /* Test if device is present by reading a global register */
  uint8_t test;
  syslog(LOG_DEBUG, "WK2132: Testing device presence at base_addr 0x%02x\n", base_addr);
  ret = wk2132_i2c_read_global(priv, WK2132_GENA, &test);
  if (ret < 0)
    {
      syslog(LOG_ERR, "WK2132: Failed to detect device at base_addr 0x%02x (error=%d)\n", base_addr, ret);
      goto errout;
    }

  syslog(LOG_INFO, "WK2132: Device detected at base_addr 0x%02x, GENA=0x%02x\n", base_addr, test);

  /* Add to device list for polling */
  if (g_wk2132_device_count < WK2132_MAX_PORTS)
    {
      g_wk2132_devices[g_wk2132_device_count++] = dev;
      syslog(LOG_DEBUG, "WK2132: Added device to polling list, count=%d\n", g_wk2132_device_count);
    }
  else
    {
      syslog(LOG_WARNING, "WK2132: Maximum device count reached, polling may not work for port %d\n", port);
    }

  syslog(LOG_INFO, "WK2132: Initialized port %d with base_addr 0x%02x\n", port, base_addr);
  return dev;

errout:
  if (dev->xmit.buffer)
    {
      kmm_free(dev->xmit.buffer);
    }
  if (dev->recv.buffer)
    {
      kmm_free(dev->recv.buffer);
    }
  kmm_free(dev);
  kmm_free(priv);
  return NULL;
}

/**
 * @brief Register WK2132 serial devices
 */
int wk2132_register_devices(int i2c_bus, uint8_t i2c_base_addr,
                            int base_tty, int num_ports)
{
  FAR struct i2c_master_s *i2c;
  FAR struct uart_dev_s *devs[WK2132_MAX_PORTS];
  char devpath[16];
  int i;
  int ret;

  syslog(LOG_INFO, "WK2132: Registering %d devices on I2C%d, base_addr=0x%02x, base_tty=%d\n",
         num_ports, i2c_bus, i2c_base_addr, base_tty);

  /* Validate parameters */
  if (num_ports < 1 || num_ports > WK2132_MAX_PORTS)
    {
      syslog(LOG_ERR, "WK2132: Invalid num_ports=%d (valid range 1-%d)\n", num_ports, WK2132_MAX_PORTS);
      return -EINVAL;
    }

  /* Get I2C bus instance */
  syslog(LOG_DEBUG, "WK2132: Initializing I2C%d bus\n", i2c_bus);
  i2c = stm32_i2cbus_initialize(i2c_bus);
  if (i2c == NULL)
    {
      syslog(LOG_ERR, "WK2132: Failed to get I2C%d\n", i2c_bus);
      return -ENODEV;
    }

  syslog(LOG_DEBUG, "WK2132: I2C%d bus initialized successfully\n", i2c_bus);

  /* Initialize UART ports */
  for (i = 0; i < num_ports; i++)
    {
      syslog(LOG_DEBUG, "WK2132: Initializing UART port %d\n", i + 1);
      devs[i] = wk2132_uart_init(i2c, i2c_base_addr, i + 1);
      if (devs[i] == NULL)
        {
          syslog(LOG_ERR, "WK2132: Failed to initialize port %d\n", i + 1);
          ret = -ENODEV;
          goto cleanup;
        }
      syslog(LOG_DEBUG, "WK2132: Successfully initialized UART port %d\n", i + 1);
    }

  /* Register UART devices */
  for (i = 0; i < num_ports; i++)
    {
      snprintf(devpath, sizeof(devpath), "/dev/ttyS%d", base_tty + i);
      syslog(LOG_DEBUG, "WK2132: Registering device %s for port %d\n", devpath, i + 1);
      ret = uart_register(devpath, devs[i]);
      if (ret < 0)
        {
          syslog(LOG_ERR, "WK2132: Failed to register %s (error=%d)\n", devpath, ret);
          goto cleanup;
        }

      syslog(LOG_INFO, "WK2132: Registered %s for port %d\n", devpath, i + 1);
    }

  syslog(LOG_INFO, "WK2132: Successfully registered all %d devices\n", num_ports);
  return OK;

cleanup:
  /* Cleanup on failure */
  syslog(LOG_WARNING, "WK2132: Cleaning up %d partially initialized devices\n", i);
  for (int j = 0; j < i; j++)
    {
      if (devs[j])
        {
          /* Note: uart_unregister would be called here if it existed */
          /* For now, we'll just free the memory */
          if (devs[j]->xmit.buffer)
            {
              kmm_free(devs[j]->xmit.buffer);
            }
          if (devs[j]->recv.buffer)
            {
              kmm_free(devs[j]->recv.buffer);
            }
          if (devs[j]->priv)
            {
              nxsem_destroy(&((FAR struct wk2132_dev_s *)devs[j]->priv)->exclsem);
              kmm_free(devs[j]->priv);
            }
          kmm_free(devs[j]);
          devs[j] = NULL;
        }
    }

  return ret;
}
