/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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
 * @file board_config.h
 *
 * NXT-Dual-WL-Rear Board internal definitions
 * Rear wheel and boom control board
 */

#pragma once

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <px4_platform_common/px4_config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

#include <stm32_gpio.h>

/****************************************************************************************************
 * Definitions
 ****************************************************************************************************/

#define FLASH_BASED_PARAMS


/* LEDs are driven with push open drain to support Anode to 5V or 3.3V */

#  define GPIO_nLED_RED         /* PD15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN15)
#  define GPIO_nLED_GREEN       /* PD11 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTD|GPIO_PIN11)
#  define GPIO_nLED_BLUE        /* PB15 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN15)

#  define BOARD_HAS_CONTROL_STATUS_LEDS      1
#  define BOARD_OVERLOAD_LED     LED_RED
#  define BOARD_ARMED_STATE_LED  LED_BLUE


/* Load Lamp Controller GPIOs */
#define BOARD_HAS_LOAD_LAMP           1
#define GPIO_LOAD_LAMP_LEFT  /* PA4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTA|GPIO_PIN4)
#define GPIO_LOAD_LAMP_RIGHT /* PC1 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN1)
#define GPIO_LOAD_LAMP_GND   /* PC0 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTC|GPIO_PIN0)

/* Driver Lamp Controller GPIOs */
#define BOARD_HAS_DRIVER_LAMP         1
#define GPIO_DRIVER_LAMP_LEFT  /* PB2 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN2)
#define GPIO_DRIVER_LAMP_RIGHT /* PB4 */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN4)
#define GPIO_DRIVER_LAMP_GND   /* PB3 */  (GPIO_OUTPUT|GPIO_OPENDRAIN|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTB|GPIO_PIN3)

/* Convenience macros for load lamp control */
#define LOAD_LAMP_LEFT(on_true)        px4_arch_gpiowrite(GPIO_LOAD_LAMP_LEFT, (on_true))
#define LOAD_LAMP_RIGHT(on_true)       px4_arch_gpiowrite(GPIO_LOAD_LAMP_RIGHT, (on_true))

/* Convenience macros for driver lamp control */
#define DRIVER_LAMP_LEFT(on_true)  px4_arch_gpiowrite(GPIO_DRIVER_LAMP_LEFT, (on_true))
#define DRIVER_LAMP_RIGHT(on_true) px4_arch_gpiowrite(GPIO_DRIVER_LAMP_RIGHT, (on_true))

/* I2C busses */
/* Devices on the onboard buses.
 *
 * Note that these are unshifted addresses.
 */
// #define PX4_I2C_OBDEV_SE050         0x48

#define GPIO_SPL_ADDR_SET     /* PB5  */  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_SET|GPIO_PORTB|GPIO_PIN5)

/*
 * ADC channels
 *
 * These are the channel numbers of the ADCs of the microcontroller that
 * can be used by the Px4 Firmware in the adc driver
 */

/* ADC defines to be used in sensors.cpp to read from a particular channel */

#define SYSTEM_ADC_BASE STM32_ADC1_BASE

#define ADC12_CH(n)		(n)

#define PX4_ADC_GPIO  \
	/* PC4  */  GPIO_ADC12_INP4,   \
	/* PC5  */  GPIO_ADC12_INP8

/* Define GPIO pins used as ADC N.B. Channel numbers must match below  */
/* Define Channel numbers must match above GPIO pin IN(n)*/
#define ADC_BATTERY_VOLTAGE_CHANNEL             ADC12_CH(4)
#define ADC_BATTERY_CURRENT_CHANNEL             ADC12_CH(8)

#define ADC_CHANNELS \
	((1 << ADC_BATTERY_VOLTAGE_CHANNEL) | \
	 (1 << ADC_BATTERY_CURRENT_CHANNEL))

#define BOARD_ADC_OPEN_CIRCUIT_V     (1.6f)

/* Define Battery 1 Voltage Divider and A per V
 */

// #define BOARD_BATTERY1_V_DIV         (11.0f)     /* measured with the provided PM board */
// #define BOARD_BATTERY1_A_PER_V       (40.0f)
// #define BOARD_BATTERY2_V_DIV         (11.0f)     /* measured with the provided PM board */

/* Tone alarm output */

#define TONE_ALARM_TIMER        4 /* Timer 4 */
#define TONE_ALARM_CHANNEL      3  /* PD14 GPIO_TIM4_CH3 NC */

/*NC can be modified with Spare GPIO then connected with hardware */
#define GPIO_BUZZER_1           /* PA4 */ GPIO_PA4

#define GPIO_TONE_ALARM_IDLE    GPIO_BUZZER_1
#define GPIO_TONE_ALARM         GPIO_BUZZER_1

/* USB OTG FS
 *
 * PD0  OTG_FS_VBUS VBUS sensing
 */
#define GPIO_OTGFS_VBUS         /* PD0 */ (GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_100MHz|GPIO_PORTD|GPIO_PIN0)
#define BOARD_ADC_USB_CONNECTED (px4_arch_gpioread(GPIO_OTGFS_VBUS))

/* Serial Port Mapping (CONFIG_STM32H7_SERIAL_DISABLE_REORDERING=y)
 * NuttX assigns /dev/ttyS* in peripheral number order (not defconfig order):
 *
 * /dev/ttyS0  USART1  uORB proxy  921600  Inter-board comms with main board (UORB_PX_UART=0)
 * /dev/ttyS1  UART5   ST3215 servo 115200 (SRV1, smart servo steering)
 * /dev/ttyS2  UART8   NSH console 115200  (SERIAL_CONSOLE)
 *
 * Pins repurposed as GPIO (not available as serial):
 *   UART4  PA0/PA1   -> not used, disabled
 *   UART7  PE7       -> DRV8701 H-bridge enable GPIO
 *   USART2 PD5/PD6   -> quadrature encoder (TELEM1 port)
 *   USART3            -> not used, disabled
 *   USART6            -> not connected by board design
 */

/* High-resolution timer */
#define HRT_TIMER               8  /* use timer1 for the HRT */
#define HRT_TIMER_CHANNEL       1  /* use capture/compare channel 1 */

// #define GPIO_SBUS_INV                  (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTD|GPIO_PIN14)
// #define RC_INVERT_INPUT(_invert_true)  px4_arch_gpiowrite(GPIO_SBUS_INV, _invert_true);

/* SD card bringup does not work if performed on the IDLE thread because it
 * will cause waiting.  Use either:
 *
 *  CONFIG_LIB_BOARDCTL=y, OR
 *  CONFIG_BOARD_INITIALIZE=y && CONFIG_BOARD_INITTHREAD=y
 */
#define SDIO_SLOTNO             0  /* Only one slot */
#define SDIO_MINOR              0
#if defined(CONFIG_BOARD_INITIALIZE) && !defined(CONFIG_LIB_BOARDCTL) && \
   !defined(CONFIG_BOARD_INITTHREAD)
#  warning SDIO initialization cannot be perfomed on the IDLE thread
#endif

/* This board provides a DMA pool and APIs */
#define BOARD_DMA_ALLOC_POOL_SIZE 5120

/* This board provides the board_on_reset interface */
#define BOARD_HAS_ON_RESET 1

/* GPIO Pin Usage for WL-Rear:
 * TELEM1 port pins (PD5/PD6) - Quad encoder from motor encoder (A/B phases)
 * UART7 RX pin (PE7) - DRV8701 H-bridge enable signal
 * I2C4 (PD12/PD13) - AS5600 magnetic encoder
 * UART1 (PA9/PA10) - Proxy client communication
 * PWM1-4 - DRV8701 H-bridge control (direction and PWM signals)
 * PWM5-6 - Boom up/down limit switches
 * PWM7-8 - Steering left/right limit switches
 */

/* AS5600 I2C Configuration */
/* I2C4 Interface for AS5600 Magnetic Rotary Encoder */
// #define AS5600_I2C_BUS                     4           /* I2C4 bus */
// #define AS5600_I2C_SCL_GPIO                /* PD12 */ (GPIO_I2C4_SCL)
// #define AS5600_I2C_SDA_GPIO                /* PD13 */ (GPIO_I2C4_SDA)
// #define AS5600_I2C_ADDR                    0x36        /* AS5600 default I2C address */

/* UART1 Interface for Proxy Client */
// #define PROXY_CLIENT_UART_PORT             "/dev/ttyS0" /* USART1 for proxy client */
// #define PROXY_CLIENT_UART_TX_GPIO          /* PA9 */ (GPIO_USART1_TX)
// #define PROXY_CLIENT_UART_RX_GPIO          /* PA10*/ (GPIO_USART1_RX)

/* Interface Support */
// #define BOARD_HAS_AS5600_I2C               1
// #define BOARD_AS5600_I2C_ENABLED           1
// #define BOARD_PROXY_CLIENT_UART_ENABLED    1

/* ST3125 Servo Serial Port */
// #define ST3125_SERVO_SERIAL_PORT           "/dev/ttyS1"  /* TELEM1 port for ST3125 servo */

/* PWM
 * PWM1-4 available for PWM output (PWM5-8 used as limit switch GPIO inputs)
 * PWM1,4 used as DIR signals (GPIO), PWM2,3 used as PWM signals for DRV8701
 */
#define DIRECT_PWM_OUTPUT_CHANNELS   		2
#define BOARD_HAS_PWM  DIRECT_PWM_OUTPUT_CHANNELS

/* H-Bridge Configuration for Rear Board */
#define BOARD_NUM_HBRIDGES                 	1
#define BOARD_HAS_HBRIDGE_CONFIG           	1

/* DRV8701 H-Bridge Control and Limit Switch Configuration */
/* PWM1 (PE13) and PWM4 (PE14) - Direction signals for DRV8701 H-bridge */
// initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel3}, {GPIO::PortE, GPIO::Pin13}),
// initIOTimerChannel(io_timers, {Timer::Timer1, Timer::Channel4}, {GPIO::PortE, GPIO::Pin14}),
#define DRV8701_RIGHT_DIR_GPIO             /* PE13 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN13) // Right wheel direction
#define DRV8701_LEFT_DIR_GPIO              /* PE14 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN14) // Left wheel direction

/* UART7 RX - Enable signal for DRV8701 H-bridge */
#define DRV8701_ENABLE_GPIO                /* PE7  */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_50MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN7)

/* Limit Sensor Configuration for Rear Board */
/* Only boom and steering sensors are configured on rear board */
#define BOARD_NUM_LIMIT_SENSORS            	4
#define BOARD_HAS_LIMIT_SENSOR_CONFIG      	1

/* PWM5-8 - Limit switch inputs for boom and steering operations */
#define STEERING_RIGHT_LIMIT_SW_GPIO       /* PB10 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN10)  /* PWM5 */
#define STEERING_LEFT_LIMIT_SW_GPIO        /* PB11 */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN11)  /* PWM6 */
#define BOOM_DOWN_LIMIT_SW_GPIO            /* PB0  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN0)   /* PWM7 */
#define BOOM_UP_LIMIT_SW_GPIO              /* PB1  */ (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTB|GPIO_PIN1)   /* PWM8 */


/* Quad Encoder Configuration for Rear Board */
#define BOARD_NUM_QUAD_ENCODERS            1
#define BOARD_HAS_QUAD_ENCODER_CONFIG      1

/* Quad Encoder GPIO pins - Motor encoder A/B phases on TELEM1 port */
#define QUAD_ENCODER_A_GPIO                /* PD5 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN5)
#define QUAD_ENCODER_B_GPIO                /* PD6 */ (GPIO_INPUT|GPIO_FLOAT|GPIO_EXTI|GPIO_PORTD|GPIO_PIN6)

#define PX4_GPIO_INIT_LIST { \
		PX4_ADC_GPIO, \
		GPIO_LOAD_LAMP_LEFT, \
		GPIO_LOAD_LAMP_RIGHT, \
		GPIO_LOAD_LAMP_GND, \
		GPIO_DRIVER_LAMP_LEFT, \
		GPIO_DRIVER_LAMP_RIGHT, \
		GPIO_DRIVER_LAMP_GND, \
		GPIO_SPL_ADDR_SET, \
		DRV8701_RIGHT_DIR_GPIO, \
		DRV8701_LEFT_DIR_GPIO, \
		DRV8701_ENABLE_GPIO, \
		BOOM_UP_LIMIT_SW_GPIO, \
		BOOM_DOWN_LIMIT_SW_GPIO, \
		STEERING_LEFT_LIMIT_SW_GPIO, \
		STEERING_RIGHT_LIMIT_SW_GPIO, \
	}

#define BOARD_ENABLE_CONSOLE_BUFFER

#define BOARD_NUM_IO_TIMERS 5


__BEGIN_DECLS

/****************************************************************************************************
 * Public Types
 ****************************************************************************************************/

/****************************************************************************************************
 * Public data
 ****************************************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************************************
 * Public Functions
 ****************************************************************************************************/

/****************************************************************************
 * Name: stm32_sdio_initialize
 *
 * Description:
 *   Initialize SDIO-based MMC/SD card support
 *
 ****************************************************************************/

int stm32_sdio_initialize(void);

/****************************************************************************************************
 * Name: stm32_spiinitialize
 *
 * Description:
 *   Called to configure SPI chip select GPIO pins for the board.
 *
 ****************************************************************************************************/

extern void stm32_spiinitialize(void);

extern void stm32_usbinitialize(void);

extern void board_peripheral_reset(int ms);

#include <px4_platform_common/board_common.h>

#endif /* __ASSEMBLY__ */

__END_DECLS
