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
 * @file drv_motor_pwm.h
 *
 * High-frequency PWM driver interface for motor control
 * Built on top of the io_timer infrastructure for consistency
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <stdint.h>
#include <stdbool.h>
#include <px4_arch/io_timer.h>

__BEGIN_DECLS

/* Motor PWM frequency options - higher than servo PWM */
#define MOTOR_PWM_FREQ_10KHZ	10000U
#define MOTOR_PWM_FREQ_20KHZ	20000U
#define MOTOR_PWM_FREQ_25KHZ	25000U  /* Recommended default */
#define MOTOR_PWM_FREQ_50KHZ	50000U
#define MOTOR_PWM_FREQ_100KHZ	100000U

/* Default motor PWM frequency */
#define MOTOR_PWM_DEFAULT_FREQ	MOTOR_PWM_FREQ_25KHZ

/* Motor PWM rate limits - higher than standard PWM servo */
#define MOTOR_PWM_RATE_LOWER_LIMIT	10000u
#define MOTOR_PWM_RATE_UPPER_LIMIT	100000u

/* Maximum motor PWM channels */
#define MOTOR_PWM_MAX_CHANNELS	8

/* PWM resolution (number of steps) */
#define MOTOR_PWM_RESOLUTION	8000U  /* At 25kHz with 200MHz timer */

/*
 * Low-level motor PWM functions (architecture-specific implementation)
 */

/**
 * Initialize motor PWM outputs using the specified configuration.
 * Uses the same io_timer infrastructure as PWM servo but with IOTimerChanMode_PWMOut
 *
 * @param channel_mask	Bitmask of channels (LSB = channel 0) to enable.
 * @return		<0 on error, the initialized channels mask on success.
 */
__EXPORT extern int	up_motor_pwm_init(uint32_t channel_mask);

/**
 * De-initialize motor PWM outputs.
 *
 * @param channel_mask	Bitmask of channels (LSB = channel 0) to deinit.
 *			A value of 0 is ALL channels
 */
__EXPORT extern void up_motor_pwm_deinit(uint32_t channel_mask);

/**
 * Arm or disarm motor PWM outputs.
 *
 * @param armed		If true, outputs are armed; if false they are disarmed.
 * @param channel_mask	Bitmask of channels (LSB = channel 0) to control.
 */
__EXPORT extern void up_motor_pwm_arm(bool armed, uint32_t channel_mask);

/**
 * Set the motor PWM update rate for all rate groups.
 *
 * @param rate		The update rate in Hz to set.
 * @return		OK on success, -ERANGE if an unsupported update rate is set.
 */
__EXPORT extern int	up_motor_pwm_set_rate(unsigned rate);

/**
 * Set the update rate for a given rate group (motor PWM version).
 *
 * @param group		The rate group whose update rate will be changed.
 * @param rate		The update rate in Hz.
 * @return		OK if the group was adjusted, -ERANGE if unsupported rate.
 */
__EXPORT extern int	up_motor_pwm_set_rate_group_update(unsigned group, unsigned rate);

/**
 * Get a bitmap of motor PWM channels assigned to a given rate group.
 *
 * @param group		The rate group to query.
 * @return		A bitmap of channels assigned to the rate group.
 */
__EXPORT extern uint32_t up_motor_pwm_get_rate_group(unsigned group);

/**
 * Set the current duty cycle for a motor PWM channel.
 * This differs from servo PWM by accepting duty cycle (0.0-1.0) instead of pulse width
 *
 * @param channel	The channel to set.
 * @param duty_cycle	The duty cycle (0.0 to 1.0).
 * @return		OK on success, negative on error.
 */
__EXPORT extern int	up_motor_pwm_set_duty_cycle(unsigned channel, float duty_cycle);

/**
 * Get the current duty cycle for a motor PWM channel.
 *
 * @param channel	The channel to read.
 * @return		The duty cycle (0.0 to 1.0), negative on error.
 */
__EXPORT extern float up_motor_pwm_get_duty_cycle(unsigned channel);

/**
 * Trigger an update of the motor PWM outputs.
 * Similar to up_pwm_update for servo PWM.
 *
 * @param channels_mask	Bitmask of channels to update.
 */
__EXPORT extern void up_motor_pwm_update(uint32_t channels_mask);

__END_DECLS
