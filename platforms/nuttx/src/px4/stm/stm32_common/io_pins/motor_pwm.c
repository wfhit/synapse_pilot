/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file motor_pwm.c
 *
 * Motor PWM output driver for high-frequency PWM control of electric motors
 * and H-bridge drivers. Supports frequencies from 10kHz to 100kHz.
 */

#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <sys/types.h>
#include <stdbool.h>
#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_motor_pwm.h>

#include <px4_arch/io_timer.h>

#include <stm32_tim.h>

/* Store current duty cycles for each channel */
static float motor_pwm_duty_cycles[MAX_TIMER_IO_CHANNELS] = {0};

int up_motor_pwm_set(unsigned channel, uint16_t value)
{
	return io_timer_set_ccr(channel, value);
}

uint16_t up_motor_pwm_get(unsigned channel)
{
	return io_channel_get_ccr(channel);
}

int up_motor_pwm_set_duty_cycle(unsigned channel, float duty_cycle)
{
	if (channel >= MAX_TIMER_IO_CHANNELS) {
		return -EINVAL;
	}

	/* Clamp duty cycle */
	if (duty_cycle < 0.0f) duty_cycle = 0.0f;
	if (duty_cycle > 1.0f) duty_cycle = 1.0f;

	/* Store the duty cycle */
	motor_pwm_duty_cycles[channel] = duty_cycle;

	/* Use a reasonable period value - this will be set properly by io_timer_set_pwm_rate */
	uint32_t period = 1000;  /* Default period, will be corrected by rate setting */

	/* Calculate new CCR value */
	uint32_t ccr_value = (uint32_t)(duty_cycle * period);

	/* Ensure CCR doesn't exceed reasonable bounds */
	if (ccr_value > 65535) {
		ccr_value = 65535;  /* Max for 16-bit timer */
	}

	return io_timer_set_ccr(channel, (uint16_t)ccr_value);
}

float up_motor_pwm_get_duty_cycle(unsigned channel)
{
	if (channel >= MAX_TIMER_IO_CHANNELS) {
		return -1.0f;
	}

	return motor_pwm_duty_cycles[channel];
}

int up_motor_pwm_init(uint32_t channel_mask)
{
	/* Init channels */
	uint32_t current = io_timer_get_mode_channels(IOTimerChanMode_PWMOut);

	for (unsigned channel = 0; (channel_mask != 0) && (channel < MAX_TIMER_IO_CHANNELS); channel++) {
		if (channel_mask & (1 << channel)) {

			/* Check if not already in PWM mode */
			if ((current & (1 << channel)) == 0) {
				int ret = io_timer_channel_init(channel, IOTimerChanMode_PWMOut, NULL, NULL);

				if (ret != OK) {
					return ret;
				}
			}

			channel_mask &= ~(1 << channel);
		}
	}

	return OK;
}

void up_motor_pwm_deinit(uint32_t channel_mask)
{
	/* Unallocate channels similar to PWM servo */
	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {
			io_timer_unallocate_channel(channel);
			motor_pwm_duty_cycles[channel] = 0.0f;
		}
	}
}

int up_motor_pwm_set_rate(unsigned rate)
{
	/* Validate motor PWM rate range */
	if ((rate < MOTOR_PWM_RATE_LOWER_LIMIT) || (rate > MOTOR_PWM_RATE_UPPER_LIMIT)) {
		return -ERANGE;
	}

	/* Set rate for all motor PWM timer groups */
	int ret = OK;
	for (unsigned group = 0; group < MAX_IO_TIMERS; group++) {
		uint32_t group_channels = up_motor_pwm_get_rate_group(group);
		if (group_channels != 0) {
			int group_ret = up_motor_pwm_set_rate_group_update(group, rate);
			if (group_ret != OK) {
				ret = group_ret;
			}
		}
	}

	return ret;
}

uint32_t up_motor_pwm_get_rate_group(unsigned group)
{
	/* only return the set of channels in the group which we own */
	return io_timer_get_mode_channels(IOTimerChanMode_PWMOut) &
	       io_timer_get_group(group);
}

int up_motor_pwm_set_rate_group_update(unsigned group, unsigned rate)
{
	if ((group >= MAX_IO_TIMERS)) {
		return ERROR;
	}

	/* Validate motor PWM rate range */
	if ((rate < MOTOR_PWM_RATE_LOWER_LIMIT) || (rate > MOTOR_PWM_RATE_UPPER_LIMIT)) {
		return -ERANGE;
	}

	return io_timer_set_pwm_rate(group, rate);
}

void up_motor_pwm_arm(bool armed, uint32_t channel_mask)
{
	io_timer_set_enable(armed, IOTimerChanMode_PWMOut, channel_mask);
}

void up_motor_pwm_update(uint32_t channels_mask)
{
	io_timer_trigger(channels_mask);
}
