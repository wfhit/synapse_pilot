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
 * @file board_limit_sensor_config.cpp
 *
 * Board-specific limit sensor configuration for NXT-Dual-WL-Front
 */

#include <board_config.h>
#include <drivers/limit_sensor/limit_sensor_config.h>

#ifdef BOARD_HAS_LIMIT_SENSOR_CONFIG

// Define limit sensor instances with their GPIO mappings for front board
const limit_sensor_config_t g_limit_sensor_config[BOARD_NUM_LIMIT_SENSORS] = {
    // Instance 0: Bucket Load Limit (WITH REDUNDANCY)
    {
        .instance_id = 0,
        .function = 0,  // BUCKET_LOAD
        .gpio_pin_1 = BUCKET_LOAD_LIMIT_SW1_GPIO,   // PB10
        .gpio_pin_2 = BUCKET_LOAD_LIMIT_SW2_GPIO,   // PB11
        .inverted = false,
        .redundancy_enabled = true,  // REDUNDANCY ENABLED
        .name = "bucket_load"
    },
    // Instance 1: Bucket Dump Limit (WITH REDUNDANCY)
    {
        .instance_id = 1,
        .function = 1,  // BUCKET_DUMP
        .gpio_pin_1 = BUCKET_DUMP_LIMIT_SW1_GPIO,   // PB0
        .gpio_pin_2 = BUCKET_DUMP_LIMIT_SW2_GPIO,   // PB1
        .inverted = false,
        .redundancy_enabled = true,  // REDUNDANCY ENABLED
        .name = "bucket_dump"
    }
};

#endif // BOARD_HAS_LIMIT_SENSOR_CONFIG
