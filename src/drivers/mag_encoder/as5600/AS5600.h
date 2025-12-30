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

#include <stdint.h>
#include <drivers/device/i2c.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/sensor_mag_encoder.h>
#include <uORB/Publication.hpp>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

/* AS5600 Registers */
#define AS5600_ZMCO_REG			0x00
#define AS5600_ZPOS_H_REG		0x01
#define AS5600_ZPOS_L_REG		0x02
#define AS5600_MPOS_H_REG		0x03
#define AS5600_MPOS_L_REG		0x04
#define AS5600_MANG_H_REG		0x05
#define AS5600_MANG_L_REG		0x06
#define AS5600_CONF_H_REG		0x07
#define AS5600_CONF_L_REG		0x08
#define AS5600_RAW_ANGLE_H_REG		0x0C
#define AS5600_RAW_ANGLE_L_REG		0x0D
#define AS5600_ANGLE_H_REG		0x0E
#define AS5600_ANGLE_L_REG		0x0F
#define AS5600_STATUS_REG		0x0B
#define AS5600_AGC_REG			0x1A
#define AS5600_MAGNITUDE_H_REG		0x1B
#define AS5600_MAGNITUDE_L_REG		0x1C
#define AS5600_BURN_REG			0xFF

/* AS5600 Status Register Bits */
#define AS5600_STATUS_MAGNET_HIGH	(1 << 3)
#define AS5600_STATUS_MAGNET_LOW	(1 << 4)
#define AS5600_STATUS_MAGNET_DETECTED	(1 << 5)

/* AS5600 I2C Address */
#define AS5600_I2C_ADDR			0x36

/* AS5600 Constants */
#define AS5600_MAX_ANGLE_VALUE		4095
#define AS5600_ANGLE_TO_RAD		(2.0f * M_PI_F / AS5600_MAX_ANGLE_VALUE)

using namespace time_literals;

/*
 * Driver for AS5600 magnetic rotary position sensor.
 * Reads 12-bit angular position and publishes sensor data.
 */
class AS5600 : public device::I2C, public I2CSPIDriver<AS5600>
{
public:
	AS5600(const I2CSPIDriverConfig &config);
	~AS5600() override;

	int init() override;

	static void print_usage();

	void RunImpl();

	int probe() override;

protected:

	void print_status() override;

	void exit_and_cleanup() override;

private:

	uORB::Publication<sensor_mag_encoder_s> _sensor_mag_encoder_pub{ORB_ID(sensor_mag_encoder)};

	static const hrt_abstime SAMPLE_INTERVAL{10_ms};

	sensor_mag_encoder_s _sensor_mag_encoder{};

	perf_counter_t _cycle_perf;
	perf_counter_t _comms_errors;

	uint16_t _raw_angle{0};
	uint16_t _magnitude{0};
	uint8_t _status{0};
	uint8_t _agc{0};

	static constexpr uint8_t MAX_READY_COUNTER{20};
	uint8_t _ready_counter{MAX_READY_COUNTER};

	// AS5600 specific methods
	bool readAngle();
	bool readStatus();
	bool readMagnitude();
	bool readSensorData();

	void processSensorData();
	void publishSensorData();
	void updateReadyCounter(bool success);

	float normalizeAngle(float angle);
	void printMagnetStatus();

	int readReg(uint8_t addr, uint8_t *buf, size_t len);
	int writeReg(uint8_t addr, uint8_t *buf, size_t len);

};
