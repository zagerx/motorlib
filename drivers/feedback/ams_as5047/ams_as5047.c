/*
 * Copyright (c) 2022, Felipe Neves
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT ams_as5047

#include <errno.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ams_as5047, CONFIG_SENSOR_LOG_LEVEL);

#define AS5047_ANGLE_REGISTER_H 0x0E
#define AS5047_FULL_ANGLE	360
#define AS5047_PULSES_PER_REV	4096
#define AS5047_MILLION_UNIT	1000000

struct as5047_dev_cfg {
	struct spi_dt_spec spi_port;
};

/* Device run time data */
struct as5047_dev_data {
	uint16_t position;
};

static int as5047_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct as5047_dev_data *dev_data = dev->data;
	const struct as5047_dev_cfg *dev_cfg = dev->config;

	uint16_t rx_buf;
	uint16_t tx_buf = AS5047_ANGLE_REGISTER_H | 0x4000;
	struct spi_buf spi_tx = {.buf = &tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf spi_rx = {.buf = &rx_buf, .len = sizeof(rx_buf)};
	struct spi_buf_set tx = {.buffers = &spi_tx, .count = 1};
	struct spi_buf_set rx = {.buffers = &spi_rx, .count = 1};

	int ret = spi_transceive_dt(&dev_cfg->spi_port, &tx, &rx);
	if (ret < 0) {
		LOG_ERR("SPI transceive failed: %d", ret);
		return ret;
	}

	dev_data->position = rx_buf & 0x3FFF;
	return 0;
}

static int as5047_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
	struct as5047_dev_data *dev_data = dev->data;

	if (chan == SENSOR_CHAN_ROTATION) {
		val->val1 =
			((int32_t)dev_data->position * AS5047_FULL_ANGLE) / AS5047_PULSES_PER_REV;

		val->val2 = (((int32_t)dev_data->position * AS5047_FULL_ANGLE) %
			     AS5047_PULSES_PER_REV) *
			    (AS5047_MILLION_UNIT / AS5047_PULSES_PER_REV);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static int as5047_initialize(const struct device *dev)
{
	struct as5047_dev_data *const dev_data = dev->data;

	dev_data->position = 0;

	LOG_INF("Device %s initialized", dev->name);

	return 0;
}

static DEVICE_API(sensor, as5047_driver_api) = {
	.sample_fetch = as5047_fetch,
	.channel_get = as5047_get,
};

#define AS5047_INIT(n)                                                                             \
	static struct as5047_dev_data as5047_data##n;                                              \
	static const struct as5047_dev_cfg as5047_cfg##n = {.spi_port = I2C_DT_SPEC_INST_GET(n)};  \
                                                                                                   \
	SENSOR_DEVICE_DT_INST_DEFINE(n, as5047_initialize, NULL, &as5047_data##n, &as5047_cfg##n,  \
				     POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                     \
				     &as5047_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AS5047_INIT)
