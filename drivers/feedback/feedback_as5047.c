#define DT_DRV_COMPAT ams_as5047
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>

#include "drivers/feedback.h"

#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(driver_feedback_as5047);
#define AS5047_ANGLE_REGISTER_H 0x0E
#define AS5047_FULL_ANGLE       360
#define AS5047_PULSES_PER_REV   4096
#define AS5047_MILLION_UNIT     1000000

struct as5047_config {
	struct spi_dt_spec spi_port;
};

struct as5047_data {
	uint16_t position;
	float angle;
	float odom;
};
static float as5047_fetch(const struct device *dev)
{
	struct as5047_data *dev_data = dev->data;
	const struct as5047_config *dev_cfg = dev->config;

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
	dev_data->angle = (float)((dev_data->position * AS5047_FULL_ANGLE) / AS5047_PULSES_PER_REV);
	return dev_data->angle;
}

static const struct feedback_driver_api driver_feedback = {
	.get_rads = NULL,
	.get_eangle = as5047_fetch,
	.calibration = NULL,
	.get_rel_odom = NULL,
	.set_rel_odom = NULL,
};

static int feedback_as5047_init(const struct device *dev)
{
	LOG_DBG("feedback_as5047 init");
	return 0;
}
#define FEEDBACK_AS5047_INIT(n)                                                                    \
	static const struct as5047_config as5047_config##n = {};                                   \
	static struct as5047_data as5047_data##n = {};                                             \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, feedback_as5047_init, NULL, &as5047_data##n, &as5047_config##n,   \
			      POST_KERNEL, CONFIG_FEEDBACK_INIT_PRIORITY, &driver_feedback);

/* Initialize all instances */
DT_INST_FOREACH_STATUS_OKAY(FEEDBACK_AS5047_INIT)