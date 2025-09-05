#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include "drivers/tle5012b.h"
#include "drivers/feedback.h"

#define DT_DRV_COMPAT infineon_tle5012b

#define TLE5012_READ_ANGLE_CMD	0x8021
#define TLE5012_SEND_CMD_BYTES	2
#define TLE5012_READ_DATA_BYTES 4

struct tle5012b_config {
	struct spi_dt_spec spi_port;
};

struct feedback_data {
	float eangle; //
	float eomega; // rad
	float odom;   // m

	uint16_t raw;
};

static inline int spi_exchange_hal_sr(const struct device *dev, uint16_t *pData)
{
	const struct tle5012b_config *cfg = dev->config;
	static const uint8_t tx_buf[TLE5012_SEND_CMD_BYTES] = {
		TLE5012_READ_ANGLE_CMD & 0xFF,
		(TLE5012_READ_ANGLE_CMD >> 8) & 0xFF,
	};
	uint8_t rx_buf[TLE5012_READ_DATA_BYTES];

	struct spi_buf tx = {.buf = (uint8_t *)tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};

	struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
	struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};
	spi_transceive_dt(&cfg->spi_port, &tx_set, &rx_set);
	*pData = (rx_buf[0] << 8) | rx_buf[1];
	return 0;
}

static int tle5012b_init(const struct device *dev)
{
	const struct tle5012b_config *cfg = dev->config;
	if (!spi_is_ready_dt(&cfg->spi_port)) {
		return -ENODEV;
	}
	return 0;
}

int tle5012b_read(const struct device *dev)
{
	uint16_t pData;
	spi_exchange_hal_sr(dev, &pData);
	return 0;
}

static float feedback_cacle_eangle(const struct device *dev)
{
	return 0.0f;
}
static float feedback_cacle_eomega(const struct device *dev)
{
	return 0.0f;
}
static float feedback_cacle_odom(const struct device *dev)
{
	return 0.0f;
}
static int feedback_calibration_firstangle(const struct device *dev)
{
	return 0;
}
static const struct feedback_driver_api driver_feedback = {
	.get_rads = feedback_cacle_eomega,
	.get_eangle = feedback_cacle_eangle,
	.get_rel_odom = feedback_cacle_odom,
	.calibration = feedback_calibration_firstangle,
	.set_rel_odom = NULL,
};

#define TLE5012B_DEFINE(inst)                                                                      \
	static struct feedback_data feedback_data_##inst;                                          \
	static const struct tle5012b_config tle5012b_cfg_##inst = {                                \
		.spi_port = SPI_DT_SPEC_INST_GET(inst,                                             \
						 SPI_OP_MODE_MASTER | SPI_HALF_DUPLEX |            \
							 SPI_WORD_SET(16) | SPI_MODE_CPHA,         \
						 0),                                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, tle5012b_init, NULL, &feedback_data_##inst,                    \
			      &tle5012b_cfg_##inst, POST_KERNEL, CONFIG_FEEDBACK_INIT_PRIORITY,    \
			      &driver_feedback);

DT_INST_FOREACH_STATUS_OKAY(TLE5012B_DEFINE)
