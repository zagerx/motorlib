#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <math.h>
#include "drivers/tle5012b.h"

#define TLE5012_READ_ANGLE_CMD	0x8001
#define TLE5012_SEND_CMD_BYTES	2
#define TLE5012_READ_DATA_BYTES 4

struct tle5012b_config {
	struct spi_dt_spec spi_port;
};

static int spi_exchange_hal_sr(const struct device *dev, uint16_t *pData)
{
	const struct tle5012b_config *cfg = dev->config;
	static const uint8_t tx_buf[TLE5012_SEND_CMD_BYTES] = {
		TLE5012_READ_ANGLE_CMD & 0xFF,	      /* 0x01 */
		(TLE5012_READ_ANGLE_CMD >> 8) & 0xFF, /* 0x80 */
	};
	uint8_t rx_buf[TLE5012_READ_DATA_BYTES];

	struct spi_buf tx = {.buf = (uint8_t *)tx_buf, .len = sizeof(tx_buf)};
	struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};

	struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
	struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

	int ret = spi_transceive_dt(&cfg->spi_port, &tx_set, &rx_set);
	if (ret) {
		printk("SPI transceive failed: %d\n", ret);
		return ret;
	}

	printk("RX: %02x %02x %02x %02x\n", rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3]);

	return 0;
}

int tle5012b_init(const struct device *dev)
{
	const struct tle5012b_config *cfg = dev->config;
	if (!spi_is_ready_dt(&cfg->spi_port)) {
		printk("SPI spi_port not ready\n");
		return -ENODEV;
	}
	printk("TLE5012B initialized successfully\n");
	return 0;
}

static int tle5012b_read(const struct device *dev, struct tle5012b_data *data)
{
	uint16_t pData;
	int ret = spi_exchange_hal_sr(dev, &pData);
	if (ret) {
		printk("SPI exchange failed: %d\n", ret);
		return ret;
	}

	int16_t angle_raw = (int16_t)pData;
	int32_t angle_17 = angle_raw * 2;

	data->rawdata = angle_17;
	data->covdata = (int32_t)((angle_17 * 2 * 3.1415926f * 1048576LL) / 65536);

	printk("Raw: %d, Converted: %d\n", data->rawdata, data->covdata);
	return 0;
}

#define TLE5012B_DEFINE(inst)                                                                      \
	static struct tle5012b_data tle5012b_data_##inst;                                          \
	static const struct tle5012b_config tle5012b_cfg_##inst = {                                \
		.spi_port = SPI_DT_SPEC_INST_GET(inst,                                             \
						 SPI_OP_MODE_MASTER | SPI_HALF_DUPLEX |            \
							 SPI_WORD_SET(16) | SPI_MODE_CPHA,         \
						 0),                                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, NULL, NULL, &tle5012b_data_##inst, &tle5012b_cfg_##inst,       \
			      POST_KERNEL, CONFIG_FEEDBACK_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(TLE5012B_DEFINE)

int tle5012b_get_data(const struct device *dev, struct tle5012b_data *data)
{
	if (!device_is_ready(dev)) {
		return -ENODEV;
	}
	return tle5012b_read(dev, data);
}

// 当前MOSI上的数据是0x80 0x01 间隔65us，0x80,0x00,0xfe,0x2A.
// 请分析一下这组数据是否符合数据手册