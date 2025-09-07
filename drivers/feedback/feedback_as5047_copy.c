
#include "zephyr/device.h"
#if CONFIG_SOC_STM32G431XX
#include "stm32g4xx_hal.h"
#endif
#define LOG_LEVEL LOG_LEVEL_DBG
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(as5047_copy);

#define DT_DRV_COMPAT ams_as5047

static uint32_t g_rawdata;
static uint16_t spi_rw_onebyte(uint16_t _txdata);
static uint8_t ams_parity(uint16_t v);
extern void MX_SPI1_Init(void);

void as5047_init(void)
{
	MX_SPI1_Init();
	g_rawdata = 0;
}

void *as5047_readangle(void)
{
	uint16_t data;
	data = spi_rw_onebyte(0xFFFF);
	g_rawdata = (uint32_t)data;
	// LOG_DBG("raw:%d", data);
	return (void *)&g_rawdata;
}
static uint8_t ams_parity(uint16_t v)
{
	v ^= v >> 8;
	v ^= v >> 4;
	v ^= v >> 2;
	v ^= v >> 1;
	return v & 1;
}
extern SPI_HandleTypeDef hspi1;

static uint16_t spi_rw_onebyte(uint16_t _txdata)
{
	uint16_t pos, rawVal;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&_txdata, (uint8_t *)&rawVal, 1, 1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	if (ams_parity(rawVal)) {
		return 0xFFFF;
	} else {
		return pos = (rawVal & 0x3fff);
	}
}

#include "drivers/feedback.h"
struct as5047_config {
	struct device *dev;
};
struct feedback_data {
	float eangle; //
	float eomega; // rad
	float odom;   // m

	uint16_t raw;
};
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

static int feedback_as5047_init(const struct device *dev)
{
	as5047_init();
	LOG_DBG("as5047 init");
	return 0;
}

#define FEEDBACK_AS5047_INIT(n)                                                                    \
	static const struct as5047_config as5047_config##n = {};                                   \
	static struct feedback_data feedback_data##n = {};                                         \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, feedback_as5047_init, NULL, &feedback_data##n, &as5047_config##n, \
			      POST_KERNEL, CONFIG_FEEDBACK_INIT_PRIORITY, &driver_feedback);

/* Initialize all instances */
DT_INST_FOREACH_STATUS_OKAY(FEEDBACK_AS5047_INIT)