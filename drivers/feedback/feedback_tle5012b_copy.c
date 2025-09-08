#include <math.h>
#include <sys/_stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>

#include "drivers/feedback.h"
#include "filter.h"
#include "lib/motor/motor_Parameter.h"
#include "amplitude_limiting_filter.h"
#include "zephyr/sys/printk.h"
#if CONFIG_SOC_STM32H723XX || CONFIG_SOC_STM32G431XX
#include "stm32h7xx_hal_spi.h"
#endif
#define DT_DRV_COMPAT infineon_tle5012b

#define TLE5012_READ_ANGLE_CMD (0x80)
#define TLE5012_TRANS_TIMEOUT  (2)
#define SPI_REG_ADRRESS_ANGLE  (2)

SPI_HandleTypeDef hspi3;

static inline float _normalize_angle(float angle)
{
	float a = fmodf(angle, 360.0f);
	return a >= 0 ? a : (a + 360.0f);
}
void MX_SPI3_Init(void)
{
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_MASTER;
	hspi3.Init.Direction = SPI_DIRECTION_1LINE;
	hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
	hspi3.Init.NSS = SPI_NSS_SOFT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
	}
}

void HAL_SPI_MspInit(SPI_HandleTypeDef *spiHandle)
{

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if (spiHandle->Instance == SPI3) {
		__HAL_RCC_SPI3_CLK_ENABLE();
		__HAL_RCC_GPIOC_CLK_ENABLE();
		GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_15;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef *spiHandle)
{

	if (spiHandle->Instance == SPI3) {
		__HAL_RCC_SPI3_CLK_DISABLE();
		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_12);
	}
}

static inline uint8_t _crc8(uint8_t *uc_data, uint8_t uc_len)
{
	uint8_t uc_crc_polynomial = 0x1d; // x^8 + x^4 + x^3 + x^2 + 1
	uint8_t uc_i, uc_j;
	uint8_t u1Crc = 0xff;
	for (uc_j = 0; uc_j < uc_len; uc_j++) {
		u1Crc ^= *(uc_data + uc_j);
		for (uc_i = 0; uc_i < 8; uc_i++) {
			if (u1Crc & 0x80) {
				u1Crc = (uint8_t)((u1Crc << 1) ^ uc_crc_polynomial);
			} else {
				u1Crc = (uint8_t)(u1Crc << 1);
			}
		}
	}

	return (~u1Crc);
}
static inline uint8_t _read_angle_reg(SPI_HandleTypeDef *hSpi, uint16_t *pData)
{
	uint8_t ret = 0u;
	uint8_t buf[6] = {0};
	uint8_t txbuf[2] = {0};
	uint8_t rxbuf[4] = {0};

	txbuf[0] = TLE5012_READ_ANGLE_CMD;
	txbuf[1] = 0x21;
	ret = 1;
	if (HAL_SPI_Transmit(hSpi, txbuf, 2, 0XFFFF) == 0u) {
		if (HAL_SPI_Receive(hSpi, &buf[2], 4, 0XFFFF) == 0u) {
			buf[0] = txbuf[0];
			buf[1] = txbuf[1];
			uint8_t u1Crc = _crc8(buf, 4);
			if (u1Crc == buf[5]) {
				*pData = ((uint16_t)buf[2] << 8) + buf[3];
				ret = 0;
			}
		}
	}
	// uint8_t ret = 0u;
	// uint8_t crcbuf[6] = {0};
	// uint8_t txbuf[6] = {0};
	// uint8_t rxbuf[4] = {0};

	// txbuf[0] = (TLE5012_READ_ANGLE_CMD >> 8) & 0xFF;
	// txbuf[1] = TLE5012_READ_ANGLE_CMD & 0xFF;
	// ret = 1;
	// HAL_SPI_TransmitReceive(hSpi, txbuf, &txbuf[2], 6, 0xFF);
	// crcbuf[0] = txbuf[0];
	// crcbuf[1] = txbuf[1];
	// crcbuf[2] = rxbuf[0];
	// crcbuf[3] = rxbuf[1];
	// uint8_t u1Crc = _crc8(crcbuf, 4);
	// if (u1Crc == rxbuf[3]) {
	// 	*pData = ((uint16_t)txbuf[2] << 8) + txbuf[3];
	// 	ret = 0;
	// }
	return ret;
}

struct tle5012b_config {
	struct spi_dt_spec spi_port;
};

struct feedback_data {
	float eangle; //
	float eomega; // rad
	float odom;   // m

	float offset;
	short calibration_state;
	AmplitudeLimitingFilter filter;
	lowfilter_t speedfilter;
	uint16_t raw;
	uint16_t last_raw;
};

static int tle5012b_init(const struct device *dev)
{
	MX_SPI3_Init();
	struct feedback_data *data = dev->data;
	lowfilter_init(&data->speedfilter, 10.0f);
	return 0;
}
static uint16_t tle5012b_read(const struct device *dev)
{
	unsigned short AngleIn17bits = 0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	_read_angle_reg(&hspi3, &AngleIn17bits);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	return AngleIn17bits;
}
static float feedback_cacle_eangle(const struct device *dev)
{
	struct feedback_data *data = dev->data;
	int32_t raw = tle5012b_read(dev) - 32768;
	raw = amplitude_limiting_filter(&data->filter, raw);
	data->raw = raw;
	if (!data->calibration_state) {
		return 0.0f;
	}
	data->eangle =
		_normalize_angle((raw * 360.0f) / 32768.0f * MOTOR_PAIRS - data->offset + 90.0f);
	return data->eangle;
}
static float feedback_cacle_eomega(const struct device *dev)
{

	struct feedback_data *data = dev->data;
	int16_t delat = 0;
	delat = data->raw - data->last_raw;
	data->last_raw = data->raw;
	float omega = (delat * 6.28f) / 32768.0f / 0.0001f;
	return lowfilter_cale(&data->speedfilter, omega);
}
static float feedback_cacle_odom(const struct device *dev)
{
	return 0.0f;
}
static int feedback_calibration_firstangle(const struct device *dev)
{
	int32_t raw = tle5012b_read(dev) - 32768;
	struct feedback_data *data = dev->data;
	data->offset = 115.0f; //_normalize_angle((raw * 360.0f) / 32768.0f * MOTOR_PAIRS);
	data->calibration_state = 1;
	data->last_raw = data->offset;
	filter_init(&data->filter, 10000, 0, 32768, raw);
	return 0;
}
static short feedback_read_calibration_state(const struct device *dev)
{
	struct feedback_data *data = dev->data;
	return data->calibration_state;
}
static const struct feedback_driver_api driver_feedback = {
	.get_rads = feedback_cacle_eomega,
	.get_eangle = feedback_cacle_eangle,
	.get_rel_odom = feedback_cacle_odom,
	.calibration = feedback_calibration_firstangle,
	.set_rel_odom = NULL,
	.get_calibration_state = feedback_read_calibration_state,
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