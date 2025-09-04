#include "zephyr/sys/printk.h"
#include <sys/_stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include "stm32h7xx_hal_spi.h"
#define TLE5012_READ_ANGLE_CMD (0x80)
#define TLE5012_TRANS_TIMEOUT  (2)
#define SPI_REG_ADRRESS_ANGLE  (2)

SPI_HandleTypeDef hspi3;

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
void tle5012b_init(void)
{
	MX_SPI3_Init();
}
void *tle5012b_read(void)
{
	unsigned short AngleIn17bits;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	_read_angle_reg(&hspi3, &AngleIn17bits);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
	printk("rawdata %d\r\n", AngleIn17bits);
	return (void *)0;
}