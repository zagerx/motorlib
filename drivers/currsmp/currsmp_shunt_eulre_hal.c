#include <zephyr/irq.h>
#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/drivers/pinctrl.h>

#if CONFIG_SOC_STM32H723XX || CONFIG_SOC_STM32G431XX
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include "stm32_ll_adc.h"
#include <stm32_ll_gpio.h>
#endif

#include "drivers/currsmp.h"
#include "filter.h"

#define LOG_LEVEL CONFIG_MOTOR_LIB_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(currsmp_shunt_stm32);

#define DT_DRV_COMPAT st_stm32_currsmp_shunt

struct currsmp_shunt_stm32_config {
	/** ADC peripheral instance. */
	ADC_TypeDef *adc;
	void (*irq_cfg_func)(void);
};

/** @brief Data structure for STM32 shunt current sampling driver. */
struct currsmp_shunt_stm32_data {
	/** Regulation callback function. */
	currsmp_regulation_cb_t regulation_cb;
	/** Regulation callback context. */
	void *regulation_ctx;
	/** ADC channel data for phase A. */
	uint32_t adc_channl_a;
	/** ADC channel data for phase B. */
	uint32_t adc_channl_b;
	/** ADC channel data for phase C. */
	uint32_t adc_channl_c;
};
/** @brief ADC interrupt service routine.
 *
 * @param[in] dev Current sampling device.
 */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	static float iabc[3];
	if (hadc->Instance == ADC1) {
		iabc[0] = -((int16_t)(hadc->Instance->JDR1) - 1970) * 0.02197f;
		iabc[2] = -((int16_t)(hadc->Instance->JDR2) - 1980) * 0.02197f;
	}
	LOG_DBG("...");
}

static void currsmp_shunt_stm32_configure(const struct device *dev,
					  currsmp_regulation_cb_t regulation_cb, void *ctx)
{
	struct currsmp_shunt_stm32_data *data = dev->data;

	data->regulation_cb = regulation_cb;
	data->regulation_ctx = ctx;
}
static void currsmp_shunt_stm32_get_currents(const struct device *dev, struct currsmp_curr *curr)
{
	const struct currsmp_shunt_stm32_config *cfg = dev->config;
	struct currsmp_shunt_stm32_data *data = dev->data;

	data->adc_channl_a = LL_ADC_INJ_ReadConversionData12(cfg->adc, LL_ADC_INJ_RANK_1);
	data->adc_channl_b = LL_ADC_INJ_ReadConversionData12(cfg->adc, LL_ADC_INJ_RANK_2);
	// data->adc_channl_c = LL_ADC_INJ_ReadConversionData12(cfg->adc, LL_ADC_INJ_RANK_3);
}

static void currsmp_shunt_stm32_get_bus_vol_curr(const struct device *dev, float *bus_vol,
						 float *bus_curr)
{
	*bus_vol = 24.0f;
	return;
}

/** @brief STM32 Shunt Current Sampling Driver API. */
static const struct currsmp_driver_api currsmp_shunt_stm32_driver_api = {
	.configure = currsmp_shunt_stm32_configure,
	.get_currents = currsmp_shunt_stm32_get_currents,
	.get_bus_volcurr = currsmp_shunt_stm32_get_bus_vol_curr,
};

extern void MX_ADC1_Init(void);
extern ADC_HandleTypeDef hadc1;
extern void MX_OPAMP1_Init(void);
extern void MX_OPAMP2_Init(void);
extern void MX_OPAMP3_Init(void);
static int currsmp_shunt_stm32_init(const struct device *dev)
{
	const struct currsmp_shunt_stm32_config *cfg = dev->config;
	MX_ADC1_Init();
	MX_OPAMP1_Init();
	MX_OPAMP2_Init();
	MX_OPAMP3_Init();
#if CONFIG_SOC_STM32G431XX
	LL_ADC_StartCalibration(cfg->adc, LL_ADC_SINGLE_ENDED);
#endif
	HAL_ADC_Start(&hadc1);
	HAL_ADCEx_InjectedStart_IT(&hadc1);
	return 0;
}

#define CURRSMP_SHUNT_STM32_CURRSMP_SHUNT_INIT(n)                                                  \
                                                                                                   \
	static const struct currsmp_shunt_stm32_config currsmp_shunt_stm32_config_##n = {          \
		.adc = (ADC_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)),                              \
	};                                                                                         \
	static struct currsmp_shunt_stm32_data currsmp_shunt_stm32_data_##n;                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &currsmp_shunt_stm32_init, NULL, &currsmp_shunt_stm32_data_##n,   \
			      &currsmp_shunt_stm32_config_##n, PRE_KERNEL_1,                       \
			      CONFIG_CURRSMP_INIT_PRIORITY, &currsmp_shunt_stm32_driver_api);

/** @brief Generate device instances. */
DT_INST_FOREACH_STATUS_OKAY(CURRSMP_SHUNT_STM32_CURRSMP_SHUNT_INIT)

/** @} */
