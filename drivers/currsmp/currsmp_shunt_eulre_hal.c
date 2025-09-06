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

	uint32_t adc_f_channl_a;
	/** ADC channel data for phase B. */
	uint32_t adc_f_channl_b;
	/** ADC channel data for phase C. */
	uint32_t adc_f_channl_c;
};
/** @brief ADC interrupt service routine.
 *
 * @param[in] dev Current sampling device.
 */
static void inline adc_stm32_isr(const struct device *dev)
{
	const struct currsmp_shunt_stm32_config *cfg = dev->config;
	struct currsmp_shunt_stm32_data *data = dev->data;
	if (LL_ADC_IsActiveFlag_JEOS(cfg->adc)) {

		if (data->regulation_cb) {
			data->regulation_cb(data->regulation_ctx);
		}
		LL_ADC_ClearFlag_JEOS(cfg->adc);
	}
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
	data->adc_channl_c = LL_ADC_INJ_ReadConversionData12(cfg->adc, LL_ADC_INJ_RANK_3);
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

static int currsmp_shunt_stm32_init(const struct device *dev)
{
	const struct currsmp_shunt_stm32_config *cfg = dev->config;

	if (cfg->irq_cfg_func != NULL) {
		cfg->irq_cfg_func();
	}
	LL_ADC_Disable(cfg->adc);
#if CONFIG_SOC_STM32H723XX
	LL_ADC_StartCalibration(cfg->adc, LL_ADC_CALIB_OFFSET, LL_ADC_SINGLE_ENDED);
#endif
	while (LL_ADC_IsCalibrationOnGoing(cfg->adc))
		;
	LL_ADC_Enable(cfg->adc);
	while (LL_ADC_IsActiveFlag_ADRDY(cfg->adc) == 0)
		;
	LL_ADC_EnableIT_JEOS(cfg->adc);
	LL_ADC_INJ_StartConversion(cfg->adc);

	return 0;
}

/** @internal Helper macro to define the first instance with a specific IRQn. */
#define FIRST_WITH_IRQN_INTERNAL(n, irqn)                                                          \
	COND_CODE_1(IS_EQ(irqn, DT_IRQN(DT_INST_PARENT(n))), (n, ), (EMPTY, ))

/** @internal Helper macro to get the first instance with a specific IRQn. */
#define FIRST_WITH_IRQN(n)                                                                         \
	GET_ARG_N(1, LIST_DROP_EMPTY(DT_INST_FOREACH_STATUS_OKAY_VARGS(                            \
			     FIRST_WITH_IRQN_INTERNAL, DT_IRQN(DT_INST_PARENT(n)))))

/** @internal Helper macro to handle IRQs for a specific instance. */
#define HANDLE_IRQS(n, irqn)                                                                       \
	COND_CODE_1(IS_EQ(irqn, DT_IRQN(DT_INST_PARENT(n))),                                       \
		    (adc_stm32_isr(DEVICE_DT_INST_GET(n));), (EMPTY))

/** @internal Helper macro to generate ISR code for a specific instance. */
#define ISR_FUNC(n) UTIL_CAT(adc_stm32_isr_, DT_IRQN(DT_INST_PARENT(n)))

/** @internal Generate ISR code for a specific instance. */
#define GENERATE_ISR_CODE(n)                                                                       \
	static void ISR_FUNC(n)(void)                                                              \
	{                                                                                          \
		DT_INST_FOREACH_STATUS_OKAY_VARGS(HANDLE_IRQS, DT_IRQN(DT_INST_PARENT(n)))         \
	}                                                                                          \
                                                                                                   \
	static void UTIL_CAT(ISR_FUNC(n), _init)(void)                                             \
	{                                                                                          \
		IRQ_CONNECT(DT_IRQN(DT_INST_PARENT(n)), DT_IRQ(DT_INST_PARENT(n), priority),       \
			    ISR_FUNC(n), NULL, 0);                                                 \
		irq_enable(DT_IRQN(DT_INST_PARENT(n)));                                            \
	}

/** @internal Generate ISR code for a specific instance. */
#define GENERATE_ISR(n) COND_CODE_1(IS_EQ(n, FIRST_WITH_IRQN(n)), (GENERATE_ISR_CODE(n)), (EMPTY))

/** @internal Generate device instances. */
DT_INST_FOREACH_STATUS_OKAY(GENERATE_ISR)

/** @internal Helper macro to define the IRQ function for a specific instance. */
#define CURRSMP_SHUNT_STM32_IRQ_FUNC(n)                                                            \
	.irq_cfg_func =                                                                            \
		COND_CODE_1(IS_EQ(n, FIRST_WITH_IRQN(n)), (UTIL_CAT(ISR_FUNC(n), _init)), (NULL)),

/** @brief Device instance definition for STM32 shunt current sampling driver. */
#define CURRSMP_SHUNT_STM32_CURRSMP_SHUNT_INIT(n)                                                  \
                                                                                                   \
	static const struct currsmp_shunt_stm32_config currsmp_shunt_stm32_config_##n = {          \
		.adc = (ADC_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)),                              \
		CURRSMP_SHUNT_STM32_IRQ_FUNC(n)};                                                  \
	static struct currsmp_shunt_stm32_data currsmp_shunt_stm32_data_##n;                       \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, &currsmp_shunt_stm32_init, NULL, &currsmp_shunt_stm32_data_##n,   \
			      &currsmp_shunt_stm32_config_##n, PRE_KERNEL_1,                       \
			      CONFIG_CURRSMP_INIT_PRIORITY, &currsmp_shunt_stm32_driver_api);

/** @brief Generate device instances. */
DT_INST_FOREACH_STATUS_OKAY(CURRSMP_SHUNT_STM32_CURRSMP_SHUNT_INIT)

/** @} */
