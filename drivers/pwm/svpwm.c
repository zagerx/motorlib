/*
 * STM32 PWM driver implementation
 *
 * Features:
 * - Center-aligned PWM generation
 * - Master/slave timer synchronization
 * - Dead-time insertion
 * - Complementary channel outputs
 */

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>

#if CONFIG_SOC_STM32H723XX || CONFIG_SOC_STM32G431XX
#include <stm32_ll_tim.h>
#endif

#include "drivers/pwm.h"

#define LOG_LEVEL CONFIG_MOTOR_LIB_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(svpwm);

#define DT_DRV_COMPAT st_stm32_pwm_custom
extern void MX_TIM1_Init(void);
extern void MX_TIM8_Init(void);
extern TIM_HandleTypeDef htim1;

/* PWM configuration structure */
struct pwm_stm32_config {
	TIM_TypeDef *timer;                      /* Timer register base */
	struct stm32_pclken pclken;              /* Clock enable info */
	const struct pinctrl_dev_config *pincfg; /* Pin configuration */
	uint32_t timing_params[3];               /* [0]=dead time (ns), [1]=ARR, [2]=PSC */
	uint32_t slave_enable;                   /* Slave mode flag */
};

/*
 * Stop PWM generation
 */
static void pwm_stm32_stop(const struct device *dev)
{
	const struct pwm_stm32_config *cfg = dev->config;
	uint32_t slave_flag = cfg->slave_enable;

	if (!slave_flag) {
		/* Master timer stop logic */
		LL_TIM_DisableCounter(cfg->timer);
	} else {
		/* Slave timer stop logic */
	}
}

/*
 * Start PWM generation
 */
static void pwm_stm32_start(const struct device *dev)
{
	const struct pwm_stm32_config *cfg = dev->config;
	uint32_t slave_flag = cfg->slave_enable;

	if (!slave_flag) {
		LOG_DBG("master timer");
		HAL_TIM_Base_Start_IT(&htim1);

		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, 30);
		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	} else {
		LOG_DBG("Slave timer");
		LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t)(cfg->timing_params[1] - 30));
		LL_TIM_EnableAllOutputs(cfg->timer);
	}
}

/*
 * Set PWM duty cycles for three phases
 */
static void pwm_stm32_setduties(const struct device *dev, float a, float b, float c)
{
	const struct pwm_stm32_config *cfg = dev->config;
	if (a > 0.9f || b > 0.9f || c > 0.9f) {
		return;
	}
	LL_TIM_OC_SetCompareCH1(cfg->timer, (uint32_t)(cfg->timing_params[1] * a));
	LL_TIM_OC_SetCompareCH2(cfg->timer, (uint32_t)(cfg->timing_params[1] * b));
	LL_TIM_OC_SetCompareCH3(cfg->timer, (uint32_t)(cfg->timing_params[1] * c));

	if (!cfg->slave_enable) {
		LL_TIM_OC_SetCompareCH4(TIM1, (uint32_t)(cfg->timing_params[1] - 30));
	}
}
static void pwm_stm32_setstatus(const struct device *dev, int8_t flag)
{
	const struct pwm_stm32_config *cfg = dev->config;
	LL_TIM_OC_SetCompareCH1(cfg->timer, 0);
	LL_TIM_OC_SetCompareCH2(cfg->timer, 0);
	LL_TIM_OC_SetCompareCH3(cfg->timer, 0);
	if (flag) {
		LL_TIM_CC_EnableChannel(cfg->timer,
					LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 |
						LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N |
						LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);
	} else {
		LL_TIM_CC_DisableChannel(cfg->timer,
					 LL_TIM_CHANNEL_CH1 | LL_TIM_CHANNEL_CH2 |
						 LL_TIM_CHANNEL_CH3 | LL_TIM_CHANNEL_CH1N |
						 LL_TIM_CHANNEL_CH2N | LL_TIM_CHANNEL_CH3N);
	}
}

/* External timer initialization functions */

/*
 * Initialize PWM device
 */
static int pwm_stm32_init(const struct device *dev)
{
	const struct pwm_stm32_config *config = dev->config;
	if (!config->slave_enable) {
		MX_TIM1_Init();
		LOG_DBG("pwm init");
	}
	return 0;
}

/* Device instance macro */
#define PMW_STM32_INIT(n)                                                                          \
	PINCTRL_DT_INST_DEFINE(n);                                                                 \
	static const struct pwm_stm32_config pwm_stm32_config_##n = {                              \
		.timer = (TIM_TypeDef *)DT_REG_ADDR(DT_INST_PARENT(n)),                            \
		.pclken = STM32_CLOCK_INFO(0, DT_INST_PARENT(n)),                                  \
		.timing_params = DT_INST_PROP(n, timing_params),                                   \
		.slave_enable = DT_INST_PROP(n, slave),                                            \
		.pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                                       \
	};                                                                                         \
	static const struct pwm_driver_api pwm_stm32_api_##n = {                                   \
		.start = pwm_stm32_start,                                                          \
		.stop = pwm_stm32_stop,                                                            \
		.set_phase_voltages = pwm_stm32_setduties,                                         \
		.set_phase_state = pwm_stm32_setstatus};                                           \
	DEVICE_DT_INST_DEFINE(n, &pwm_stm32_init, NULL, NULL, &pwm_stm32_config_##n, PRE_KERNEL_1, \
			      CONFIG_PWMX_STM32_INIT_PRIORITY, &pwm_stm32_api_##n);

/* Initialize all instances */
DT_INST_FOREACH_STATUS_OKAY(PMW_STM32_INIT)