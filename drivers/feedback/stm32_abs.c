/*
 * STM32 ABZ Hall Sensor Driver
 *
 * Features:
 * - ABZ encoder interface
 * - Hall sensor position detection
 * - Speed calculation
 */

#include "zephyr/device.h"
#include "stm32h7xx_ll_tim.h"
#include <sys/_intsup.h>
#include <sys/_stdint.h>
#define DT_DRV_COMPAT ams_as5047

#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>
#include <drivers/feedback.h>
#include <stm32_ll_tim.h>
#include <lib/motor/motor_Parameter.h>

#define LOG_LEVEL CONFIG_MOTOR_LIB_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(stm32_abs);

#define ABZ_ENCODER_LINES_X2 (ABZ_ENCODER_LINES * 2)

static inline float _normalize_angle(float angle)
{
	float a = fmodf(angle, 360.0f);
	return a >= 0 ? a : (a + 360.0f);
}
/* Driver configuration structure */
struct stm32_abs_config {
};

struct stm32_abs_data {
};
static void hall_angleupdate(const void *obj, uint8_t cur_sect)
{
}
/* API implementation */
static float abz_stm32_get_eangle(const struct device *dev)
{
	return 0.0f;
}
/* Driver API structure */
static const struct feedback_driver_api driver_feedback = {
	.get_rads = NULL,
	.get_eangle = NULL,
	.calibration = NULL,
	.get_rel_odom = NULL,
	.feedback_enable = NULL,
	.set_rel_odom = NULL,
};
/*
 * Main initialization function
 */
static int stm32_abs_init(const struct device *dev)
{
	return 0;
}

/* Device instance macro */
#define STM32_ABS_INIT(n)                                                                          \
	static const struct stm32_abs_config stm32_abs_cfg_##n = {};                               \
	static struct stm32_abs_data stm32_abs_data_##n = {};                                      \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, stm32_abz_hall_init, NULL, &stm32_abs_data_##n,                   \
			      &stm32_abs_cfg_##n, PRE_KERNEL_1, CONFIG_FEEDBACK_INIT_PRIORITY,     \
			      &driver_feedback);

/* Initialize all instances */
DT_INST_FOREACH_STATUS_OKAY(STM32_ABS_INIT)
