#define DT_DRV_COMPAT st_feedback

#include <zephyr/device.h>
#include <zephyr/logging/log.h>

#include "drivers/feedback.h"

#define LOG_LEVEL CONFIG_MOTOR_LIB_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(driver_feedback);

struct feedback_config {
};

struct feedback_data {
};
static const struct feedback_driver_api driver_feedback = {
	.get_rads = NULL,
	.get_eangle = NULL,
	.calibration = NULL,
	.get_rel_odom = NULL,
	.feedback_enable = NULL,
	.set_rel_odom = NULL,
};

static int feedback_init(const struct device *dev)
{
	return 0;
}
#define FEEDBACKINIT(n)                                                                            \
	static const struct feedback_config feedback_config_##n = {};                              \
	static struct feedback_data feedback_data_##n = {};                                        \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, feedback_init, NULL, &feedback_data_##n, &feedback_config_##n,    \
			      PRE_KERNEL_1, CONFIG_FEEDBACK_INIT_PRIORITY, &driver_feedback);

/* Initialize all instances */
DT_INST_FOREACH_STATUS_OKAY(FEEDBACKINIT)
