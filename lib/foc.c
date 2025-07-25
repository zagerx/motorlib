/*
 * Field Oriented Control (FOC) implementation
 *
 * Implements FOC control algorithms for BLDC/PMSM motors
 * Features:
 * - Current loop regulation
 * - Position/speed control
 * - Open loop control
 */

#include "filter.h"
#include "pid.h"
#include "lib/focutils/svm/svm.h"
#include "lib/focutils/utils/deadtime_comp.h"
#include "zephyr/device.h"

#include <sys/_intsup.h>
#include <zephyr/logging/log.h>
#include <lib/foc/foc.h>
#include <lib/focutils/utils/focutils.h>

LOG_MODULE_REGISTER(foc, LOG_LEVEL_DBG);

#define DT_DRV_COMPAT foc_ctrl_algo
extern void modulation_manager_init(modulation_ctrl_t *ctrl, float max_modulation, float dead_time,
				    float fsw);
#define POS_PID_LIMIT_MAX (2000.0f)
int foc_currloop_init(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	pid_init(&(f_data->id_pid), 0.08f, 0.006f, 0.5f, 12.0f, -12.0f);
	pid_init(&(f_data->iq_pid), 0.08f, 0.006f, 0.5f, 12.0f, -12.0f);
	f_data->id_ref = 0.0f;
	f_data->iq_ref = 0.0f;
	return 0;
}
int foc_currloop(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	float cur_iq = f_data->i_q;
	f_data->id_ref = 0.0f;
	f_data->iq_ref = pid_contrl(&f_data->iq_pid, f_data->iq_ref, cur_iq);
	return 0;
}
int foc_currloop_deinit(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	f_data->iq_ref = 0.0f;
	f_data->id_ref = 0.0f;
	pid_reset(&(f_data->id_pid));
	pid_reset(&(f_data->iq_pid));
	return 0;
}
int foc_speedloop_init(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	pid_init(&(f_data->id_pid), 0.08f, 0.006f, 0.5f, 12.0f, -12.0f);
	pid_init(&(f_data->iq_pid), 0.08f, 0.006f, 0.5f, 12.0f, -12.0f);
	pid_init(&(f_data->speed_pid), 0.0125f, 0.0083f, 0.5f, 48.0f, -48.0f);
	s_type_interpolation_init((void *)&f_data->s_speed_ph, 100.00f, 300.00f, 0.00f, 0.00f);
	return 0;
}
int foc_speedloop_deinit(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	f_data->iq_ref = 0.0f;
	f_data->speed_ref = 0.0f;
	pid_reset(&(f_data->speed_pid));
	return 0;
}
int foc_speedloop(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	float cur_speed = f_data->speed_real;

	f_data->speed_ref = s_velocity_actory(&(f_data->s_speed_ph)) * 60.0F;
	f_data->id_ref = 0.0f;
	f_data->iq_ref = pid_contrl(&f_data->speed_pid, f_data->speed_ref, cur_speed);
	return 0;
}
int foc_posloop_init(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	SPosPlanner *planner = &(f_data->s_pos_ph);
	pid_init(&(f_data->id_pid), 0.08f, 0.006f, 0.5f, 12.0f, -12.0f);
	pid_init(&(f_data->iq_pid), 0.08f, 0.006f, 0.5f, 12.0f, -12.0f);
	pid_init(&(f_data->speed_pid), 0.0125f, 0.0083f, 0.5f, 48.0f, -48.0f);
	pid_init(&(f_data->pos_pid), 10.0f, 0.0001f, 0.50f, POS_PID_LIMIT_MAX, -POS_PID_LIMIT_MAX);
	s_pos_planner_init(planner, 1400.0f, 3000.0f, 15000.0f);
	return 0;
}
int foc_posloop(const struct device *dev)
{
	float cur_speed, cur_pos;
	struct foc_data *f_data = dev->data;
	SPosPlanner *planner = &(f_data->s_pos_ph);
	cur_speed = f_data->speed_real;
	cur_pos = f_data->pos_real;
	f_data->pos_ref = s_pos_update(planner, 0.001f);
	f_data->speed_ref = pid_contrl(&f_data->pos_pid, f_data->pos_ref, cur_pos);
	f_data->id_ref = 0.0f;
	f_data->iq_ref = pid_contrl(&f_data->speed_pid, f_data->speed_ref, cur_speed);
	return 0;
}
int foc_posloop_deinit(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	f_data->iq_ref = 0.0f;
	f_data->speed_ref = 0.0f;
	pid_reset(&(f_data->speed_pid));
	pid_reset(&(f_data->id_pid));
	pid_reset(&(f_data->iq_pid));
	return 0;
}

float foc_calculate_speed(const struct device *dev, float cur_speed)
{
	struct foc_data *data = dev->data;
	float speed;
	speed = lowfilter_cale((lowfilter_t *)&data->speed_filter, cur_speed);
	data->speed_real = speed;
	return speed;
}

/*
 * Initialize FOC device
 * Returns: 0 on success
 */
static int foc_init(const struct device *dev)
{
	const struct foc_data *data = dev->data;
	lowfilter_init((lowfilter_t *)&(data->speed_filter), 10.0f);
	modulation_manager_init((modulation_ctrl_t *)&(data->modulation), 0.95f, 650e-9f, 10000);
	return 0;
}

/* Device instance macro */
#define FOC_INIT(n)                                                                                \
	static const struct foc_api foc_api_##n = {                                                \
		.posloop = foc_posloop,                                                            \
	};                                                                                         \
	static struct foc_data foc_data_##n;                                                       \
	static const struct foc_config foc_cfg_##n = {                                             \
		.modulate = svm_set,                                                               \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, &foc_init, NULL, &foc_data_##n, &foc_cfg_##n, POST_KERNEL,        \
			      CONFIG_FOC_INIT_PRIORITY, &foc_api_##n);

/* Initialize all FOC instances */
DT_INST_FOREACH_STATUS_OKAY(FOC_INIT)