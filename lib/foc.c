/*
 * Field Oriented Control (FOC) implementation
 *
 * Implements FOC control algorithms for BLDC/PMSM motors
 * Features:
 * - Current loop regulation
 * - Position/speed control
 * - Open loop control
 */

#include "zephyr/device.h"
#include <sys/_intsup.h>
#include <zephyr/logging/log.h>

#include <lib/foc/foc.h>
#include <lib/foc/focutils.h>
#include <lib/motor/motor_Parameter.h>

#include "filter.h"
#include "pid.h"
#include "lib/foc/svm.h"
#include "lib/foc/deadtime_comp.h"

LOG_MODULE_REGISTER(foc, LOG_LEVEL_DBG);

extern void modulation_manager_init(modulation_ctrl_t *ctrl, float max_modulation, float dead_time,
				    float fsw);
#define POS_PID_LIMIT_MAX (2000.0f)
int foc_currloop_init(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	pid_init(&(f_data->id_pid), TOQREMODE_D_KP, TOQREMODE_D_KI, TOQREMODE_D_KC,
		 TOQREMODE_D_MAX_LIMIT, TOQREMODE_D_MIN_LIMIT);
	pid_init(&(f_data->iq_pid), TOQREMODE_Q_KP, TOQREMODE_Q_KI, TOQREMODE_Q_KC,
		 TOQREMODE_Q_MAX_LIMIT, TOQREMODE_Q_MIN_LIMIT);
	f_data->id_ref = 0.0f;
	f_data->iq_ref = 0.0f;
	return 0;
}
int foc_currloop(const struct device *dev)
{

#if defined(CONFIG_MOTOR_DEBUG_ENCODERMODE) || defined(CONFIG_MOTOR_DEBUG_TORQUE_PID)

#else
	struct foc_data *f_data = dev->data;
	float cur_iq = f_data->i_q;
	f_data->id_ref = 0.0f;
	f_data->iq_ref = pid_contrl(&f_data->iq_pid, f_data->iq_ref, cur_iq);
#endif

	return 0;
}
int foc_currloop_update_idq_Ref(const struct device *dev, float d_ref, float q_ref)
{
	struct foc_data *f_data = dev->data;
	f_data->id_ref = d_ref;
	f_data->iq_ref = q_ref;
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
int speedloop_init(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	pid_init(&(f_data->id_pid), TOQREMODE_D_KP, TOQREMODE_D_KI, TOQREMODE_D_KC,
		 TOQREMODE_D_MAX_LIMIT, TOQREMODE_D_MIN_LIMIT);
	pid_init(&(f_data->iq_pid), TOQREMODE_Q_KP, TOQREMODE_Q_KI, TOQREMODE_Q_KC,
		 TOQREMODE_Q_MAX_LIMIT, TOQREMODE_Q_MIN_LIMIT);
	pid_init(&(f_data->speed_pid), SPEEDMODE_SPEED_KP, SPEEDMODE_SPEED_KI, SPEEDMODE_SPEED_KC,
		 SPEEDMODE_SPEED_MAX_LIMIT, SPEEDMODE_SPEED_MIN_LIMIT);
#if defined(CONFIG_SPEEDLOOP_S_SPEEDPLAN)
	s_type_interpolation_init((void *)&f_data->s_speed_ph, SPEEDPLAN_MAX_A, SPEEDPLAN_MAX_JA,
				  SPEEDPLAN_MIN_A, SPEEDPLAN_MIN_JA);
#endif
	return 0;
}
int speedloop_deinit(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	f_data->iq_ref = 0.0f;
	f_data->speed_ref = 0.0f;
	pid_reset(&(f_data->speed_pid));
	return 0;
}
int speedloop(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	float cur_speed = f_data->speed_real;
#if defined(CONFIG_SPEEDLOOP_S_SPEEDPLAN)
	f_data->speed_ref = s_velocity_actory(&(f_data->s_speed_ph)) * 60.0F;
#endif
	f_data->id_ref = 0.0f;
	f_data->iq_ref = pid_contrl(&f_data->speed_pid, f_data->speed_ref, cur_speed);
	return 0;
}
int posloop_init(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	SPosPlanner *planner = &(f_data->s_pos_ph);
	pid_init(&(f_data->id_pid), TOQREMODE_D_KP, TOQREMODE_D_KI, TOQREMODE_D_KC,
		 TOQREMODE_D_MAX_LIMIT, TOQREMODE_D_MIN_LIMIT);
	pid_init(&(f_data->iq_pid), TOQREMODE_Q_KP, TOQREMODE_Q_KI, TOQREMODE_Q_KC,
		 TOQREMODE_Q_MAX_LIMIT, TOQREMODE_Q_MIN_LIMIT);
	pid_init(&(f_data->speed_pid), SPEEDMODE_SPEED_KP, SPEEDMODE_SPEED_KI, SPEEDMODE_SPEED_KC,
		 SPEEDMODE_SPEED_MAX_LIMIT, SPEEDMODE_SPEED_MIN_LIMIT);
	pid_init(&(f_data->pos_pid), POSMODE_POS_KP, POSMODE_POS_KI, POSMODE_POS_KC,
		 POSMODE_POS_MAX_LIMIT, POSMODE_POS_MIN_LIMIT);
	s_pos_planner_init(planner, POSPLAN_MAX_SPEED, POSPLAN_MAX_ACC, POSPLAN_MAX_JACC);
	return 0;
}
int posloop(const struct device *dev)
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
int posloop_deinit(const struct device *dev)
{
	struct foc_data *f_data = dev->data;
	f_data->iq_ref = 0.0f;
	f_data->speed_ref = 0.0f;
	pid_reset(&(f_data->speed_pid));
	pid_reset(&(f_data->id_pid));
	pid_reset(&(f_data->iq_pid));
	return 0;
}

/*
 * Initialize FOC device
 * Returns: 0 on success
 */
int foc_init(const struct device *dev)
{
	const struct foc_data *data = dev->data;
	modulation_manager_init((modulation_ctrl_t *)&(data->modulation), 0.95f, 650e-9f, 10000);
	return 0;
}

/* Device instance macro */