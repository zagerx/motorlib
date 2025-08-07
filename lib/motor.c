/**
 * @file motor.c
 * @brief BLDC motor control thread implementation
 *
 * This module implements:
 * 1. Hardware initialization (GPIO/PWM etc)
 * 2. FOC control algorithm
 * 3. Watchdog timer operation
 *
 * Copyright (c) 2023 Your Company
 * SPDX-License-Identifier: Apache-2.0
 */

/* System includes */
#include "pid.h"
#include "stm32h723xx.h"
#include "stm32h7xx_ll_gpio.h"

#include "zephyr/device.h"
#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

/* Local includes */
#include <lib/foc/focutils.h>
#include <lib/motor/motor.h>
#include <lib/foc/foc.h> //TODO
#include <drivers/currsmp.h>
#include <drivers/pwm.h>
#include <drivers/feedback.h>
#include <statemachine.h>
#include <lib/motor/motor_internal.h>
/* Device tree compatibility string */
#define DT_DRV_COMPAT motor_bldc

/* Module logging setup */
LOG_MODULE_REGISTER(motor, LOG_LEVEL_DBG);

/* External FSM state handlers */
extern fsm_rt_t motor_idle_control_mode(fsm_cb_t *obj);
extern fsm_rt_t motor_torque_control_mode(fsm_cb_t *obj);
extern fsm_rt_t motor_speed_control_mode(fsm_cb_t *obj);
extern fsm_rt_t motor_position_control_mode(fsm_cb_t *obj);
extern fsm_rt_t motor_init_state(fsm_cb_t *obj);
extern fsm_rt_t motor_ready_state(fsm_cb_t *obj);
extern fsm_rt_t motor_runing_state(fsm_cb_t *obj);
extern fsm_rt_t motor_stop_state(fsm_cb_t *obj);

/**
 * @brief FOC current regulator callback
 * @param ctx Device context pointer
 *
 * Implements the FOC current control loop:
 * 1. Gets current measurements
 * 2. Performs Park/Inverse Park transforms
 * 3. Generates PWM outputs via SVM
 */
static void foc_curr_regulator(void *ctx)
{
	struct device *dev = (struct device *)ctx;
	struct motor_config *cfg = (struct motor_config *)dev->config;
	struct device *currsmp = (struct device *)cfg->currsmp;

	const struct device *foc = cfg->foc_dev;
	struct foc_data *data = foc->data;
	struct currsmp_curr current_now;

	/* Get current measurements */
	currsmp_get_phase_currents(currsmp, &current_now);
	data->i_a = current_now.i_a;
	data->i_b = current_now.i_b;
	data->i_c = current_now.i_c;
	data->eangle = feedback_get_eangle(cfg->feedback);
	data->pos_real = feedback_get_position(cfg->feedback);
	foc_calculate_speed(foc, feedback_get_rads(cfg->feedback) * 95493.0f * 0.2f);

	/* Generate test signals for open loop */
	float alph, beta, sin_the, cos_the;
	sin_cos_f32(((data->eangle - 90.0f)), &sin_the, &cos_the);

	clarke_f32(current_now.i_a, current_now.i_b, &(data->i_alpha), &(data->i_beta));
	park_f32((data->i_alpha), (data->i_beta), &(data->i_d), &(data->i_q), sin_the, cos_the);

	float d_out, q_out;
	if (motor_get_state(dev) == MOTOR_STATE_CLOSED_LOOP) {
		d_out = pid_contrl((pid_cb_t *)(&data->id_pid), 0.0f, data->i_d);
		q_out = pid_contrl((pid_cb_t *)(&data->iq_pid), data->iq_ref, data->i_q);
		svm_apply_voltage_limiting(foc, &d_out, &q_out, data->bus_vol);
		sin_cos_f32(((data->eangle)), &sin_the, &cos_the);
		inv_park_f32(d_out, q_out, &alph, &beta, sin_the, cos_the);
	} else {
		d_out = 0.0f;
		q_out = 0.00f;
		svm_apply_voltage_limiting(foc, &d_out, &q_out, data->bus_vol);
		sin_cos_f32(((data->eangle)), &sin_the, &cos_the);
		inv_park_f32(d_out, q_out, &alph, &beta, sin_the, cos_the);
	}
	float dabc[3];
	/* Perform SVM modulation */
	foc_modulate(foc, alph, beta, dabc);
	/* Set PWM outputs */
	pwm_set_phase_voltages(cfg->pwm, dabc[0], dabc[1], dabc[2]);
}
void motor_set_mode(const struct device *motor, enum motor_mode mode)
{
	const struct motor_data *m_data = motor->data;
	fsm_cb_t *mfsm = m_data->mode_state_mec;
	if (mode == MOTOR_MODE_TORQUE) {
		TRAN_STATE(mfsm, motor_torque_control_mode);
	} else if (mode == MOTOR_MODE_SPEED) {
		TRAN_STATE(mfsm, motor_speed_control_mode);
	} else if (mode == MOTOR_MODE_POSI) {
		TRAN_STATE(mfsm, motor_position_control_mode);
	}
}
enum motor_mode motor_get_mode(const struct device *motor)
{
	const struct motor_data *mdata = motor->data;
	return mdata->mode;
}
void motor_set_state(const struct device *motor, enum motor_cmd sig)
{
	const struct motor_data *m_data = motor->data;
	fsm_cb_t *sub_sm = m_data->mode_state_mec->sub_state_machine;

	if (sig == MOTOR_CMD_SET_ENABLE) {
		TRAN_STATE(sub_sm, motor_ready_state);
	} else if (sig == MOTOR_CMD_SET_START) {
		TRAN_STATE(sub_sm, motor_runing_state);
	} else if (sig == MOTOR_CMD_SET_DISABLE) {
		TRAN_STATE(sub_sm, motor_stop_state);
	}
}
enum motor_state motor_get_state(const struct device *motor)
{
	const struct motor_data *mdata = motor->data;
	return mdata->statue;
}

void motor_set_target_speed(const struct device *motor, float target)
{
	const struct motor_config *mcfg = motor->config;
	const struct device *devfoc = mcfg->foc_dev;
	foc_update_target_speed(devfoc, target);
}
void motor_set_target_position(const struct device *motor, float start, float target,
			       float total_time)
{
	const struct motor_config *mcfg = motor->config;
	const struct device *devfoc = mcfg->foc_dev;
	foc_update_target_position(devfoc, start, target, total_time);
}
void motor_set_vol(const struct device *motor, float *bus_vol)
{
	const struct motor_config *mcfg = motor->config;
	const struct device *devfoc = mcfg->foc_dev;
	foc_update_vbusvolita_cbuscurr(devfoc, bus_vol[0], bus_vol[1]);
}
float motor_get_current_position(const struct device *motor)
{
	const struct motor_config *mcfg = motor->config;
	const struct device *devfoc = mcfg->foc_dev;
	const struct foc_data *fdata = devfoc->data;
	return fdata->pos_real;
}
void motor_clear_realodom(const struct device *motor, float odom)
{
	const struct motor_config *mcfg = motor->config;
	const struct device *feedback = mcfg->feedback;
	feedback_set_pos(feedback);
}
void motor_get_bus_voltage_current(const struct device *motor, float *bus_vol, float *bus_curr)
{
	struct motor_config *cfg = (struct motor_config *)motor->config;
	struct device *currsmp = (struct device *)cfg->currsmp;
	currsmp_get_bus_voltage_current(currsmp, bus_vol, bus_curr);
}
/**
 * @brief Motor device initialization

 * @param dev Motor device instance
 * @return 0 on success, negative errno on failure
 *
 * Sets up:
 * 1. Current sampling callback
 * 2. Initial FSM state
 */
static int motor_init(const struct device *dev)
{
	const struct motor_config *cfg = dev->config;
	const struct device *currsmp = cfg->currsmp;

	/* Configure current sampling */
	currsmp_configure(currsmp, foc_curr_regulator, (void *)dev);
	return 0;
}

/* Device tree instantiation macros */
#define MOTOR_INIT(n)                                                                              \
	fsm_cb_t control_state_machine_##n;                                                        \
	fsm_cb_t mode_state_machine_##n = {                                                        \
		.sub_state_machine = &control_state_machine_##n,                                   \
		.current_state = motor_idle_control_mode,                                          \
		.p1 = (void *)DEVICE_DT_GET(DT_DRV_INST(n)),                                       \
	};                                                                                         \
	fmm_t bus_vol_fault_##n;                                                                   \
	fmm_t bus_curr_fault_##n;                                                                  \
	static const struct motor_config motor_cfg_##n = {                                         \
		.foc_dev = DEVICE_DT_GET(DT_INST_PHANDLE(n, control_algorithm)),                   \
		.pwm = DEVICE_DT_GET(DT_INST_PHANDLE(n, pwm)),                                     \
		.currsmp = DEVICE_DT_GET(DT_INST_PHANDLE(n, currsmp)),                             \
		.feedback = DEVICE_DT_GET(DT_INST_PHANDLE(n, feedback)),                           \
		.fault = {&bus_vol_fault_##n, &bus_curr_fault_##n}};                               \
	static struct motor_data motor_data_##n = {                                                \
		.mode_state_mec = &mode_state_machine_##n,                                         \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(n, motor_init, NULL, &motor_data_##n, &motor_cfg_##n, POST_KERNEL,   \
			      CONFIG_MOTOR_INIT_PRIORITY, NULL);

/* Create device instances for all enabled nodes */
DT_INST_FOREACH_STATUS_OKAY(MOTOR_INIT)