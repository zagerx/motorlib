#include "pid.h"
#include "s_trajectory_planning.h"
#include "zephyr/device.h"
#include <drivers/currsmp.h>
#include <drivers/feedback.h>
#include <drivers/pwm.h>
#include <lib/motor/motor.h>
#include <lib/motor/motor_internal.h>
#include <lib/foc/foc.h> //TODO
#include <statemachine.h>
#include <stdint.h>
#include <zephyr/logging/log.h>
#include "stm32h7xx_ll_gpio.h"
#include "zephyr/kernel.h"
#include <s_posi_planning.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>

#define LOG_LEVEL CONFIG_MOTOR_LIB_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(motor_state);

#define MOT12_BRK_PIN_NODE DT_NODELABEL(mot12_brk_pin)

fsm_rt_t motor_ready_state(fsm_cb_t *obj);
fsm_rt_t motor_runing_state(fsm_cb_t *obj);
fsm_rt_t motor_stop_state(fsm_cb_t *obj);

fsm_rt_t motor_init_state(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;
	struct motor_data *m_data = motor->data;

	const struct device *foc = ((const struct motor_config *)motor->config)->foc_dev;
	const struct motor_config *mcfg = motor->config;

	switch (obj->chState) {
	case ENTER:
		m_data->statue = MOTOR_STATE_INIT;
		if (m_data->mode == MOTOR_MODE_POSI) {
			foc_posloop_init(foc);
		} else if (m_data->mode == MOTOR_MODE_SPEED) {
			foc_speedloop_init(foc);
		}
		feedback_calibration(mcfg->feedback);
		obj->chState = RUNING;
		break;
	case RUNING:

		break;
	case EXIT:
		break;
	default:
		break;
	}

	return 0;
}
fsm_rt_t motor_ready_state(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;
	struct motor_data *m_data = motor->data;
	const struct device *svpwm = ((const struct motor_config *)motor->config)->pwm;
	switch (obj->chState) {
	case ENTER:
		m_data->statue = MOTOR_STATE_READY;
		svpwm_enable_threephase_channle(svpwm);
		obj->chState = RUNING;
		break;
	case RUNING:
		break;
	case EXIT:
		break;
	default:
		break;
	}

	return 0;
}
fsm_rt_t motor_runing_state(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;
	const struct device *foc = ((const struct motor_config *)motor->config)->foc_dev;
	struct motor_data *m_data = motor->data;

	switch (obj->chState) {
	case ENTER:
		m_data->statue = MOTOR_STATE_CLOSED_LOOP;
	case RUNING:
		if (m_data->mode == MOTOR_MODE_POSI) {
			foc_posloop(foc);
		} else if (m_data->mode == MOTOR_MODE_SPEED) {
			foc_speedloop(foc);
		}

		break;
	case EXIT:
		break;
	default:
		break;
	}

	return 0;
}
fsm_rt_t motor_stop_state(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;
	const struct device *foc = ((const struct motor_config *)motor->config)->foc_dev;
	const struct device *svpwm = ((const struct motor_config *)motor->config)->pwm;
	struct motor_data *m_data = motor->data;

	switch (obj->chState) {
	case ENTER:
		m_data->statue = MOTOR_STATE_STOP;
		foc_posloop_deinit(foc);
		svpwm_disable_threephase_channle(svpwm);
		break;
	case RUNING:
		break;
	case EXIT:
		break;
	default:
		break;
	}

	return 0;
}
fsm_rt_t motor_falut_state(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;
	const struct device *foc = ((const struct motor_config *)motor->config)->foc_dev;
	const struct device *svpwm = ((const struct motor_config *)motor->config)->pwm;
	struct motor_data *m_data = motor->data;

	switch (obj->chState) {
	case ENTER:
		m_data->statue = MOTOR_STATE_FAULT;
		foc_posloop_deinit(foc);
		svpwm_disable_threephase_channle(svpwm);
		break;
	case RUNING:
		break;
	case EXIT:
		break;
	default:
		break;
	}

	return 0;
}
fsm_rt_t motor_idle_state(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;
	struct motor_data *m_data = motor->data;
	switch (obj->chState) {
	case ENTER:
		m_data->statue = MOTOR_STATE_IDLE;
		break;
	case RUNING:
		break;
	case EXIT:
		break;
	default:
		break;
	}

	return 0;
}
