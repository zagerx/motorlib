/**
 * @file motor_mode.c
 * @brief BLDC motor control mode implementations
 *
 * Contains state machine implementations for:
 * - Open loop control
 * - Speed control
 * - Position control
 * - Torque control
 *
 * Copyright (c) 2023 Your Company
 * SPDX-License-Identifier: Apache-2.0
 */

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
/* Module logging setup */
LOG_MODULE_REGISTER(motor_mode, LOG_LEVEL_DBG);
fsm_rt_t motor_init_state(fsm_cb_t *obj);

/**
 * @brief Open loop control mode state machine
 * @param obj State machine control block
 * @return fsm_rt_cpl when complete
 *
 * States:
 * 1. ENTER: Initialize open loop mode
 * 2. IDLE: Main operational state
 * 3. EXIT: Cleanup when exiting mode
 */
fsm_rt_t motor_torque_control_mode(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;
	struct motor_data *m_data = motor->data;
	switch (obj->chState) {
	case ENTER:
		m_data->mode = MOTOR_MODE_TORQUE;
		obj->chState = RUNING;
		break;
	case RUNING:
		if (obj->sub_state_machine) {
			DISPATCH_FSM(obj->sub_state_machine);
		}
		break;
	case EXIT:
		break;

	default:
		break;
	}
	return fsm_rt_cpl;
}
fsm_rt_t motor_idle_control_mode(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;
	struct motor_data *m_data = motor->data;
	switch (obj->chState) {
	case ENTER:
		motor_start(obj->p1);
		statemachine_init(obj->sub_state_machine, NULL, motor_init_state, obj->p1, NULL, 0);
		m_data->mode = MOTOR_MODE_IDLE;
		obj->chState = RUNING;
		break;
	case RUNING:
		if (obj->sub_state_machine) {
			DISPATCH_FSM(obj->sub_state_machine);
		}
		break;
	case EXIT:
		break;

	default:
		break;
	}
	return fsm_rt_cpl;
}

/**
 * @brief Speed control mode state machine
 * @param obj State machine control block
 * @return fsm_rt_cpl when complete
 *
 * States:
 * 1. ENTER: Initialize speed control
 * 2. IDLE: Main operational state
 * 3. EXIT: Cleanup when exiting mode
 */
fsm_rt_t motor_init_state(fsm_cb_t *obj);

fsm_rt_t motor_speed_control_mode(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;

	struct motor_data *m_data = motor->data;
	switch (obj->chState) {
	case ENTER:
		m_data->mode = MOTOR_MODE_SPEED;
		statemachine_init(obj->sub_state_machine, NULL, motor_init_state, obj->p1, NULL, 0);
		obj->chState = RUNING;
		break;
	case RUNING:

		if (obj->sub_state_machine) {
			DISPATCH_FSM(obj->sub_state_machine);
		}
		break;
	case EXIT:
		break;

	default:
		break;
	}
	return fsm_rt_cpl;
}

/**
 * @brief Position control mode (stub)
 * @param obj State machine control block
 * @return fsm_rt_cpl when complete
 *
 * TODO: Implement position control logic
 */
fsm_rt_t motor_position_control_mode(fsm_cb_t *obj)
{
	enum {
		RUNING = USER_STATUS,
	};
	const struct device *motor = obj->p1;
	struct motor_data *m_data = motor->data;
	switch (obj->chState) {
	case ENTER:
		m_data->mode = MOTOR_MODE_POSI;
		statemachine_init(obj->sub_state_machine, NULL, motor_init_state, obj->p1, NULL, 0);
		obj->chState = RUNING;
		break;
	case RUNING:
		if (obj->sub_state_machine) {
			DISPATCH_FSM(obj->sub_state_machine);
		}
		break;
	case EXIT:
		break;

	default:
		break;
	}
	return fsm_rt_cpl;
}
