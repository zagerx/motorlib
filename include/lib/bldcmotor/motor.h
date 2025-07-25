#ifndef __MOTOR__H
#define __MOTOR__H
#include "zephyr/device.h"
#include <statemachine.h>
#include <fault_monitoring_module.h>
struct motor_config {
	const struct device *foc_dev; // FOC控制算法设备
	const struct device *currsmp;
	const struct device *pwm;
	const struct device *feedback;
	fsm_cb_t *fsm;
	fmm_t *fault[2];
};

/**
 * @brief FOC电机控制状态枚举
 */
enum motor_state {
	MOTOR_STATE_IDLE = USER_STATUS, // 空闲状态，未使能
	MOTOR_STATE_PARAM_UPDATE,
	MOTOR_STATE_INIT,	 // 初始化状态
	MOTOR_STATE_READY,	 // 就绪态 已使能，准备进入闭环前的状态
	MOTOR_STATE_ALIGN,	 // 电机对齐状态(初始位置校准)
	MOTOR_STATE_OPEN_LOOP,	 // 开环运行状态
	MOTOR_STATE_CLOSED_LOOP, // 闭环运行状态
	MOTOR_STATE_FAULT,	 // 故障状态
	MOTOR_STATE_CALIBRATION, // 校准状态(参数辨识)
	MOTOR_STATE_STOP,	 // 受控停止状态
	MOTOR_STATE_EMERGENCY	 // 紧急停止状态
};

enum motor_mode {
	MOTOR_MODE_SPEED = 1,
	MOTOR_MODE_POSI,
	MOTOR_MODE_TORQUE,
	MOTOR_MODE_SELFCHECK,
	MOTOR_MODE_IDLE
};

enum motor_cmd {
	MOTOR_CMD_SET_SPEED_MODE = USER_SIG, // 进入速度模式
	MOTOR_CMD_SET_POSTION_MODE,
	MOTOR_CMD_SET_TORQUE_MODE,
	MOTOR_CMD_SET_ENABLE,
	MOTOR_CMD_SET_DISABLE,
	MOTOR_CMD_SET_SPEED, // 设置速度
	MOTOR_CMD_SET_START,
	MOTOR_CMD_SET_PIDPARAM,
	MOTOR_CMD_SET_VOLFAULT,
	MOTOR_CMD_SET_IDLE
};
struct motor_data {
	enum motor_mode mode;
	enum motor_state statue;
	enum motor_cmd cmd;
};

enum motor_mode motor_get_mode(const struct device *motor);
enum motor_state motor_get_state(const struct device *motor);
void motor_set_mode(const struct device *motor, enum motor_mode mode);
void motor_set_state(const struct device *motor, enum motor_cmd sig);
float motor_get_current_position(const struct device *motor);
void motor_get_bus_voltage_current(const struct device *motor, float *bus_vol, float *bus_curr);
void motor_set_target_position(const struct device *motor, float target, float start);
void motor_clear_realodom(const struct device *motor, float odom);
void motor_set_target_speed(const struct device *motor, float target);

#endif
