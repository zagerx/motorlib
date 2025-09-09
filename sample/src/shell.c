#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>

#include "lib/motor/motor.h"
#include "zephyr/posix/sys/stat.h"
const struct device *motor = DEVICE_DT_GET(DT_NODELABEL(motor0));

static int cmd_hello(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "Hello Zephyr Shell! argc=%d", argc);
	return 0;
}

static int cmd_motor_set_ready(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	const struct gpio_dt_spec mot12_brk = GPIO_DT_SPEC_GET(DT_NODELABEL(mot12_brk_pin), gpios);
	gpio_pin_set_dt(&mot12_brk, 1);
	motor_set_state(motor, MOTOR_STATE_READY);
	return 0;
}
// static int cmd_motor_set_aligh(const struct shell *sh, size_t argc, char **argv)
// {
// 	ARG_UNUSED(argc);
// 	ARG_UNUSED(argv);
// 	motor_set_state(motor, MOTOR_STATE_ALIGN);
// 	return 0;
// }
static int cmd_motor_set_disable(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	motor_set_state(motor, MOTOR_STATE_STOP);
	return 0;
}
static int cmd_motor_set_closeloop(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	motor_set_state(motor, MOTOR_STATE_CLOSED_LOOP);
	return 0;
}
static int cmd_motor_set_speed(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: add <num1> <num2>");
		return -EINVAL;
	}
	long temp = strtol(argv[1], NULL, 0);
	float speed = (float)temp / 1000.0f;
	motor_set_target_speed(motor, speed);
	motor_set_state(motor, MOTOR_STATE_CLOSED_LOOP);
	return 0;
}

static int cmd_motor_set_mode(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: add <num1> <num2>");
		return -EINVAL;
	}
	long mode = strtol(argv[1], NULL, 0);
	if (mode == 1) {
		motor_set_mode(motor, MOTOR_MODE_SPEED);
	} else if (mode == 2) {
		motor_set_mode(motor, MOTOR_MODE_POSI);
	} else if (mode == 3) {
		motor_set_mode(motor, MOTOR_MODE_TORQUE);
	} else if (mode == 5) {
		motor_set_mode(motor, MOTOR_MODE_IDLE);
	}
	return 0;
}
static int cmd_motor_set_toque(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: add <num1> <num2>");
		return -EINVAL;
	}
	long d_ref = strtol(argv[1], NULL, 0);
	long q_ref = strtol(argv[2], NULL, 0);
	float d_ref_float = (float)d_ref / 1000.0f;
	float q_ref_float = (float)q_ref / 1000.0f;
	motor_set_torque(motor, d_ref_float, q_ref_float);
	motor_set_state(motor, MOTOR_STATE_CLOSED_LOOP);
	return 0;
}

static int cmd_motor_set_posi(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: add <num1> <num2>");
		return -EINVAL;
	}
	long mode = strtol(argv[1], NULL, 0);
	if (mode == 1) {
		motor_clear_realodom(motor, 0.0f);
		motor_set_target_position(motor, 0.0f, -1500.0f, 5.0f);
	} else if (mode == 2) {
		motor_set_target_position(motor, -1500.0f, 0.0f, 5.0f);
	}
	motor_set_state(motor, MOTOR_STATE_CLOSED_LOOP);
	return 0;
}
static int cmd_motor_get_modestate(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 2) {
		shell_error(sh, "Usage: add <num1> <num2>");
		return -EINVAL;
	}
	long state = strtol(argv[1], NULL, 0);
	if (state == 1) {
		if (motor_get_state(motor) == MOTOR_STATE_READY) {
			shell_print(sh, "motor_ready_state");
		} else if (motor_get_state(motor) == MOTOR_STATE_CLOSED_LOOP) {
			shell_print(sh, "motor_runing_state");
		} else if (motor_get_state(motor) == MOTOR_STATE_STOP) {
			shell_print(sh, "motor_stop_state");
		} else if (motor_get_state(motor) == MOTOR_STATE_IDLE) {
			shell_print(sh, "motor_idle_state");
		} else if (motor_get_state(motor) == MOTOR_STATE_FAULT) {
			shell_print(sh, " motor_falut_state");
		} else if (motor_get_state(motor) == MOTOR_STATE_ALIGN) {
			shell_print(sh, " motor_aligh_state");
		}
	} else if (state == 2) {
		if (motor_get_mode(motor) == MOTOR_MODE_TORQUE) {
			shell_print(sh, " motor_torque_control_mode");
		} else if (motor_get_mode(motor) == MOTOR_MODE_SPEED) {
			shell_print(sh, "motor_speed_control_mode");
		} else if (motor_get_mode(motor) == MOTOR_MODE_POSI) {
			shell_print(sh, "motor_position_control_mode");
		}
	}
}
SHELL_CMD_REGISTER(hello, NULL, "Print hello", cmd_hello);
SHELL_CMD_REGISTER(mode, NULL, "Motor Set Mode", cmd_motor_set_mode);
SHELL_CMD_REGISTER(r, NULL, "Motor Set Ready", cmd_motor_set_ready);
// SHELL_CMD_REGISTER(a, NULL, "Motor Set Close", cmd_motor_set_aligh);
SHELL_CMD_REGISTER(c, NULL, "Motor Set Close", cmd_motor_set_closeloop);
SHELL_CMD_REGISTER(speed, NULL, "Motor Set Speed", cmd_motor_set_speed);
SHELL_CMD_REGISTER(s, NULL, "Motor Set Stop", cmd_motor_set_disable);
SHELL_CMD_REGISTER(p, NULL, "Motor Set Posi", cmd_motor_set_posi);
SHELL_CMD_REGISTER(t, NULL, "Motor Set Toque", cmd_motor_set_toque);
SHELL_CMD_REGISTER(g, NULL, "Motor Get State", cmd_motor_get_modestate);
