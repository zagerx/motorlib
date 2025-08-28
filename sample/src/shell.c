#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
#include <zephyr/drivers/gpio.h>

#include "lib/motor/motor.h"
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
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);
	motor_set_target_speed(motor, 2.5f);
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
	} else if (mode == 5) {
		motor_set_mode(motor, MOTOR_MODE_IDLE);
	}
	return 0;
}

SHELL_CMD_REGISTER(hello, NULL, "Print hello", cmd_hello);
SHELL_CMD_REGISTER(mode, NULL, "Motor Set Mode", cmd_motor_set_mode);
SHELL_CMD_REGISTER(ready, NULL, "Motor Set Ready", cmd_motor_set_ready);
SHELL_CMD_REGISTER(close, NULL, "Motor Set Close", cmd_motor_set_closeloop);
SHELL_CMD_REGISTER(speed, NULL, "Motor Set Speed", cmd_motor_set_speed);
SHELL_CMD_REGISTER(stop, NULL, "Motor Set Stop", cmd_motor_set_disable);
