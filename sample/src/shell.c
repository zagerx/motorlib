#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <stdlib.h>
/*------------------ 1. 可选：普通 printk ------------------*/
static int cmd_hello(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc); /* 避免未使用参数警告 */
	ARG_UNUSED(argv);

	shell_print(sh, "Hello Zephyr Shell! argc=%d", argc);
	return 0;
}

/*------------------ 2. 带参数的加法示例 ------------------*/
static int cmd_add(const struct shell *sh, size_t argc, char **argv)
{
	if (argc != 3) {
		shell_error(sh, "Usage: add <num1> <num2>");
		return -EINVAL;
	}

	long a = strtol(argv[1], NULL, 10);
	long b = strtol(argv[2], NULL, 10);

	shell_print(sh, "%ld + %ld = %ld", a, b, a + b);
	return 0;
}

/*------------------ 3. 用子命令形式实现 led ------------------*/
/* 3.1 子命令：led on */
static int cmd_led_on(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "LED turned ON");
	/* 此处可调用 gpio_pin_set(led_dev, LED_PIN, 1); */
	return 0;
}

/* 3.2 子命令：led off */
static int cmd_led_off(const struct shell *sh, size_t argc, char **argv)
{
	ARG_UNUSED(argc);
	ARG_UNUSED(argv);

	shell_print(sh, "LED turned OFF");
	/* gpio_pin_set(led_dev, LED_PIN, 0); */
	return 0;
}

/* 3.3 创建 led 的子命令表 */
SHELL_STATIC_SUBCMD_SET_CREATE(led_cmds, SHELL_CMD(on, NULL, "Turn LED on", cmd_led_on),
			       SHELL_CMD(off, NULL, "Turn LED off", cmd_led_off),
			       SHELL_SUBCMD_SET_END);

/* 3.4 注册顶层 led 命令 */
SHELL_CMD_REGISTER(led, &led_cmds, "LED control commands", NULL);

/*------------------ 4. 注册顶层普通命令 ------------------*/
SHELL_CMD_REGISTER(hello, NULL, "Print hello", cmd_hello);
SHELL_CMD_REGISTER(add, NULL, "Add two integers", cmd_add);
