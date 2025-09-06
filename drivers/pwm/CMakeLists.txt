# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_BOARD_ZGM_002 pwm_stm32.c)
zephyr_library_sources_ifdef(CONFIG_BOARD_ZGM_002 pwm_stm32_cube.c)

zephyr_library_sources_ifdef(CONFIG_BOARD_ZGM_001 pwm_stm32.c)
zephyr_library_sources_ifdef(CONFIG_BOARD_ZGM_001 pwm_stm32g4_cube.c)