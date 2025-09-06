# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_BOARD_ZGM_002 currsmp_shunt_stm32.c)
zephyr_library_sources_ifdef(CONFIG_BOARD_ZGM_001 currsmp_shunt_eulre_hal.c)
zephyr_library_sources_ifdef(CONFIG_BOARD_ZGM_001 currsmp_shunt_eulre_hal_adc.c)
zephyr_library_sources_ifdef(CONFIG_BOARD_ZGM_001 currsmp_shunt_eulre_hal_opm.c)
