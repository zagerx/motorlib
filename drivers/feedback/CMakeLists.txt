# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

#zephyr_library_sources(stm32_abz.c)
#zephyr_library_sources(stm32_hall.c)
zephyr_library_sources(stm32_abz_hall.c)


if(CONFIG_ENCODER_TYPE_ABS)
	zephyr_library_sources(stm32_as5047.c)
endif()
