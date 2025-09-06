# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_MOTOR_SUPER_ABZHALL_400W feedback_abz_hall.c)
zephyr_library_sources_ifdef(CONFIG_ENCODER_TYPE_AS5047 feedback_as5047_copy.c)
zephyr_library_sources_ifdef(CONFIG_ENCODER_TYPE_AS5047 feedback_cube_hal_spi.c)
zephyr_library_sources_ifdef(CONFIG_ENCODER_TYPE_TLE5012B feedback_tle5012b_copy.c)
