# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources(
	foc.c
	motor_internal.c
	motor_mode.c
	motor.c
	svm.c
	deadtime_comp.c
)
