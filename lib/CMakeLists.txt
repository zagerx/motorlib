# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0
add_subdirectory(motor)
add_subdirectory(foc)

zephyr_library_sources(
	foc.c
	motor.c
)
