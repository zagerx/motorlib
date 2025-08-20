# Copyright (c) 2021 Teslabs Engineering S.L.
# SPDX-License-Identifier: Apache-2.0
if(CONFIG_MOTORLIB_ENABLE)
add_subdirectory(drivers)
add_subdirectory(lib)
zephyr_include_directories(
    include
)
endif()