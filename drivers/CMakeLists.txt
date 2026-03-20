# Copyright (c) 2026 Aaron Chan
# SPDX-License-Identifier: Apache-2.0

add_subdirectory_ifdef(CONFIG_SENSOR sensor)
add_subdirectory(CONFIG_RDA5807M rda5807m)