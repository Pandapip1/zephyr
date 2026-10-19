# Copyright (c) 2020 Intel Corporation.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(timing.c)
zephyr_library_sources_ifdef(CONFIG_TIMING_COUNTER timing_counter.c)
