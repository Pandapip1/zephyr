# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(clock_event_timer)

target_sources(app PRIVATE src/main.c)

# The driver under test, beside the board's own system timer: its system-timer entry points
# are renamed so that the suite calls them and records its announcements.
set(driver ${ZEPHYR_BASE}/drivers/timer/clock_event_timer.c)
target_sources(app PRIVATE ${driver})
set_source_files_properties(${driver} PROPERTIES COMPILE_DEFINITIONS
  "sys_clock_set_timeout=cet_set_timeout;sys_clock_elapsed=cet_elapsed;\
sys_clock_cycle_get_32=cet_cycle_get_32;sys_clock_cycle_get_64=cet_cycle_get_64;\
sys_clock_announce_locked=cet_announce_locked;\
CONFIG_CLOCK_EVENT_SYSTEM_TIMER_MAX_TIMEOUT_MS=1000")
