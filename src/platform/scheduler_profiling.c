/*
 * Copyright (C) 2016 Dimitris Geromichalos
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

/******************************** Inclusions *********************************/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>

#include "scheduler_profiling.h"

/***************************** Static Variables ******************************/

static struct timespec last_statistics_timer;

static u8_t is_first_cycle = 1u;

static u64_t number_of_calls;

static f32_t cycle_time;
static f32_t cur_error;
static f32_t avg_error = 0.f;
static f32_t min_error;
static f32_t max_error = -1.f;

/************************ Static Function Prototypes *************************/

/**
  * @brief Update the errors according to the time delta.
  * @param time_delta The time difference between the current and previous cycle.
  * @return Void.
  */
static void updateStatistics(s32_t time_delta);

/***************************** Static Functions ******************************/

void updateStatistics(s32_t time_delta)
{
    if (is_first_cycle)
    {
        is_first_cycle = 0;
        return;
    }

    cur_error = cycle_time - time_delta;
    cur_error = (cur_error >= 0) ? cur_error : -cur_error;

    avg_error = ((avg_error * number_of_calls) + cur_error) / (number_of_calls + 1);
    number_of_calls++;

    if (cur_error < min_error)
        min_error = cur_error;

    if (cur_error > max_error)
        max_error = cur_error;
}

/***************************** Public Functions ******************************/

void InitializeStatistics(f32_t cycle, s32_t sched_timer_nsec)
{
    cycle_time = cycle;

    min_error = cycle_time;
    last_statistics_timer.tv_nsec = sched_timer_nsec;
}

void PrintStatistics(void)
{
    struct timespec current_t;
    s32_t time_delta;

    clock_gettime(CLOCK_MONOTONIC, &current_t);

    time_delta = current_t.tv_nsec - last_statistics_timer.tv_nsec;
    time_delta = (time_delta >= 0) ? time_delta : (time_delta + 1000000000u);  /* FIXME: Use NSEC_PER_SEC */
    updateStatistics(time_delta);

    last_statistics_timer.tv_nsec = current_t.tv_nsec;

    printf("time=%.6f s , cur_error=%.1f us , avg_error=%.1f us , min_error=%.1f us , max_error=%.1f us\n",
        current_t.tv_sec + (current_t.tv_nsec / (f32_t)1000000000u),
        cur_error / 1000.f,
        avg_error / 1000.f,
        min_error / 1000.f,
        max_error / 1000.f);
}
