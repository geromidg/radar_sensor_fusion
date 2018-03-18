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

#ifndef SCHEDULER_PROFILING_H
#define SCHEDULER_PROFILING_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"

/***************************** Public Functions ******************************/

/**
  * @brief Initialize the statistics module.
  * @details This module should be initialized just before the main task
  *          is executed, so that the scheduler's timer is known.
  * @param cycle The cycle time of the system.
  * @param sched_timer_nsec The scheduler's timer in nsec.
  * @return Void.
  */
void InitializeStatistics(f32_t cycle, s32_t sched_timer_nsec);

/**
  * @brief Print the scheduler's statistics to the screen.
  * @warning If the system's cycle time is too low, the buffer might overflow!
  * @return Void.
  */
void PrintStatistics(void);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* SCHEDULER_PROFILING_H */
