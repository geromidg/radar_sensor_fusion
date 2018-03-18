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

#ifndef PLATFORM_PARAMS
#define PLATFORM_PARAMS

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "can_protocol.h"
#include "sensor_interface.h"

/***************************** Macro Definitions *****************************/

/** The time needed for the system to complete a full cycle (run all tasks). */
#define CYCLE_TIME (0.04f)

/**
  * @defgroup external_params External parameters
  * Parameters passed from platform to algorithm. 
  *
  * @{
  */
#define NUM_PREFUSED_OBJ (NUM_RX_OBJS)
#define NUM_FUSED_OBJ    (NUM_TX_OBJS)
/** @} */

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* PLATFORM_PARAMS */
