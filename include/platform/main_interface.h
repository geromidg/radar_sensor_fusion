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

#ifndef PLATFORM_INTERFACE_H
#define PLATFORM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"


/***************************** Macro Definitions *****************************/

/** Clamps the value between a min and a max limit. */
#define CLAMP(x,u,o)  (((o)>=(u)) ? (((x)>(o)) ? (o) : (((x)<(u))?(u):(x))) : (((x)>(u)) ? (u) : (((x)<(o))?(o):(x))))

/***************************** Public Functions ******************************/

/**
  * @brief Initialize the platform's system.
  * @details All interfaces (CAN, sensors etc.) as well as all the modules/submodules
  *          are initialized.
  * @return Void.
  */
void Initialize(void);

/**
  * @brief Copy the prefused data from the CAN module to the local buffers.
  * @details Copy the raw data (CAN frames) and convert it to objects, that
  *          can be inputted to the algo module.
  * @todo Generate warning/DTC when not all prefused objects have been received.
  * @return Void.
  */
void CopyPrefusedData(void);

/**
  * @brief Executes one step of the fusion algorithm.
  * @details Calls the algorithm using the previously created prefused object list.
  *          When the execution completes, a fused object list is returned.
  * @return Void.
  */
void ExecuteFusionAlgo(void);

/**
  * @brief Publishes the processed data to the CAN bus.
  * @details Converts the fused objects outputted from the algorithm to CAN frames,
  *          that are passed to the CAN module for transmition.
  * @return Void.
  */
void PublishFusedData(void);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* PLATFORM_INTERFACE_H */
