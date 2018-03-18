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

#ifndef ALGORITHM_INTERFACE_H
#define ALGORITHM_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "base_types.h"

#include "algorithm_types.h"

/***************************** Public Functions ******************************/

/**
  * @brief Initializes the algorithm.
  * @details Initializes the module's algorithms and registers the input sensors.
  * @return Void.
  */
void InitializeAlgorithm(void);

/**
  * @brief Runs the algorithm for one cycle.
  * @details The input list is converted from platform type to native.
  *          Then, the algorithm is executed for one cycle and the new fused object
  *          list is received. The fused object list is converted from native
  *          back to platform type and is returned.
  * @param pInputObjectList The input object list from an external module.
  * @param pOutputObjectList The output object list from an external module.
  * @return Void.
  */
void RunAlgorithm(const BaseObject_t* pInputObjectList, BaseObject_t* pOutputObjectList);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* ALGORITHM_INTERFACE_H */
