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

#ifndef FUSION_H
#define FUSION_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "algorithm_types.h"

/***************************** Public Functions ******************************/

/**
  * @brief Initializes the fusion algorithm.
  * @details All the lists are reset and the tracker is initialized.
  * @return Void.
  */ 
void InitializeFusion(void);
    
/**
  * @brief Runs the algorithm for one cycle.
  * @details The three steps of the algo (predict, update, manage) are executed.
  * @return Void.
  */ 
void RunFusion(const PrefusedObject_t* prefusedObjectList, FusedObject_t* fusedObjectList);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* FUSION_H */
