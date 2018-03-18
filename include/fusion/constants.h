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

#ifndef CONSTANTS_H
#define CONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "algorithm_types.h"
#include "config.h"

/********************************* Constants *********************************/

/**
  * @defgroup id_constants Constants for ID assigning
  * The IDs each object can take.
  * @todo Remove ID completely from object and replace it with a validity flag (ID does not have a practical use anyway).
  *
  * @{
  */
#define INVALID_ID (0u)
#define MAX_ID     (32u)
/** @} */

/** Priority of each object that is determined by range (zero range means max priority).
  * @todo Calculate from the objects max limits for x and y.
  */
#define MAX_PRIORITY (150.f)

/**
  * @defgroup similarity_constants Constants for the gating functions
  * The invalid and max values for the gating (and similarity) functions.
  * @todo Define a better max similarity value.
  *
  * @{
  */
#define INVALID_SIMILARITY_VALUE (-1.f)
#define INVALID_GATING_VALUE (INVALID_SIMILARITY_VALUE)
#define MAX_SIMILARITY_VALUE (1000.f)
/** @} */

/**
  * @defgroup gating_limits The parameters of the acceptance gate
  * State limit: The minimum limit for two gaussians to be considered similar. Based on the 3-sigma rule (99.7%).
  * Total limit: The sum of all states should be above this limit for a plot to pass the acceptance gate.
  *
  * @{
  */
#define STATE_GATING_VALUE_MIN_LIMIT (0.1f)
/** @} */

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* CONSTANTS_H */
