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

#ifndef RADAR_UTILS_H
#define RADAR_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "algorithm_types.h"

/***************************** Public Functions ******************************/

/**
  * @brief Calculates the range of an object.
  * @param posX The x position of an object.
  * @param posY The y position of an object.
  * @return The calculated range.
  */
f32_t GetRange(const f32_t posX, const f32_t posY);

/**
  * @brief Calculates the bearing (angle) of an object.
  * @param posX The x position of an object.
  * @param posY The y position of an object.
  * @return The calculated bearing.
  */
f32_t GetBearing(const f32_t posX, const f32_t posY);

/**
  * @brief Calculates the variance in x of an object from cartesian coordinates.
  * @details Firstly, the polar coordinates are derived from the inputs.
  *          Then, the variance is calculated using the 'GetVarXPolar'.
  *          Lastly, if the variance does not pass a specific threshold, then the threshold is returned.
  *          This happens to avoid very low variances that can cause a divergence to the system.
  *          The reason why the polar coordinates are used, is because the radar's specifications are
  *          expressed in polar coordinates (i.e. range, bearing, doppler).
  *         
  * @param posX The x position of an object.
  * @param posY The y position of an object.
  * @param rangeVar The variance of the range.
  * @param bearingVar The variance of the bearing.
  * @param baseVar The base (minimum) variance to be calculated.
  * @return The calculated variance.
  */
f32_t GetVarX(const f32_t posX, const f32_t posY, const f32_t rangeVar, const f32_t bearingVar, const f32_t baseVar);

/**
  * @brief Calculates the variance in y of an object from cartesian coordinates.
  * @see `GetVarXPolar`.
  * @param posX The x position of an object.
  * @param posY The y position of an object.
  * @param rangeVar The variance of the range.
  * @param bearingVar The variance of the bearing.
  * @param baseVar The base (minimum) variance to be calculated.
  * @return The calculated variance.
  */
f32_t GetVarY(const f32_t posX, const f32_t posY, const f32_t rangeVar, const f32_t bearingVar, const f32_t baseVar);

/**
  * @brief Calculates the similarity between two distributions.
  * @details The similarity is the inverse of the dissimilarity value, and is a more intuitive value.
  *          If the two distribution have the same mean (dissimilarity is zero), the maximum (default) value is assumed.
  * @param mean1 The mean value of the first distribution.
  * @param mean2 The mean value of the second distribution.
  * @param variance1 The variance of the first distribution.
  * @param variance2 The variance of the second distribution.
  * @return The calculated similarity value.
  */
f32_t GetSimilarityValue(const f32_t mean1, const f32_t mean2, const f32_t variance1, const f32_t variance2);

/**
  * @brief Calculates the y coordinate of a point, given the x and the limits, using linear interpolation.
  * @param x The x value of the point.
  * @param x1 The lower limit on the x axis.
  * @param x2 The upper limit on the x axis.
  * @param y1 The lower limit on the y axis.
  * @param y2 The upper limit on the y axis.
  * @return The calculated y value.
  */
f32_t GetLinInterpolatedValue(const f32_t x, const f32_t x1, const f32_t x2, const f32_t y1, const f32_t y2);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* RADAR_UTILS_H */
