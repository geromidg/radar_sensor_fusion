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

 /**
  * Most utilities are aimed at processing Gaussian distributions
  * and radar related attributes.
  */

/******************************** Inclusions *********************************/

#include <math.h>

#include "constants.h"

#include "radar_utils.h"

/************************ Static Function Prototypes *************************/

/**
  * @brief Calculates the variance in x of an object from polar coordinates.
  * @details The formula used is derived by applied the rotation matrix to the covariance matrix.
  *          Specifically, P(cart) = R * P(polar) * R'.
  * @param range The range of an object.
  * @param bearing The bearing (angle) of an object.
  * @param rangeVar The variance of the range.
  * @param bearingVar The variance of the bearing.
  * @return The calculated variance.
  */
static f32_t GetVarXPolar(const f32_t range, const f32_t bearing, const f32_t rangeVar, const f32_t bearingVar);

/**
  * @brief Calculates the variance in y of an object from polar coordinates.
  * @see `GetVarXPolar`.
  * @param range The range of an object.
  * @param bearing The bearing (angle) of an object.
  * @param rangeVar The variance of the range.
  * @param bearingVar The variance of the bearing.
  * @return The calculated variance.
  */
static f32_t GetVarYPolar(const f32_t range, const f32_t bearing, const f32_t rangeVar, const f32_t bearingVar);

/**
  * @brief Calculates the dissimilarity between two distributions.
  * @details The dissimilarity is calculated as the squared Mahalanobis distance.
  *          Valid values are positive, since the variances are always positive.
  *          Zero variances return an invalid (default) value to avoid divison by zero.
  * @param mean1 The mean value of the first distribution.
  * @param mean2 The mean value of the second distribution.
  * @param variance1 The variance of the first distribution.
  * @param variance2 The variance of the second distribution.
  * @return The calculated dissimilarity value.
  */
static f32_t GetDissimilarityValue(const f32_t mean1, const f32_t mean2, const f32_t variance1, const f32_t variance2);

/***************************** Static Functions ******************************/

f32_t GetVarXPolar(const f32_t range, const f32_t bearing, const f32_t rangeVar, const f32_t bearingVar)
{
    f32_t cosPhi2 = (f32_t)(cos(bearing) * cos(bearing));
    f32_t sinPhi2 = (f32_t)(sin(bearing) * sin(bearing));

    return ((rangeVar * cosPhi2) + (range * range * bearingVar * sinPhi2));
}

f32_t GetVarYPolar(const f32_t range, const f32_t bearing, const f32_t rangeVar, const f32_t bearingVar)
{
    f32_t cosPhi2 = (f32_t)(cos(bearing) * cos(bearing));
    f32_t sinPhi2 = (f32_t)(sin(bearing) * sin(bearing));

    return ((rangeVar * sinPhi2) + (range * range * bearingVar * cosPhi2));
}

f32_t GetDissimilarityValue(const f32_t mean1, const f32_t mean2, const f32_t variance1, const f32_t variance2)
{
    f32_t dissimilarity = INVALID_SIMILARITY_VALUE;
    f32_t variance12 = variance1 + variance2;

    if (variance12 != 0.f)
    {
        dissimilarity = ((mean1 - mean2) * (mean1 - mean2)) / variance12;
    }

    return dissimilarity;
}

/***************************** Public Functions ******************************/

f32_t GetRange(const f32_t posX, const f32_t posY)
{
    return sqrt(((posX)*(posX)) + ((posY)*(posY)));
}

f32_t GetBearing(const f32_t posX, const f32_t posY)
{
    return atan2(posY, posX);
}

f32_t GetVarX(const f32_t posX, const f32_t posY, const f32_t rangeVar, const f32_t bearingVar, const f32_t baseVar)
{
    f32_t range = GetRange(posX, posY);
    f32_t bearing = GetBearing(posX, posY);
    f32_t varX = GetVarXPolar(range, bearing, rangeVar, bearingVar);

    return fmax(baseVar, varX);
}

f32_t GetVarY(const f32_t posX, const f32_t posY, const f32_t rangeVar, const f32_t bearingVar, const f32_t baseVar)
{
    f32_t range = GetRange(posX, posY);
    f32_t bearing = GetBearing(posX, posY);
    f32_t varY = GetVarYPolar(range, bearing, rangeVar, bearingVar);

    return fmax(baseVar, varY);
}

f32_t GetSimilarityValue(const f32_t mean1, const f32_t mean2, const f32_t variance1, const f32_t variance2)
{
    f32_t similarity;
    f32_t dissimilarity = GetDissimilarityValue(mean1, mean2, variance1, variance2);

    if (dissimilarity == 0.f)
    {
        similarity = MAX_SIMILARITY_VALUE;
    }
    else
    {
        similarity = 1.f / dissimilarity;
    }

    return similarity;
}

f32_t GetLinInterpolatedValue(const f32_t x, const f32_t x1, const f32_t x2, const f32_t y1, const f32_t y2)
{
    return (((y2 - y1) / (x2 - x1)) * (x - x1) + y1);
}
