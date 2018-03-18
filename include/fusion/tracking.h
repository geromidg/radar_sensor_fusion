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

#ifndef TRACKING_H
#define TRACKING_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"
#include "algorithm_types.h"

/***************************** Public Functions ******************************/

/**
  * @brief Initializes the tracker of the algo.
  * @details Initializes the process' state prediction and noise covariance matrices (F and Q).
  *          A UD (Cholesky) decomposition is perfomed in Q, for use in the Kalman update step.
  * @param dt The cycle time of the algo.
  * @return Void.
  */
void InitializeTracking(const f32_t dt);

/**
  * @brief Initializes a track given a plot (measurement).
  * @details The plot's state (Z) and covariance matrix (R) are copied to
  *          the track's state (X) and covariance matrix (P).
  *          A UD decomposition is performed on the P matrix for numerical stability.
  * @param track The track of the object that will be initialized.
  * @param plot The plot that the object is initialized from.
  * @return Void.
  */
void InitializeTrack(Track_t* track, const Plot_t* plot);

/**
  * @brief Performs the predict step of the Kalman filter for a track.
  * @details First, the P matrix is predicted and directly decomposed (UD).
  *          Then, the X matrix is predicted.
  *          Lastly, backwards composition is performed on P for use in the update step.
  * @param track The track to be predicted.
  * @todo Handle object appropriately if new values are out of limits.
  * @return Void.
  */
void PredictTrack(Track_t* track);

/**
  * @brief Performs the update step of the Kalman filter.
  * @details For each state, the innovation (residual) is calculated and the two distributions
  *          are multiplied (fused).
  * @param track The original track to be fused with.
  * @param plot The plot that will be fused with its paired track.
  * @return Void.
  */
void FuseTrack(Track_t* track, const Plot_t* plot);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* TRACKING_H */
