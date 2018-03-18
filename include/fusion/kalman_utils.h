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

#ifndef KALMAN_UTILS_H
#define KALMAN_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"

/***************************** Public Functions ******************************/

/**
  * @brief Composes the UDU matrix given the U and D matrices.
  * @param U The upper triangular matrix.
  * @param D The diagonal matrix.
  * @param UDU The UDU matrix.
  * @return Void.
  */
void ComposeUD(const f32_t* U, const f32_t* D, f32_t* UDU);

/**
  * @brief Computes the UD decomposition given a UDU matrix.
  * @param UDU The UDU matrix.
  * @param U The upper triangular matrix.
  * @param D The diagonal matrix.
  * @return Void.
  */
void DecomposeUD(const f32_t* UDU, f32_t* U, f32_t* D);

/**
  * @brief Predicts the state vector given the state transition matrix.
  * @param inputF The state transition matrix.
  * @param state The state vector.
  * @return Void.
  */
void PredictState(const f32_t* inputF, f32_t* state);

/**
  * @brief Fuses a measurement with the state and calculate the UD decomposition of the state covariance matrix.
  * @param innovation The measurement innovation.
  * @param alpha The noise of the innovation.
  * @param transformation The sensor transformation matrix.
  * @param state The state vector.
  * @param outputQu The upper triangular matrix of the UD factor.
  * @param outputQd The diagonal matrix of the UD factor.
  * @return Void.
  */
void FuseState(const f32_t innovation, const f32_t alpha, const f32_t* transformation, f32_t* state, f32_t* outputQu, f32_t* outputQd);

/**
  * @brief Estimates the UD decomposition of the covariance matrix of the predicted state.
  * @param inputF The state transition matrix.
  * @param inputQu The input upper triangular matrix of the UD factor.
  * @param inputQd The input diagonal matrix of the UD factor.
  * @param outputQu The output upper triangular matrix of the UD factor.
  * @param outputQd The output diagonal matrix of the UD factor.
  * @return Void.
  */
void EstimateCovariance(const f32_t* inputF, const f32_t* inputQu, const f32_t* inputQd, f32_t* outputQu, f32_t* outputQd);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* KALMAN_UTILS_H */
