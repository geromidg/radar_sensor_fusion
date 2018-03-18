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

/******************************** Inclusions *********************************/

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <float.h>

#include "algorithm_types.h"

#include "kalman_utils.h"

/***************************** Static Variables ******************************/

/** The state prediction matrix of the process. */
static KalmanF_t F;

/** The upper decomposed matrix of Q. */
static KalmanQu_t Qu;

/** The diagonal decomposed matrix of Q. */
static KalmanQd_t Qd;

/************************ Static Function Prototypes *************************/

/**
  * @brief Transforms a diagonal vector to a diagonal matrix.
  * @param diagonal The diagonal vector.
  * @param square The square matrix.
  * @return Void.
  */
void ConvertDiagonalVectorToMatrix(const f32_t* diagonal, f32_t* square);

/**
  * @brief Transforms an upper triangular matrix to a full matrix.
  * @param upper The upper matrix.
  * @param square The square matrix.
  * @return Void.
  */
void ConvertUpperMatrixToFull(const f32_t* upper, f32_t* square);

/**
  * @brief Multiplies two matrices.
  * @param A The first matrix.
  * @param B The second matrix.
  * @return Void.
  */
void MultiplyMatrix(const f32_t* A, const f32_t* B, f32_t* C);

/**
  * @brief Transposes a matrix.
  * @param A The original matrix.
  * @param At The transposed matrix.
  * @return Void.
  */
void TransposeMatrix(const f32_t* A, f32_t* At);

/***************************** Static Functions ******************************/

void ConvertDiagonalVectorToMatrix(const f32_t* diagonal, f32_t* square)
{
    s16_t i;

    (void)memset(square, 0u, sizeof(f32_t) * KALMAN_STATES * KALMAN_STATES);

    for (i = 0u; i < KALMAN_STATES; i++)
    {
        square[(KALMAN_STATES * i) + i] = diagonal[i];
    }
}

void ConvertUpperMatrixToFull(const f32_t* upper, f32_t* square)
{
    s16_t i, j;

    (void)memset(square, 0u, sizeof(f32_t) * KALMAN_STATES * KALMAN_STATES);

    for (i = 0u; i < KALMAN_STATES; i++)
    {
        for (j = 0u; j < KALMAN_STATES; j++)
        {
            if (i <= j)
            {
                square[(KALMAN_STATES * i) + j] = upper[GET_UPPER_INDEX(i, j, KALMAN_STATES)];
            }
        }
    }
}

void MultiplyMatrix(const f32_t* A, const f32_t* B, f32_t* C)
{
    s16_t i, j, k;
    
    for (i = 0u; i < KALMAN_STATES; i++)
    {
        for (j = 0u; j < KALMAN_STATES; j++)
        {
            C[(j * KALMAN_STATES) + i] = 0.f;
            
            for (k = 0u; k < KALMAN_STATES; k++)
            {
                C[(KALMAN_STATES * j) + i] += A[(KALMAN_STATES * j) + k] * B[(KALMAN_STATES * k) + i];
            }
        }
    }
}

void TransposeMatrix(const f32_t* A, f32_t* At)
{
    s16_t i, j;
    
    for (i = 0u; i < KALMAN_STATES; i++)
    {
        for (j = 0u; j < KALMAN_STATES; j++)
        {
            At[(KALMAN_STATES * j) + i] = A[(KALMAN_STATES * i) + j];
        }
    }
}

/***************************** Public Functions ******************************/

void ComposeUD(const f32_t* U, const f32_t* D, f32_t* UDU)
{
    f32_t tempMatrix1[KALMAN_STATES * KALMAN_STATES];
    f32_t tempMatrix2[KALMAN_STATES * KALMAN_STATES];
    f32_t tempMatrix3[KALMAN_STATES * KALMAN_STATES];

    (void)ConvertDiagonalVectorToMatrix(D, tempMatrix1);
    (void)ConvertUpperMatrixToFull(U, tempMatrix2);
    (void)TransposeMatrix((const f32_t*) tempMatrix2, UDU);
    (void)MultiplyMatrix((const f32_t*) tempMatrix1, (const f32_t*) UDU, tempMatrix3);
    (void)MultiplyMatrix((const f32_t*) tempMatrix2, (const f32_t*) tempMatrix3, UDU);
}

void DecomposeUD(const f32_t* UDU, f32_t* U, f32_t* D)
{
    s16_t i, j, k;
    f32_t sigma;

    (void)memset(U, 0, GET_BYTE_UPPER(KALMAN_STATES));
    (void)memset(D, 0, sizeof(f32_t) * KALMAN_STATES);

    for (j = KALMAN_STATES - 1u; j >= 0u; j--)
    {
        for (i = j; i >= 0u; i--)
        {
            sigma = UDU[(KALMAN_STATES * i) + j];
            
            for (k = j + 1u; k <= KALMAN_STATES - 1u; k++)
            {
                sigma += -U[GET_UPPER_INDEX(i, k, KALMAN_STATES)] * D[k] * U[GET_UPPER_INDEX(j, k, KALMAN_STATES)];
            }
            
            if (i == j)
            {
                D[j] = sigma;
                U[GET_UPPER_INDEX(j, j, KALMAN_STATES)]  = 1.0f;
            }
            else
            {
                U[GET_UPPER_INDEX(i, j, KALMAN_STATES)] = sigma / D[j];
            }
        }
    }
}

void PredictState(const f32_t* inputF, f32_t* state)
{
    s16_t i, j;
    f32_t tempVector[KALMAN_STATES];

    (void)memset(tempVector, 0u, sizeof(f32_t) * KALMAN_STATES);

    for (i = 0u; i < KALMAN_STATES; i++)
    {
        for (j = 0u; j < KALMAN_STATES; j++)
        {
            tempVector[i] += inputF[(KALMAN_STATES * i) + j] * state[j];
        }
    }

    (void)memcpy(state, tempVector, sizeof(f32_t) * KALMAN_STATES);
}

void FuseState(const f32_t innovation, const f32_t alpha, const f32_t* transformation, f32_t* state, f32_t* outputQu, f32_t* outputQd)
{
    s16_t i, j;
    f32_t beta, lambda, gamma, scaledInnovation;
    f32_t tempAlpha = alpha;
    f32_t tempVector1[KALMAN_STATES];
    f32_t tempVector2[KALMAN_STATES];
    
    (void)memset(tempVector1, 0u, sizeof(f32_t) * KALMAN_STATES);
    (void)memset(tempVector2, 0u, sizeof(f32_t) * KALMAN_STATES);

    gamma = 1.f / tempAlpha;
    
    for (j = 0u; j < KALMAN_STATES; j++)
    {
        tempVector1[j] = transformation[j];
    
        for (i = 0u; i <= j - 1u; i++)
        {
            tempVector1[j] += outputQu[GET_UPPER_INDEX(i, j, KALMAN_STATES)] * (transformation[i]);
        }
    }

    for (j = 0u; j < KALMAN_STATES; j++)
    {
        tempVector2[j] = outputQd[j] * tempVector1[j];
    }

    for (j = 0u; j < KALMAN_STATES; j++)
    {
        beta = tempAlpha;
        tempAlpha += tempVector1[j] * tempVector2[j];
        lambda = -tempVector1[j] * gamma;
        gamma = 1.0f / tempAlpha;
        outputQd[j] *= beta * gamma;

        for (i = 0u; i <= j - 1u; i++)
        {
            beta = outputQu[GET_UPPER_INDEX(i, j, KALMAN_STATES)];
            outputQu[GET_UPPER_INDEX(i, j, KALMAN_STATES)] = beta + (tempVector2[i] * lambda);
            tempVector2[i] += tempVector2[j] * beta;
        }
    }

    scaledInnovation = gamma * innovation;

    for (j = 0u; j < KALMAN_STATES; j++)
    {
        state[j] += scaledInnovation * tempVector2[j];
    }
}

void EstimateCovariance(const f32_t* inputF, const f32_t* inputQu, const f32_t* inputQd, f32_t* outputQu, f32_t* outputQd)
{
    s16_t i, j, k;
    f32_t sigma;

    (void)memset(Qd, 0u, sizeof(f32_t) * KALMAN_STATES);
    (void)memcpy(Qu, inputQu, GET_BYTE_UPPER(KALMAN_STATES));

    for (i = 0u; i < KALMAN_STATES; i++)
    {
        for (j = KALMAN_STATES - 1u; j >= 0u; j--)
        {
            sigma = inputF[(KALMAN_STATES * i) + j];
        
            for (k = 0u; k <= j - 1u; k++)
            {
                sigma += inputF[(KALMAN_STATES * i) + k] * outputQu[GET_UPPER_INDEX(k, j, KALMAN_STATES)];
            }
            
            F[(KALMAN_STATES * i) + j] = sigma;
        }
    }

    for (i = KALMAN_STATES - 1u; i >= 0u; i--)
    {
        sigma = 0.f;
    
        for (j = 0u; j < KALMAN_STATES; j++)
        {
            sigma += F[(KALMAN_STATES * i) + j] * F[(KALMAN_STATES * i) + j]* outputQd[j];
        
            if (i <= j)
            {
                sigma += Qu[GET_UPPER_INDEX(i, j, KALMAN_STATES)] * Qu[GET_UPPER_INDEX(i, j, KALMAN_STATES)] * inputQd[j];
            }
        }

        Qd[i] = sigma;
        
        for (j = 0u; j <= i - 1u; j++)
        {
            sigma = 0.0f;
        
            for (k = 0u; k < KALMAN_STATES; k++)
            {
                sigma += F[(KALMAN_STATES * i) + k] * outputQd[k] * F[(KALMAN_STATES*j) + k];
            
                if ((i <= k) && (j <= k))
                {
                    sigma += Qu[GET_UPPER_INDEX(i, k, KALMAN_STATES)] * inputQd[k] * Qu[GET_UPPER_INDEX(j, k, KALMAN_STATES)];
                }
            }

            outputQu[GET_UPPER_INDEX(j,i,KALMAN_STATES)] = sigma / Qd[i];

            for (k = 0u; k < KALMAN_STATES; k++)
            {
                F[(KALMAN_STATES*j) + k] += -outputQu[GET_UPPER_INDEX(j,i,KALMAN_STATES)] * F[(KALMAN_STATES * i) + k];
            
                if ((i <= k) && (j <= k))
                {
                    Qu[GET_UPPER_INDEX(j, k, KALMAN_STATES)] += -outputQu[GET_UPPER_INDEX(j,i,KALMAN_STATES)] * Qu[GET_UPPER_INDEX(i, k, KALMAN_STATES)];
                }
            }
        }
    }

    (void)memcpy(outputQd, Qd, sizeof(f32_t) * KALMAN_STATES);
}
