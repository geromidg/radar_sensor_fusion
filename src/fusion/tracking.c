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
  * The tracker solves the filtering problem using a Linear Kalman Filter (LKF).
  * The LKF is implemented using 4 states: x, y, vx, vy.
  
  * For each cycle, the steps of the LKF are:
  * 1. Predict the new states of a fused object (track).
  * 2. Fuse any prefused object (plot) with its associated track.
  *    If a plot is not associated with a track, initialize a new track.
  */

/******************************** Inclusions *********************************/

#include <string.h>

#include "kalman_utils.h"

#include "constants.h"
#include "config.h"
#include "fusion_utils.h"
#include "radar_utils.h"

#include "tracking.h"

/***************************** Static Variables ******************************/

/** The state prediction matrix of the process. */
static KalmanF_t F;

/** The noise covariance matrix of the process. */
static KalmanQ_t Q;

/** The upper decomposed matrix of Q. */
static KalmanQu_t Qu;

/** The diagonal decomposed matrix of Q. */
static KalmanQd_t Qd;

/** The sensor transformation matrix (to the tracker's domain).*/
static KalmanH_t H;

/************************ Static Function Prototypes *************************/

/**
  * @brief Initializes the F (state prediction) matrix of the Kalman filter.
  * @details The F matrix is calculated according to the model of an object's motion.
  *          Currently, a decoupled model is used (linear equation).
  * @param dt The cycle time of the algo.
  * @return Void.
  */
static void InitF(const f32_t dt);

/**
  * @brief Initializes the Q (noise covariance) matrix of the Kalman filter.
  * @details The Q matrix is calculated using user-defined parameters.
  *          The bigger the values of the matrix, the higher the uncertainty of the prediction will be.
  * @param dt The cycle time of the algo.
  * @return Void.
  */
static void InitQ(const f32_t dt);

/***************************** Static Functions ******************************/

void InitF(const f32_t dt)
{
    (void)memset(F, 0, sizeof(KalmanF_t));

    F[(KALMAN_STATES * STATE_X) + STATE_X] = 1.f;
    F[(KALMAN_STATES * STATE_X) + STATE_VX] = dt;

    F[(KALMAN_STATES * STATE_Y) + STATE_Y] = 1.f;
    F[(KALMAN_STATES * STATE_Y) + STATE_VY] = dt;

    F[(KALMAN_STATES * STATE_VX) + STATE_VX] = 1.f;

    F[(KALMAN_STATES * STATE_VY) + STATE_VY] = 1.f;
}

void InitQ(const f32_t dt)
{
    f32_t var_q_x = Q_SIGMA_X * Q_SIGMA_X;
    f32_t var_q_y = Q_SIGMA_Y * Q_SIGMA_Y;
    f32_t var_q_vx = Q_SIGMA_VX * Q_SIGMA_VX;
    f32_t var_q_vy = Q_SIGMA_VY * Q_SIGMA_VY;

    (void)memset(Q, 0, sizeof(KalmanQ_t));
  
    Q[(KALMAN_STATES * STATE_X) + STATE_X]   = (var_q_x * dt) + ((var_q_vx * dt * dt * dt) / 3.f);
    Q[(KALMAN_STATES * STATE_X) + STATE_VX]  = (var_q_vx * dt * dt) / 2.f;

    Q[(KALMAN_STATES * STATE_Y) + STATE_Y]   = (var_q_y * dt) +  ((var_q_vy * dt * dt * dt) / 3.f);
    Q[(KALMAN_STATES * STATE_Y) + STATE_VY]  = (var_q_vy * dt * dt) / 2.f;

    Q[(KALMAN_STATES * STATE_VX) + STATE_VX] = var_q_vx * dt;
    Q[(KALMAN_STATES * STATE_VX) + STATE_X]  = Q[(KALMAN_STATES * STATE_X) + STATE_VX];

    Q[(KALMAN_STATES * STATE_VY) + STATE_VY] = var_q_vy * dt;
    Q[(KALMAN_STATES * STATE_VY) + STATE_Y]  = Q[(KALMAN_STATES * STATE_Y) + STATE_VY];
}

/***************************** Public Functions ******************************/

void InitializeTracking(const f32_t dt)
{
    InitF(dt);

    InitQ(dt);

    (void)DecomposeUD((const f32_t*) Q, Qu, Qd);
}

void InitializeTrack(Track_t* track, const Plot_t* plot)
{
    u8_t i;
    
    (void)memset(track->X, 0, sizeof(KalmanX_t));
    (void)memset(track->P, 0, sizeof(KalmanP_t));
    
    for (i = 0u; i < KALMAN_STATES; i++)
    {
        track->X[i] = plot->Z[i];

        track->P[(KALMAN_STATES * i) + i] = plot->R[(KALMAN_STATES * i) + i];
    }
        
    (void)DecomposeUD(track->P, track->P_U, track->P_D);
}

void PredictTrack(Track_t* track)
{
    (void)EstimateCovariance((const f32_t*) F, Qu, Qd, track->P_U, track->P_D);

    (void)PredictState(F, track->X);

    (void)ComposeUD(track->P_U, track->P_D, (f32_t*) track->P);
}

void FuseTrack(Track_t* track, const Plot_t* plot)
{
    u8_t i;
    f32_t innovation;
    
    for (i = 0u; i < KALMAN_STATES; i++)
    {
        (void)memset(H, 0, sizeof(KalmanH_t));
        H[i] = 1.f;

        innovation = plot->Z[i] - track->X[i];
        innovation *= plot->weight;

        (void)FuseState(innovation, plot->R[(KALMAN_STATES * i) + i], (const f32_t*) H, track->X, track->P_U, track->P_D);
    }
}
