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

#ifndef ALGORITHM_TYPES_H
#define ALGORITHM_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"

#include "base_types.h"

#include "platform_params.h"

/***************************** Type Definitions ******************************/

/*********************
 *** Kalman Filter ***
 ********************/

/** Get the size of an upper matrix, given the dimension. */
#define GET_SIZE_UPPER(N) ((((N)*(N))+(N)) / 2u)

/** Get the size of a diagonal matrix, given the dimension. */
#define GET_SIZE_DIAGONAL(N) (N)

/** Get the index of an upper matrix. */
#define GET_UPPER_INDEX(i,j,N) (((N)*(i)) - (((i)*((i) - 1u)) / 2u)) + ((j)-(i))

/** Get the size of an upper matrix in bytes, given the dimension. */
#define GET_BYTE_UPPER(N) (sizeof(f32_t) * GET_SIZE_UPPER(N))

/**
  * @enum StateType_t
  * @brief The different states for the LKF.
  */
typedef enum {
    STATE_X = 0,
    STATE_Y,
    STATE_VX,
    STATE_VY,
    STATE_MAX
} StateType_t;

/** The number of the states used in the LKF. */
#define KALMAN_STATES (STATE_MAX)

/**
  * @defgroup track_matrices The matrices stored in a track
  *
  * @{
  */
typedef f32_t KalmanX_t[KALMAN_STATES];
typedef f32_t KalmanP_t[KALMAN_STATES * KALMAN_STATES];
typedef f32_t KalmanPu_t[GET_SIZE_UPPER(KALMAN_STATES)];
typedef f32_t KalmanPd_t[GET_SIZE_DIAGONAL(KALMAN_STATES)];
/** @} */

/**
  * @defgroup predict_matrices The matrices used in the predict step
  *
  * @{
  */
typedef f32_t KalmanF_t[KALMAN_STATES * KALMAN_STATES];
typedef f32_t KalmanQ_t[KALMAN_STATES * KALMAN_STATES];
typedef f32_t KalmanQu_t[GET_SIZE_UPPER(KALMAN_STATES)];
typedef f32_t KalmanQd_t[GET_SIZE_DIAGONAL(KALMAN_STATES)];
/** @} */

/**
  * @defgroup plot_matrices The matrices stored in a plot
  *
  * @{
  */
typedef f32_t KalmanZ_t[KALMAN_STATES];
typedef f32_t KalmanR_t[KALMAN_STATES * KALMAN_STATES];
/** @} */

/** The matrix used in the update step for each state. */
typedef f32_t KalmanH_t[KALMAN_STATES];

/***********************
 *** Prefused Object ***
 **********************/

/**
  * @struct Plot_t
  * @brief The plot (measurement) of a prefused object.
  */
typedef struct {
    KalmanZ_t Z;
    KalmanR_t R;
    f32_t weight;
} Plot_t;

/**
  * @struct PrefusedObject_t
  * @brief A prefused (input) object.
  */
typedef struct {
    u8_t valid;
    Plot_t plot;

    const Sensor_t* sensor;
  
    f32_t priority;
} PrefusedObject_t;

/********************
 *** Fused Object ***
 *******************/

/**
  * @struct Track_t
  * @brief The track (state) of a fused object.
  */
typedef struct {
    KalmanX_t X;
    KalmanP_t P;
    KalmanPu_t P_U;
    KalmanPd_t P_D;
} Track_t;

/**
  * @struct FusedObject_t
  * @brief A fused (prior & posterior) object.
  * @todo Remove id and replace with a valid flag;
  *       the id will be determined by the objects position in the list!
  */
typedef struct {
    u8_t id;
    Track_t track;
    
    u16_t lifetimeCounter;
    u8_t seenThisCycle[NUM_SENSORS];
    u8_t lostCounter;

    f32_t priority;
} FusedObject_t;

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* ALGORITHM_TYPES_H */
