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

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"

/*********************** Global Variable Declarations ************************/

/**
  * @defgroup radar_std Standard deviation values of the radar
  * These determine the R matrix of Kalman.
  * The base sigma is the minimum sigma that a state can take.
  * @todo Receive direct values from prefused objects and don't assume.
  *
  * @{
  */
extern f32_t SIGMA_BASE;
extern f32_t SIGMA_RANGE;
extern f32_t SIGMA_DOPPLER;
extern f32_t SIGMA_BEARING;
/** @} */

/**
  * @defgroup radar_bearing_confidence Confidence values of the radar
  * These determine the level of confidence of a radar measurement, and
  * directly weight the innovation of the Kalman's update.
  * The max bearing confidence belongs to the true bearing (0 deg).
  * The min bearing confidence belongs to the min bearing (+/- 70 deg).
  * The sensor's weak bearing area determines the maximum bearing from the limit,
  * where the sensor's measurement can be trusted with max confidence.
  * @todo Derive values from calibration.
  *
  * @{
  */
extern f32_t MAX_BEARING_CONFIDENCE;
extern f32_t MIN_BEARING_CONFIDENCE;
extern f32_t SENSOR_WEAK_BEARING_AREA;
/** @} */

/**
  * @defgroup noise_covariance Parameters for the noise covariance matrix of the process' model
  * These determine the Q matrix of Kalman.
  *
  * @{
  */
extern f32_t Q_SIGMA_X;
extern f32_t Q_SIGMA_Y;
extern f32_t Q_SIGMA_VX;
extern f32_t Q_SIGMA_VY;
/** @} */

/**
  * @defgroup prune_limits Each state's limit for pruning
  * All states should be under those limits for the object to be pruned.
  * @todo Should also depend on road geo, and tracker's noise.
  *
  * @{
  */
extern f32_t PRUNE_LIMIT_X;
extern f32_t PRUNE_LIMIT_Y;
extern f32_t PRUNE_LIMIT_VX;
extern f32_t PRUNE_LIMIT_VY;
/** @} */

/**
  * @defgroup gating_weights Each state's weight for gating
  * The weight of each state determines how much its Gaussian will affect the object's gating.
  * This weight is applied after the similarity value is calculated, and is summed to the total similarity value.
  * The acceptance gate sum factor is a factor determining the limit, that the sum of the states' gating values needs
  * to pass for an object to pass the total gate.
  * @example For the state VX to pass the gate, the condition is: similarity(track_vx, plot_vx) * `GATING_WEIGHT_VX` > `STATE_GATING_VALUE_MIN_LIMIT`
  *
  * @{
  */
extern f32_t GATING_WEIGHT_X;
extern f32_t GATING_WEIGHT_Y;
extern f32_t GATING_WEIGHT_VX;
extern f32_t GATING_WEIGHT_VY;
extern f32_t ACCEPTANCE_GATE_SUM_FACTOR;
/** @} */

/**
  * @defgroup coasting_limits The coasting limits of the algorithm
  * The number of cycles an object that has been lost by sensors will be coasted (predicted).
  * Example: 20 cycles = 2.4m at 3m/s (~10kph)
  * @todo Make it variable, according to each object's speed.
  * @todo Use coasting dist.
  *
  * @{
  */
extern u8_t MAX_COASTING_CYCLES;
extern f32_t MIN_COASTING_DIST;
extern f32_t MAX_COASTING_DIST;
/** @} */

/**
  * @defgroup velocity_limits The velocity limits of the algorithm
  * Both limits are expressed at m/s units.
  * These velocities determine the limits at which the algorithm can
  * track an object. There are no guarantees for velocities beyond these values.
  * The max velocity is taken from the system's requirements.
  * The min velocity is taken from the system's assumptions.
  * @todo Use these values in the association step of the algo.
  *
  * @{
  */
extern f32_t MAX_VELOCITY;
extern f32_t MIN_VELOCITY;
/** @} */

/**
  * @defgroup object_lifetime Parameters related to the lifetime of the objects
  * The number of the cycles it takes for an object to be outputted.
  * @todo Adjust according to requirements/assumptions.
  *
  * @{
  */
extern u8_t MIN_LIFETIME_TX_CYCLES;
/** @} */

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* CONFIG_H */
