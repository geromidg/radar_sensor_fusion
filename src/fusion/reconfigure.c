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
  * This module enables the dynamic reconfiguring of the parameters
  * used in the algorithm. When the special CAN message is received,
  * a callback handles the update of 1 parameter. Each frame contains
  * info to select the parameter and update its value.
  *
  * All configurable parameters should only be used on runtime.
  * When a new configuration is received, the module's prefused/fused objects
  * and parameters will be reinitialized.
  */

/******************************** Inclusions *********************************/

#include "config.h"

#include "algorithm_interface.h"
#include "config.h"

#include "reconfigure.h"

/************************ Global Variable Definitions ************************/

f32_t SIGMA_BASE    = 0.1f;
f32_t SIGMA_RANGE   = 0.5f;
f32_t SIGMA_DOPPLER = 1.5f;
f32_t SIGMA_BEARING = 3.0f;

f32_t MAX_BEARING_CONFIDENCE   = 1.0f;
f32_t MIN_BEARING_CONFIDENCE   = 0.7f;
f32_t SENSOR_WEAK_BEARING_AREA = 10.0f;

f32_t Q_SIGMA_X  = 1.5f;
f32_t Q_SIGMA_Y  = 1.5f;
f32_t Q_SIGMA_VX = 3.0f;
f32_t Q_SIGMA_VY = 3.0f;

f32_t PRUNE_LIMIT_X  = 2.0f;
f32_t PRUNE_LIMIT_Y  = 2.0f;
f32_t PRUNE_LIMIT_VX = 5.0f;
f32_t PRUNE_LIMIT_VY = 5.0f;

f32_t GATING_WEIGHT_X  = 10.0f;
f32_t GATING_WEIGHT_Y  = 10.0f;
f32_t GATING_WEIGHT_VX = 30.0f;
f32_t GATING_WEIGHT_VY = 30.0f;
f32_t ACCEPTANCE_GATE_SUM_FACTOR = 1.0f;

u8_t MAX_COASTING_CYCLES = 20u;
f32_t MIN_COASTING_DIST  = 5.0f;
f32_t MAX_COASTING_DIST  = 15.0f;

f32_t MAX_VELOCITY = 19.2f;
f32_t MIN_VELOCITY = 3.0f;

u8_t MIN_LIFETIME_TX_CYCLES = 3u;

/***************************** Public Functions ******************************/

void CfgCallback(u8_t cfgSelect, f32_t cfgValue)
{
    u8_t valid = 1u;

    switch (cfgSelect)
    {
        case 0u:
            SIGMA_BASE = cfgValue;
            break;
        case 1u:
            SIGMA_RANGE = cfgValue;
            break;
        case 2u:
            SIGMA_DOPPLER = cfgValue;
            break;
        case 3u:
            SIGMA_BEARING = cfgValue;
            break;
        case 4u:
            MAX_BEARING_CONFIDENCE = cfgValue;
            break;
        case 5u:
            MIN_BEARING_CONFIDENCE = cfgValue;
            break;
        case 6u:
            SENSOR_WEAK_BEARING_AREA = cfgValue;
            break;
        case 7u:
            Q_SIGMA_X = cfgValue;
            break;
        case 8u:
            Q_SIGMA_Y = cfgValue;
            break;
        case 9u:
            Q_SIGMA_VX = cfgValue;
            break;
        case 10u:
            Q_SIGMA_VY = cfgValue;
            break;
        case 11u:
            PRUNE_LIMIT_X = cfgValue;
            break;
        case 12u:
            PRUNE_LIMIT_Y = cfgValue;
            break;
        case 13u:
            PRUNE_LIMIT_VX = cfgValue;
            break;
        case 14u:
            PRUNE_LIMIT_VY = cfgValue;
            break;
        case 15u:
            GATING_WEIGHT_X = cfgValue;
            break;
        case 16u:
            GATING_WEIGHT_Y = cfgValue;
            break;
        case 17u:
            GATING_WEIGHT_VX = cfgValue;
            break;
        case 18u:
            GATING_WEIGHT_VY = cfgValue;
            break;
        case 19u:
            ACCEPTANCE_GATE_SUM_FACTOR = cfgValue;
            break;
        case 20u:
            MAX_COASTING_CYCLES = (u8_t)cfgValue;
            break;
        case 21u:
            MIN_COASTING_DIST = cfgValue;
            break;
        case 22u:
            MAX_COASTING_DIST = cfgValue;
            break;
        case 23u:
            MAX_VELOCITY = cfgValue;
            break;
        case 24u:
            MIN_VELOCITY = cfgValue;
            break;
        case 25u:
            MIN_LIFETIME_TX_CYCLES = (u8_t)cfgValue;
            break;
        default:
            valid = 0u;
            break;
    }

    if (valid)
    {
        InitializeAlgorithm();
    }
}