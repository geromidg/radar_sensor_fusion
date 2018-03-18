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
  * The algorithm consists of 3 steps:
  * 1. Predict the objects of the previous cycle to receive the prior objects.
  * 2. Update the prior objects by associating the prefused objects (input).
  *    If an input is associated with a prior, it fuses with the prior.
  *    If not, a new object is created.
  * 3. Manage the posterior objects by checking if they are still valid etc.
  */

/******************************** Inclusions *********************************/

#include <string.h>

#include "constants.h"
#include "platform_params.h"
#include "reconfigure.h"
#include "fusion_utils.h"
#include "radar_utils.h"
#include "tracking.h"

#include "fusion.h"

/************************ Static Function Prototypes *************************/

/**
  * @brief Predicts the next state of the fused objects.
  * @details For each valid fused object, predicts its next state.
  * @return Void.
  */
static void Predict(FusedObject_t* fusedObjectList);

/**
  * @brief Updates the fused objects with the information from the prefused objects.
  * @details Each prefused object passes from an acceptance gate.
  *          If it succeeds, it fuses with its paired object. If not, an new object is created from it.
  * @return Void.
  */
static void Update(const PrefusedObject_t* prefusedObjectList, FusedObject_t* fusedObjectList);

/**
  * @brief Performs maintenance functions on the fused objects.
  * @details Each valid fused object is checked with the rest of the objects for pruning.
  *          If the 2 objects are very close, one of them will be deleted.
  *          All the remaining valid objects have their lifetime and lost counters updated.
  * @return Void.
  */
static void Manage(FusedObject_t* fusedObjectList);

/***************************** Static Functions ******************************/

void Predict(FusedObject_t* fusedObjectList)
{
    u8_t i;

    for (i = 0u; i < NUM_FUSED_OBJ; i++)
    {
        if (fusedObjectList[i].id != INVALID_ID)
        {
            PredictTrack(&fusedObjectList[i].track);

            fusedObjectList[i].priority = GetObjectPriority(fusedObjectList[i].track.X[STATE_X], fusedObjectList[i].track.X[STATE_Y]);
        }
    }
}

void Update(const PrefusedObject_t* prefusedObjectList,  FusedObject_t* fusedObjectList)
{
    u8_t i;

    for (i = 0u; i < NUM_PREFUSED_OBJ; i++)
    {
        if (prefusedObjectList[i].valid)
        {
            AssociatePrefusedObject(&prefusedObjectList[i], fusedObjectList);
        }
    }
}

void Manage(FusedObject_t* fusedObjectList)
{
    u8_t i, j;

    for (i = 0u; i < NUM_FUSED_OBJ; i++)
    {
        if (fusedObjectList[i].id != INVALID_ID)
        {
            for (j = (i + 1u); j < NUM_FUSED_OBJ; j++)
            {
                if ((fusedObjectList[i].id != INVALID_ID) &&
                    (fusedObjectList[j].id != INVALID_ID))
                {
                    CheckObjectsForPruning(&fusedObjectList[i], &fusedObjectList[j]);
                }
            }
        }
    }

    for (i = 0u; i < NUM_FUSED_OBJ; i++)
    {
        if (fusedObjectList[i].id != INVALID_ID)
        {
            MaintainObject(&fusedObjectList[i]);
        }
    }
}

/***************************** Public Functions ******************************/

void InitializeFusion(void)
{
    InitializeTracking(CYCLE_TIME);
    InitializeFusionUtils();
}

void RunFusion(const PrefusedObject_t* prefusedObjectList, FusedObject_t* fusedObjectList)
{
    Predict(fusedObjectList);
    Update(prefusedObjectList, fusedObjectList);
    Manage(fusedObjectList);
}
