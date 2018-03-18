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
  * Utilities for handling fused and prefused objects (init, reset, associate, prune etc.)
  */

/******************************** Inclusions *********************************/

#include <string.h>
#include <math.h>
#include "constants.h"
#include "platform_params.h"
#include "config.h"
#include "radar_utils.h"
#include "tracking.h"

#include "fusion_utils.h"

/***************************** Static Variables ******************************/

static f32_t gatingWeights[KALMAN_STATES];
static f32_t totalGatingValueMinLimit;

/************************ Static Function Prototypes *************************/

/**
  * @brief Get the confidence value of an object, according to its bearing (angle).
  * @details The bearing confidence is a normalized value that is calculated according
  *          to the specifications of the radar. When the bearing is close to the true bearing (0 deg),
  *          the confidence is at its maximum. Likewise, when the bearing is close to the radar's limits
  *          (weak angles), the confidence will have a small value. This behaviour can be observed during calibration.
  *          This value works as a weight for the innovation of the Kalman update step. This basically means
  *          that the objects near the FOV limits of the radar will not be trusted that much.
  * @param posX The x position of the object.
  * @param posY The y position of the object.
  * @todo Use a different function to approximate the bearing-error (bearing-confidence) function.
  *       For example, use the damped oscillation (exponential) function to approximate the behaviour during calibration.
  * @return The confidence value that was calculated.
  */
static f32_t GetBearingConfidence(const f32_t targetX, const f32_t targetY, const Sensor_t* sensor);

/**
  * @brief Transforms the prefused object from its sensor's coordinates to the global ones.
  * @details The transformation stored in the object's sensor is applied to its state.
  *          Currently, only the offset in x is applied. The offset in y has been applied in the sensors.
  * @param prefusedObjectList A list containing the prefused objects (input of algo).
  * @return Void.
  */
static void ApplySensorTF(PrefusedObject_t* prefusedObjectList);

/**
  * @brief Resets a fused object.
  * @details All the attributes of a fused object (id, track, counters etc.) are set to zero/default.
  * @param fusedObject A fused object that is outputted from the algo.
  * @return Void.
  */
static void ResetFusedObject(FusedObject_t* fusedObject);

/**
  * @brief Creates an object in the fused object list from a prefused object.
  * @details Generates a new id for the object and initializes the track.
  *          If the fused object list is full and every fused object has a higher priority
  *          that the prefused object's priority, then no action is taken.
  * @param fusedObjectList A list containing the fused objects (output of algo).
  * @param prefusedObject A prefused object used as an input to the algo.
  * @return Void.
  */
static void CreateFusedObject(FusedObject_t* fusedObjectList, const PrefusedObject_t* prefusedObject);

/**
  * @brief Finds the worst priority in the fused object list.
  * @details Finds the worst priority of all the valid fused objects and the index of the object holding it.
  *          If the list contains no valid object, the minimum default priority is returned. 
  * @param fusedObjectList A list containing the fused objects (output of algo).
  * @param objectIndex The index in the fusedObjectList where the object with the worst priority is located.
  * @return The lowest priority of all objects.
  */
static f32_t GetWorstPriority(const FusedObject_t* fusedObjectList, u8_t* objectIndex);

/**
  * @brief Get the first available id in the fused object list.
  * @details For each valid id, searches if that id is used by an object.
  *          If not valid id is found, an invalid (default) id is returned.
  * @warning An invalid id should be never returned. This function is normally called
  *          when the object list has a free slot or an object has been just deleted.
  * @param fusedObjectList A list containing the fused objects (output of algo).
  * @todo Make current available id static so all the id are recycled.
  * @return The available id found.
  */
static u8_t GetAvailableId(const FusedObject_t* fusedObjectList);

/**
  * @brief Determine if the id is in use inside the fused object list.
  * @details Cycles through all the fused objects to check if the id is in use.
  * @param fusedObjectList A list containing the fused objects (output of algo).
  * @param id The id to be checked if used inside the fusedObjectList.
  * @return Whether the id is used or not.
  */
static u8_t IsIdUsed(const FusedObject_t* fusedObjectList, const u8_t id);

/**
  * @brief Checks if the prefused oject is inside any of the acceptance gates of all fused object.
  * @details For all valid fused objects, checks if the gating value is above a limit and finds the best pair.
  *          If a pair (fused object) is found, its index in the fused object list is also returned.
  * @param prefusedObject A prefused object used as an input to the algo.
  * @param fusedObjectList A list containing the fused objects (output of algo).
  * @param pairIndex The index in the fusedObjectList where an object is paired with the prefusedObject.
  * @return Whether the prefused object passes the acceptance gate or not.
  */
static u8_t IsInsideAcceptanceGate(const PrefusedObject_t* prefusedObject, const FusedObject_t* fusedObjectList, u8_t* pairIndex);

/**
  * @brief Calculates the gating (comparison) value between a prefused and fused object.
  * @details The gating value is the comparison between the track's (fused state) and the plot's (prefused state) Gaussians.
  *          In particular, for each object, each state's (pos/vel) Gaussian is compared with the other object's state.
  *          The sum of all the states' comparisons is returned. The more similar the objects, the higher the value is.
  * @param prefusedObject A prefused object used as an input to the algo.
  * @param fusedObject A fused object that is outputted from the algo.
  * @return The calculated gating value.
  */
static f32_t GetGatingValue(const PrefusedObject_t* prefusedObject, const FusedObject_t* fusedObject);
   
/**
  * @brief Checks if the fused object is lost.
  * @details Sums the seen counter for all the sensors contained in this fused object.
  *          If the sum is equal to zero, this mean that the object is lost (not seen by any sensor).
  * @param fusedObject A fused object that is outputted from the algo.
  * @return Whether the object is lost or not.
  */
static u8_t IsObjectLost(const FusedObject_t* fusedObject);

/**
  * @brief Checks if the fused object is coastable.
  * @details An object is coastable if its current lost counter has not exceeded the coasting limit.
  *          This means, that in the next cycle it will be predicted, even if no
  *          prefused (input) object is associated to it.
  * @param fusedObject A fused object that is outputted from the algo.
  * @return Whether the object is should be coasted or not.
  */
static u8_t IsObjectCoastable(FusedObject_t* fusedObject);

/***************************** Static Functions ******************************/

void InitializeFusionUtils(void)
{
    gatingWeights[STATE_X] = GATING_WEIGHT_X;
    gatingWeights[STATE_Y] = GATING_WEIGHT_Y;
    gatingWeights[STATE_VX] = GATING_WEIGHT_VX;
    gatingWeights[STATE_VY] = GATING_WEIGHT_VY;
    
    totalGatingValueMinLimit = KALMAN_STATES * STATE_GATING_VALUE_MIN_LIMIT * ACCEPTANCE_GATE_SUM_FACTOR;
}

f32_t GetBearingConfidence(const f32_t targetX, const f32_t targetY, const Sensor_t* sensor)
{
    f32_t confidence;
    f32_t sensorX, sensorY;
    f32_t trueBearing, maxBearing, weakBearing, targetBearing;

    sensorX = targetX - sensor->tf.x;
    sensorY = targetY - sensor->tf.y;
   
    trueBearing = 0.f;
    maxBearing = sensor->tf.fov / 2.f;
    weakBearing = maxBearing - SENSOR_WEAK_BEARING_AREA;
    targetBearing = fabs(RAD2DEG(GetBearing(sensorX, sensorY)) - sensor->tf.mounting);

    if (targetBearing >= trueBearing && targetBearing <= weakBearing)
    {
        confidence = MAX_BEARING_CONFIDENCE;
    }
    else if (targetBearing > weakBearing && targetBearing <= maxBearing)
    {
        confidence = GetLinInterpolatedValue(targetBearing, weakBearing, maxBearing, MAX_BEARING_CONFIDENCE, MIN_BEARING_CONFIDENCE);
    }
    else
    {
        confidence = MIN_BEARING_CONFIDENCE;
    }

    return confidence;
}

void ApplySensorTF(PrefusedObject_t* prefusedObject)
{
    prefusedObject->plot.Z[STATE_X] += prefusedObject->sensor->tf.canX;
    prefusedObject->plot.Z[STATE_Y] += prefusedObject->sensor->tf.canY;
}

void ResetFusedObject(FusedObject_t* fusedObject)
{
    fusedObject->id = INVALID_ID;

    (void)memset(&fusedObject->track, 0, sizeof(Track_t));

    fusedObject->lifetimeCounter = 0u;
    (void)memset(fusedObject->seenThisCycle, 0, sizeof(u8_t) * NUM_SENSORS);
    fusedObject->lostCounter = 0u;

    fusedObject->priority = 0.f;
}

void CreateFusedObject(FusedObject_t* fusedObjectList, const PrefusedObject_t* prefusedObject)
{
    u8_t index;

    if (prefusedObject->priority > GetWorstPriority(fusedObjectList, &index))
    {
        if (fusedObjectList[index].id != INVALID_ID)
        {
            ResetFusedObject(&fusedObjectList[index]);
        }

        fusedObjectList[index].id = GetAvailableId(fusedObjectList);

        InitializeTrack(&fusedObjectList[index].track, &prefusedObject->plot);
    }
}

f32_t GetWorstPriority(const FusedObject_t* fusedObjectList, u8_t* objectIndex)
{
    u8_t i;
    f32_t worstPriority = MAX_PRIORITY;

    for (i = 0u; i < NUM_FUSED_OBJ; i++)
    {
        if (fusedObjectList[i].id != INVALID_ID)
        {
            if (fusedObjectList[i].priority < worstPriority)
            {
                worstPriority = fusedObjectList[i].priority;
                *objectIndex = i;
            }
        }
        else
        {
            worstPriority = (-1.f) * MAX_PRIORITY;
            *objectIndex = i;

            break;
        }
    }

    return worstPriority;
}

u8_t GetAvailableId(const FusedObject_t* fusedObjectList)
{
    u8_t tentId;
    u8_t availId = INVALID_ID;
    
    for (tentId = (INVALID_ID + 1u); tentId < MAX_ID; tentId++)
    {
        if (!IsIdUsed(fusedObjectList, tentId))
        {
            availId = tentId;
            break;
        }
    }

    return availId;
}

u8_t IsIdUsed(const FusedObject_t* fusedObjectList, const u8_t id)
{
    u8_t i;
    u8_t isUsed = FALSE;

    for (i = 0u; i < NUM_FUSED_OBJ; i++)
    {
        if (fusedObjectList[i].id == id)
        {
            isUsed = TRUE;
            break;
        }
    }

    return isUsed;
}

u8_t IsInsideAcceptanceGate(const PrefusedObject_t* prefusedObject, const FusedObject_t* fusedObjectList, u8_t* pairIndex)
{
    u8_t i;
    f32_t bestGatingValue, gatingValue;
    
    bestGatingValue = INVALID_GATING_VALUE;

    for (i = 0u; i < NUM_FUSED_OBJ; i++)
    {
        if (fusedObjectList[i].id != INVALID_ID)
        {
            gatingValue = GetGatingValue(prefusedObject, &fusedObjectList[i]);

            if (gatingValue > bestGatingValue)
            {
                *pairIndex = i;
                bestGatingValue = gatingValue;
            }
        }
    }

    return (bestGatingValue > totalGatingValueMinLimit);
}

f32_t GetGatingValue(const PrefusedObject_t* prefusedObject, const FusedObject_t* fusedObject)
{
    u8_t i;
    f32_t similarityValue;
    f32_t gatingValue = 0.f;
    
    for (i = 0u; i < KALMAN_STATES; i++)
    {
        similarityValue = GetSimilarityValue(
            prefusedObject->plot.Z[i],
            fusedObject->track.X[i],
            prefusedObject->plot.R[(KALMAN_STATES * i) + i],
            fusedObject->track.P[(KALMAN_STATES * i) + i]);

        similarityValue *= gatingWeights[i];
    
        if (similarityValue > STATE_GATING_VALUE_MIN_LIMIT)
        {
            gatingValue += similarityValue;
        }
        else
        {
            gatingValue = INVALID_GATING_VALUE;
            break;
        }
    }

    return gatingValue;
}

u8_t IsObjectLost(const FusedObject_t* fusedObject)
{
    u8_t i;
    u16_t seenSum = 0u;

    for (i = 0u; i < (sizeof(fusedObject->seenThisCycle) / sizeof(u8_t)); i++)
    {
        seenSum += fusedObject->seenThisCycle[i];
    }

    return (seenSum == 0u);
}

u8_t IsObjectCoastable(FusedObject_t* fusedObject)
{
    return (fusedObject->lostCounter <= MAX_COASTING_CYCLES);
}

/***************************** Public Functions ******************************/

f32_t GetObjectPriority(const f32_t dist_x, const f32_t dist_y)
{
    return (MAX_PRIORITY - GetRange(dist_x, dist_y));
}

void CreatePrefusedObject(PrefusedObject_t* prefusedObject, const Sensor_t* pSensor, f32_t posX, f32_t posY, f32_t velX, f32_t velY)
{
    f32_t varRange = SIGMA_RANGE * SIGMA_RANGE;
    f32_t varDoppler = SIGMA_DOPPLER * SIGMA_DOPPLER;
    f32_t varBearing = DEG2RAD(SIGMA_BEARING) * DEG2RAD(SIGMA_BEARING);
    f32_t varBase = SIGMA_BASE * SIGMA_BASE;

    prefusedObject->valid = TRUE;

    prefusedObject->sensor = pSensor;

    prefusedObject->plot.Z[STATE_X] = posX;
    prefusedObject->plot.Z[STATE_Y] = posY;
    prefusedObject->plot.Z[STATE_VX] = velX;
    prefusedObject->plot.Z[STATE_VY] = velY;

    ApplySensorTF(prefusedObject);

    prefusedObject->plot.R[(KALMAN_STATES * STATE_X) + STATE_X] = GetVarX(prefusedObject->plot.Z[STATE_X], prefusedObject->plot.Z[STATE_Y], varRange, varBearing, varBase);
    prefusedObject->plot.R[(KALMAN_STATES * STATE_Y) + STATE_Y] = GetVarY(prefusedObject->plot.Z[STATE_X], prefusedObject->plot.Z[STATE_Y], varRange, varBearing, varBase);
    prefusedObject->plot.R[(KALMAN_STATES * STATE_VX) + STATE_VX] = varDoppler;
    prefusedObject->plot.R[(KALMAN_STATES * STATE_VY) + STATE_VY] = varDoppler;

    prefusedObject->plot.weight = GetBearingConfidence(prefusedObject->plot.Z[STATE_X], prefusedObject->plot.Z[STATE_Y], pSensor);
    
    prefusedObject->priority = GetObjectPriority(prefusedObject->plot.Z[STATE_X], prefusedObject->plot.Z[STATE_Y]);
}

void AssociatePrefusedObject(const PrefusedObject_t* prefusedObject, FusedObject_t* fusedObjectList)
{
    u8_t pairIndex;

    if (IsInsideAcceptanceGate(prefusedObject, fusedObjectList, &pairIndex))
    {
        fusedObjectList[pairIndex].seenThisCycle[prefusedObject->sensor->type] = 1;

        FuseTrack(&fusedObjectList[pairIndex].track, &prefusedObject->plot);
    }
    else
    {
        CreateFusedObject(fusedObjectList, prefusedObject);
    }
}

void CheckObjectsForPruning(FusedObject_t* fusedObject1, FusedObject_t* fusedObject2)
{
    FusedObject_t* objectToPrune;

    if (((f32_t)(fabs(fusedObject1->track.X[STATE_X] - fusedObject2->track.X[STATE_X])) <= PRUNE_LIMIT_X) &&
        ((f32_t)(fabs(fusedObject1->track.X[STATE_Y] - fusedObject2->track.X[STATE_Y])) <= PRUNE_LIMIT_Y) &&
        ((f32_t)(fabs(fusedObject1->track.X[STATE_VX] - fusedObject2->track.X[STATE_VX])) <= PRUNE_LIMIT_VX) &&
        ((f32_t)(fabs(fusedObject1->track.X[STATE_VY] - fusedObject2->track.X[STATE_VY])) <= PRUNE_LIMIT_VY))
    {
        if (fusedObject1->priority > fusedObject2->priority)
        {
            objectToPrune = fusedObject2;
        }
        else
        {
            objectToPrune = fusedObject1;
        }

        ResetFusedObject(objectToPrune);
    }
}

void MaintainObject(FusedObject_t* fusedObject)
{
    fusedObject->lifetimeCounter = (fusedObject->lifetimeCounter + 1) % U16_MAX;

    /* Don't update lost counter if it's the first cycle of the object! */
    if (fusedObject->lifetimeCounter > 1u)
    {
        if (IsObjectLost(fusedObject))
        {
            fusedObject->lostCounter = (fusedObject->lostCounter + 1) % U8_MAX;

            if (!IsObjectCoastable(fusedObject))
            {
                ResetFusedObject(fusedObject);
            }
        }
        else
        {
            fusedObject->lostCounter = 0u;
        }
    }

    (void)memset(fusedObject->seenThisCycle, 0, sizeof(u8_t) * NUM_SENSORS);
}

u8_t IsTentativeObjectConfirmed(const FusedObject_t* fusedObject)
{
    u8_t confirmed = FALSE;

    if ((fusedObject->id != INVALID_ID) &&
        (fusedObject->lifetimeCounter >= MIN_LIFETIME_TX_CYCLES))
    {
        confirmed = TRUE;
    }

    return confirmed;
}
