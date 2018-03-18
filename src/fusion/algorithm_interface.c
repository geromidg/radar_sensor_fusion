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
  * Provides functionality to init, run and get feedback from the module.
  * Depends only on the datatypes of the inputs. No external call is made.
  */

/******************************** Inclusions *********************************/

#include <string.h>

#include "sensor_interface.h"

#include "constants.h"
#include "platform_params.h"
#include "config.h"
#include "fusion.h"
#include "fusion_utils.h"

#include "algorithm_interface.h"

/***************************** Static Variables ******************************/

/** The list that contains all the input sensors of the system. */
static Sensor_t sensorList[NUM_SENSORS];

/** The list (buffer) that contains all the objects that have been fed by an external module. */
static BaseObject_t inputObjectList[NUM_PREFUSED_OBJ];

/** The list (buffer) that contains all the objects that are ready to be outputtted to an external module. */
static BaseObject_t outputObjectList[NUM_FUSED_OBJ];

/** The list that contains all the prefused objects that are inputted to the algo. */
static PrefusedObject_t prefusedObjectList[NUM_PREFUSED_OBJ];

/** The list that contains all the fused objects that are tracked by the algo. */
static FusedObject_t fusedObjectList[NUM_FUSED_OBJ];

/************************ Static Function Prototypes *************************/

/**
  * @brief Cycles through the input object list and adds the valid objects to the prefusedObjectList.
  * @details Determines if an input object is valid and tracked and determines the sensor it comes from.
  * @return Void.
  */
static void PrepareInputObjects(void);

/**
  * @brief Converts an input object to a prefused object (native).
  * @details The input object's information (pos/vel) as well as the sensor that is coming from are stored to the prefused object.
  * @param prefusedObject A prefused object used as an input to the algo.
  * @param inputObject An object of platform type, coming from an external module.
  * @param sensor A sensor of the system. Its type and geometry is contained.
  * @return Void.
  */
static void AddInputObject(PrefusedObject_t* prefusedObject, const BaseObject_t* inputObject, const Sensor_t* sensor);

/**
  * @brief Cycles through the fusedObjectList (output of the algo) and adds the valid object to the output object list.
  * @details For each valid fused object, it checks if the tentative object has been confirmed (is stable enough) and adds it.
  * @todo Sort according to TTC.
  * @return Void.
  */
static void PrepareOutputObjects(void);

/**
  * @brief Converts a fused object (native) to an output object.
  * @details The fused object's information (pos/vel) and id are stored to the output object.
  * @param outputObject An object of platform type, targeted to an external module.
  * @param fusedObject A fused object that is outputted from the algo.
  * @todo Use the vehicle_speed from VehDyn data to determine if object is moving.
  * @todo Add quality signals.
  * @return Void.
  */
static void AddOutputObject(BaseObject_t* outputObject, const FusedObject_t* fusedObject);

/***************************** Static Functions ******************************/

void PrepareInputObjects(void)
{
    u8_t i;
    Sensor_t* sensor = NULL;
    u8_t numInputObjects = 0u;

    (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

    for (i = 0u; i < NUM_PREFUSED_OBJ; i++)
    {
        if (inputObjectList[i].valid && GetSensorFromIndex(i, &sensor))
        {          
            AddInputObject(&prefusedObjectList[numInputObjects], &inputObjectList[i], sensor);

            numInputObjects++;
        }
    }
}

void AddInputObject(PrefusedObject_t* prefusedObject, const BaseObject_t* inputObject, const Sensor_t* sensor)
{
    f32_t x = inputObject->posX;
    f32_t y = inputObject->posY;
    f32_t vx = inputObject->velX;
    f32_t vy = inputObject->velY;

    CreatePrefusedObject(prefusedObject, sensor, x, y, vx, vy);
}

void PrepareOutputObjects(void)
{
    u8_t i;
    u8_t numOutputObjects = 0u;

    (void)memset(outputObjectList, 0, sizeof(BaseObject_t) * (u32_t)NUM_FUSED_OBJ);

    for (i = 0u; i < NUM_FUSED_OBJ; i++)
    {
        if (IsTentativeObjectConfirmed(&fusedObjectList[i]))
        {
            AddOutputObject(&outputObjectList[numOutputObjects], &fusedObjectList[i]);

            numOutputObjects++;
        }
    }
}

void AddOutputObject(BaseObject_t* outputObject, const FusedObject_t* fusedObject)
{
    outputObject->valid = (fusedObject->id == INVALID_ID) ? 0 : 1;

    outputObject->posX = fusedObject->track.X[STATE_X];
    outputObject->posY = fusedObject->track.X[STATE_Y];
    outputObject->velX = fusedObject->track.X[STATE_VX];
    outputObject->velY = fusedObject->track.X[STATE_VY];
}

/***************************** Public Functions ******************************/

void InitializeAlgorithm(void)
{
    InitializeSensorInterface(sensorList);

    (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);
    (void)memset(fusedObjectList, 0, sizeof(FusedObject_t) * (u32_t)NUM_FUSED_OBJ);

    InitializeFusion();
}

void RunAlgorithm(const BaseObject_t* pInputObjectList, BaseObject_t* pOutputObjectList)
{
    (void)memcpy(inputObjectList, pInputObjectList, sizeof(BaseObject_t) * (u32_t)NUM_PREFUSED_OBJ);
    PrepareInputObjects();

    RunFusion(prefusedObjectList, fusedObjectList);

    PrepareOutputObjects();
    (void)memcpy(pOutputObjectList, outputObjectList, sizeof(BaseObject_t) * (u32_t)NUM_FUSED_OBJ);
}
