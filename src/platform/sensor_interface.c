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
  * Registers all the sensors of the system and implements the functions needed
  * to extract and convert the raw CAN frames from the sensors to objects.
  */

/******************************** Inclusions *********************************/

#include <string.h>

#include "sensor_interface.h"

/************************ Static Function Prototypes *************************/

/**
  * @brief Creates a prefused object from a CAN frame.
  * @details If a frame was not received in this cycle, the prefused object's
  *          attributes take the default values. Otherwise the frame's signals
  *          are extracted into decimal values and then converted to the
  *          physical domain.
  * @param frameReceived Whether a CAN frame was received this cycle.
  * @param canData The data contained in the CAN frame.
  * @param prefusedObject The prefused object to be filled.
  * @return Void.
  */
static void GetFrontRadarObject(const u8_t frameReceived, const u8_t* canData, BaseObject_t* prefusedObject);

/**
  * @brief Creates a prefused object from a CAN frame.
  * @details If a frame was not received in this cycle, the prefused object's
  *          attributes take the default values. Otherwise the frame's signals
  *          are extracted into decimal values and then converted to the
  *          physical domain.
  * @param frameReceived Whether a CAN frame was received this cycle.
  * @param canData The data contained in the CAN frame.
  * @param prefusedObject The prefused object to be filled.
  * @return Void.
  */
static void GetRearRadarObject(const u8_t frameReceived, const u8_t* canData, BaseObject_t* prefusedObject);

/***************************** Static Variables ******************************/

/** The list that contains all the input sensors of the system. */
static Sensor_t sensorList[NUM_SENSORS] = 
{
    {
        /* .type = */ RADAR,  /* Front Left */
        /* .tf = */
        {
            /* .canX = */ 0.f,
            /* .canY = */ 0.f,
            /* .x = */ -0.5f,
            /* .y = */ 0.5f,
            /* .mounting = */ 70.f,
            /* .fov = */ 140.f,
        },
        /* .objects = */
        {
            /* .index = */ 0u,
            /* .length = */ 12u,
            /* .GetObject = */ GetFrontRadarObject,
        },
    },
    {
        /* .type = */ RADAR,  /* Front Right */
        /* .tf = */
        {
            /* .canX = */ 0.f,
            /* .canY = */ 0.f,
            /* .x = */ -0.5f,
            /* .y = */ -0.5f,
            /* .mounting = */ -70.f,
            /* .fov = */ 140.f,
        },
        /* .objects = */
        {
            /* .index = */ 12u,
            /* .length = */ 12u,
            /* .GetObject = */ GetFrontRadarObject,
        },
    },
    {
        /* .type = */ RADAR,  /* Rear Right */
        /* .tf = */
        {
            /* .canX = */ -5.f,
            /* .canY = */ 0.f,
            /* .x = */ -5.f,
            /* .y = */ -0.5f,
            /* .mounting = */ -135.f,
            /* .fov = */ 140.f,
        },
        /* .objects = */
        {
            /* .index = */ 0u,
            /* .length = */ 0u,
            /* .GetObject = */ GetRearRadarObject,
        },
    },
    {
        /* .type = */ RADAR,  /* Rear Left */
        /* .tf = */
        {
            /* .canX = */ -5.f,
            /* .canY = */ 0.f,
            /* .x = */ -5.f,
            /* .y = */ 0.5f,
            /* .mounting = */ 135.f,
            /* .fov = */ 140.f,
        },
        /* .objects = */
        {
            /* .index = */ 0u,
            /* .length = */ 0u,
            /* .GetObject = */ GetRearRadarObject,
        },
    },
};

/***************************** Static Functions ******************************/

void GetFrontRadarObject(const u8_t frameReceived, const u8_t* canData, BaseObject_t* prefusedObject)
{
    if (frameReceived)
    {
        prefusedObject->valid = (RX_FRONT_OBJECT_ID_DEC2PHYS(GET_RX_ID(canData)) == 0u) ? FALSE : TRUE;
        prefusedObject->posX = RX_FRONT_OBJECT_DISTANCE_X_DEC2PHYS(GET_RX_DISTANCE_X(canData));
        prefusedObject->posY = RX_FRONT_OBJECT_DISTANCE_Y_DEC2PHYS(GET_RX_DISTANCE_Y(canData));
        prefusedObject->velX = RX_FRONT_OBJECT_VELOCITY_X_DEC2PHYS(GET_RX_VELOCITY_X(canData));
        prefusedObject->velY = RX_FRONT_OBJECT_VELOCITY_Y_DEC2PHYS(GET_RX_VELOCITY_Y(canData));
    }
    else
    {
        prefusedObject->valid = FALSE;
        prefusedObject->posX = RX_FRONT_OBJECT_DISTANCE_X_UNKNOWN;
        prefusedObject->posY = RX_FRONT_OBJECT_DISTANCE_Y_UNKNOWN;
        prefusedObject->velX = RX_FRONT_OBJECT_VELOCITY_X_UNKNOWN;
        prefusedObject->velY = RX_FRONT_OBJECT_VELOCITY_Y_UNKNOWN;
    }
}

void GetRearRadarObject(const u8_t frameReceived, const u8_t* canData, BaseObject_t* prefusedObject)
{
    if (frameReceived)
    {
        prefusedObject->valid = (RX_REAR_OBJECT_ID_DEC2PHYS(GET_RX_ID(canData)) == 0u) ? FALSE : TRUE;
        prefusedObject->posX = RX_REAR_OBJECT_DISTANCE_X_DEC2PHYS(GET_RX_DISTANCE_X(canData));
        prefusedObject->posY = RX_REAR_OBJECT_DISTANCE_Y_DEC2PHYS(GET_RX_DISTANCE_Y(canData));
        prefusedObject->velX = RX_REAR_OBJECT_VELOCITY_X_DEC2PHYS(GET_RX_VELOCITY_X(canData));
        prefusedObject->velY = RX_REAR_OBJECT_VELOCITY_Y_DEC2PHYS(GET_RX_VELOCITY_Y(canData));
    }
    else
    {
        prefusedObject->valid = FALSE;
        prefusedObject->posX = RX_REAR_OBJECT_DISTANCE_X_UNKNOWN;
        prefusedObject->posY = RX_REAR_OBJECT_DISTANCE_Y_UNKNOWN;
        prefusedObject->velX = RX_REAR_OBJECT_VELOCITY_X_UNKNOWN;
        prefusedObject->velY = RX_REAR_OBJECT_VELOCITY_Y_UNKNOWN;
    }
}

/******************************* Public Functions ****************************/

void InitializeSensorInterface(Sensor_t* pSensorList)
{
    (void)memcpy(pSensorList, sensorList, sizeof(Sensor_t) * (u32_t)NUM_SENSORS);
}

u8_t GetSensorFromIndex(const u8_t index, Sensor_t** sensor)
{
    u8_t i;
    u8_t found = FALSE;

    for (i = 0; i < NUM_SENSORS; i++)
    {
        if (sensorList[i].objects.length != 0u &&
          index >= sensorList[i].objects.index &&
          index < (sensorList[i].objects.index + sensorList[i].objects.length))
        {
            *sensor = &sensorList[i];

            found = TRUE;
            break;
        }
    }

    return found;
}
