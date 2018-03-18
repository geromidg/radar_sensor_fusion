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
  * Contains functionality needed for the communication of the platform's main core
  * and different modules. All the functions are called directly from the scheduler's tasks.
  *
  * For the algorithm module, it receives the prefused frames from CAN, and passes
  * the object list to the module to do the fusion. Once done, it passes the
  * output object list to CAN buffers that are transmitted synchronously.
  *
  * The main role of this module is to convert to and from frame (raw) lists to object lists.
  * The cycle looks like: CAN (Rx) -> prefusedFrameList -> prefusedObjectlist -> fusion
  *                       -> fusedObjectList -> fusedFrameList -> CAN (Tx)
  */

/******************************** Inclusions *********************************/

#include <string.h>

#include "algorithm_interface.h"

#include "base_types.h"
#include "platform_params.h"
#include "can_protocol.h"
#include "can_interface.h"
#include "sensor_interface.h"

#include "main_interface.h"

/***************************** Static Variables ******************************/

/** The list that contains all the input sensors of the system. */
static Sensor_t sensorList[NUM_SENSORS];

/** The flag array containing the reception status of each frame for a cycle. */
static u8_t prefusedObjectReceived[NUM_RX_OBJS];

/** The list that holds the prefused frames received from the CAN module. */
static CanFrame_t prefusedFrameList[NUM_RX_OBJS];

/** The list that contains the prefused objects and is inputted to the algorithm. */
static BaseObject_t prefusedObjectList[NUM_RX_OBJS];

/** The list that contains the fused objects and is outputted from the algorithm. */
static BaseObject_t fusedObjectList[NUM_TX_OBJS];

/** The CAN frame that is used to transmit the fused objects one by one. */
static CanFrame_t txFrame;

/************************ Static Function Prototypes *************************/

/**
  * @brief Reset all the buffers related to prefusion data.
  * @details All the buffers are cleared in order to prepare for the reception
  *          of new data in the next cycle.
  * @return Void.
  */
static void ResetPrefusedBuffers(void);

/**
  * @brief Reset all the buffers related to fusion data.
  * @details All the buffers are cleared in order to prepare for the reception
  *          of new data in the next cycle.
  * @return Void.
  */
static void ResetFusedBuffers(void);

/***************************** Static Functions ******************************/

void ResetPrefusedBuffers(void)
{
    (void)memset(prefusedFrameList, 0, sizeof(CanFrame_t) * (u32_t)NUM_RX_OBJS);
    (void)memset(prefusedObjectList, 0, sizeof(BaseObject_t) * (u32_t)NUM_RX_OBJS);
}

void ResetFusedBuffers(void)
{
    (void)memset(fusedObjectList, 0, sizeof(BaseObject_t) * (u32_t)NUM_TX_OBJS);
}

/***************************** Public Functions ******************************/

void Initialize(void)
{
    InitializeCanInterface();
    InitializeSensorInterface(sensorList);

    InitializeAlgorithm();
}

void CopyPrefusedData(void)
{
    u8_t i;
    Sensor_t* sensor = NULL;

    CopyPrefusedFrameList(prefusedObjectReceived, prefusedFrameList);

    for (i = 0; i < NUM_RX_OBJS; i++)
    {
        if (GetSensorFromIndex(i, &sensor))
        {
            sensor->objects.GetObject(prefusedObjectReceived[i], prefusedFrameList[i].data8, &prefusedObjectList[i]);
        }
    }
}

void ExecuteFusionAlgo(void)
{
    RunAlgorithm(prefusedObjectList, fusedObjectList);
}

void PublishFusedData(void)
{
    u8_t index;

    for (index = 0; index < NUM_TX_OBJS; index++)
    {
        u8_t valid;
        u16_t distanceX;
        u16_t distanceY;
        u16_t velocityX;
        u16_t velocityY;

        if (fusedObjectList[index].valid)
        {
            valid = TX_OBJECT_VALID_PHYS2DEC(fusedObjectList[index].valid);
            distanceX = TX_OBJECT_DISTANCE_X_PHYS2DEC(CLAMP(fusedObjectList[index].posX, TX_OBJECT_DISTANCE_X_MIN, TX_OBJECT_DISTANCE_X_MAX));
            distanceY = TX_OBJECT_DISTANCE_Y_PHYS2DEC(CLAMP(fusedObjectList[index].posY, TX_OBJECT_DISTANCE_Y_MIN, TX_OBJECT_DISTANCE_Y_MAX));
            velocityX = TX_OBJECT_VELOCITY_X_PHYS2DEC(CLAMP(fusedObjectList[index].velX, TX_OBJECT_VELOCITY_X_MIN, TX_OBJECT_VELOCITY_X_MAX));
            velocityY = TX_OBJECT_VELOCITY_Y_PHYS2DEC(CLAMP(fusedObjectList[index].velY, TX_OBJECT_VELOCITY_Y_MIN, TX_OBJECT_VELOCITY_Y_MAX));
        }
        else
        {
            valid = TX_OBJECT_VALID_UNKNOWN;
            distanceX = TX_OBJECT_DISTANCE_X_UNKNOWN;
            distanceY = TX_OBJECT_DISTANCE_Y_UNKNOWN;
            velocityX = TX_OBJECT_VELOCITY_X_UNKNOWN;
            velocityY = TX_OBJECT_VELOCITY_Y_UNKNOWN;
        }

        if (MapIndexToIdTx(index, &txFrame.id))
        {
            txFrame.dlc = 8u;
            txFrame.data64 = 0u;
            
            SET_TX_VALID(txFrame.data8, valid);
            SET_TX_DISTANCE_X(txFrame.data8, distanceX);
            SET_TX_DISTANCE_Y(txFrame.data8, distanceY);
            SET_TX_VELOCITY_X(txFrame.data8, velocityX);
            SET_TX_VELOCITY_Y(txFrame.data8, velocityY);

            TransmitCanFrame(&txFrame);
        }
    }

    ResetRxBuffers();
    ResetPrefusedBuffers();
    ResetFusedBuffers();
}
