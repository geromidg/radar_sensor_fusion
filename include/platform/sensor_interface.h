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

#ifndef SENSOR_INTERFACE
#define SENSOR_INTERFACE

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"

#include "base_types.h"
#include "platform_params.h"

/***************************** Macro Definitions *****************************/

/** The number of the input sensors. */
#define NUM_SENSORS (4u)

/** The numbers of the total input objects (summed from all sensors). */
#define NUM_RX_OBJS (24)  /* 6+6 Front & 6+6 Rear */

/*******************
 *** Extractions ***
 ******************/

#define GET_RX_ID(canData) \
    ( (((u8_t)((canData)[1] & (u8_t)0xf0)) >> 4u) | \
      (((u8_t)((canData)[2] & (u8_t)0x03)) << 4u) )

#define GET_RX_DISTANCE_X(canData) \
    ( (((u16_t)((canData)[2] & (u8_t)0xfc)) >> 2u) | \
      (((u16_t)((canData)[3] & (u8_t)0x3f)) << 6u) )

#define GET_RX_DISTANCE_Y(canData) \
    ( (((u16_t)((canData)[3] & (u8_t)0xc0)) >> 6u) | \
      (((u16_t)((canData)[4] & (u8_t)0xff)) << 2u) | \
      (((u16_t)((canData)[5] & (u8_t)0x03)) << 10u) )

#define GET_RX_VELOCITY_X(canData) \
    ( (((u16_t)((canData)[5] & (u8_t)0xfc)) >> 2u) | \
      (((u16_t)((canData)[6] & (u8_t)0x1f)) << 6u) )

#define GET_RX_VELOCITY_Y(canData) \
    ( (((u16_t)((canData)[6] & (u8_t)0xe0)) >> 5u) | \
      (((u16_t)((canData)[7] & (u8_t)0xff)) << 3u) )

/*******************
 *** Conversions ***
 ******************/

#define RX_FRONT_OBJECT_ID_MIN        (0u)
#define RX_FRONT_OBJECT_ID_MAX        (63u)
#define RX_FRONT_OBJECT_ID_OFFSET     (0u)
#define RX_FRONT_OBJECT_ID_INV_FACTOR (1u)
#define RX_FRONT_OBJECT_ID_FACTOR     (1u)
#define RX_FRONT_OBJECT_ID_CAN_MIN    (0x0u)
#define RX_FRONT_OBJECT_ID_CAN_MAX    (0xFFu)
#define RX_FRONT_OBJECT_ID_UNKNOWN    (0xFFu)
#define RX_FRONT_OBJECT_ID_DEC2PHYS(value) ((u8_t)(((u8_t)(value) * RX_FRONT_OBJECT_ID_FACTOR ) + RX_FRONT_OBJECT_ID_OFFSET))

#define RX_FRONT_OBJECT_DISTANCE_X_MIN        (-200.f)
#define RX_FRONT_OBJECT_DISTANCE_X_MAX        (200.f)
#define RX_FRONT_OBJECT_DISTANCE_X_OFFSET     (-200.f)
#define RX_FRONT_OBJECT_DISTANCE_X_INV_FACTOR (10.f)
#define RX_FRONT_OBJECT_DISTANCE_X_FACTOR     (0.1f)
#define RX_FRONT_OBJECT_DISTANCE_X_CAN_MIN    (0x0u)
#define RX_FRONT_OBJECT_DISTANCE_X_CAN_MAX    (0xFFFu)
#define RX_FRONT_OBJECT_DISTANCE_X_UNKNOWN    (0xFFFu)
#define RX_FRONT_OBJECT_DISTANCE_X_DEC2PHYS(value) ((f32_t)(((f32_t)(value) * RX_FRONT_OBJECT_DISTANCE_X_FACTOR ) + RX_FRONT_OBJECT_DISTANCE_X_OFFSET))

#define RX_FRONT_OBJECT_DISTANCE_Y_MIN        (-200.f)
#define RX_FRONT_OBJECT_DISTANCE_Y_MAX        (200.f)
#define RX_FRONT_OBJECT_DISTANCE_Y_OFFSET     (-200.f)
#define RX_FRONT_OBJECT_DISTANCE_Y_INV_FACTOR (10.f)
#define RX_FRONT_OBJECT_DISTANCE_Y_FACTOR     (0.1f)
#define RX_FRONT_OBJECT_DISTANCE_Y_CAN_MIN    (0x0u)
#define RX_FRONT_OBJECT_DISTANCE_Y_CAN_MAX    (0xFFFu)
#define RX_FRONT_OBJECT_DISTANCE_Y_UNKNOWN    (0xFFFu)
#define RX_FRONT_OBJECT_DISTANCE_Y_DEC2PHYS(value) ((f32_t)(((f32_t)(value) * RX_FRONT_OBJECT_DISTANCE_Y_FACTOR ) + RX_FRONT_OBJECT_DISTANCE_Y_OFFSET))

#define RX_FRONT_OBJECT_VELOCITY_X_MIN        (-50.f)
#define RX_FRONT_OBJECT_VELOCITY_X_MAX        (50.f)
#define RX_FRONT_OBJECT_VELOCITY_X_OFFSET     (-50.f)
#define RX_FRONT_OBJECT_VELOCITY_X_INV_FACTOR (10.f)
#define RX_FRONT_OBJECT_VELOCITY_X_FACTOR     (0.1f)
#define RX_FRONT_OBJECT_VELOCITY_X_CAN_MIN    (0x0u)
#define RX_FRONT_OBJECT_VELOCITY_X_CAN_MAX    (0xFFFu)
#define RX_FRONT_OBJECT_VELOCITY_X_UNKNOWN    (0xFFFu)
#define RX_FRONT_OBJECT_VELOCITY_X_DEC2PHYS(value) ((f32_t)(((f32_t) (value) * RX_FRONT_OBJECT_VELOCITY_X_FACTOR) + RX_FRONT_OBJECT_VELOCITY_X_OFFSET))

#define RX_FRONT_OBJECT_VELOCITY_Y_MIN        (-50.f)
#define RX_FRONT_OBJECT_VELOCITY_Y_MAX        (50.f)
#define RX_FRONT_OBJECT_VELOCITY_Y_OFFSET     (-50.f)
#define RX_FRONT_OBJECT_VELOCITY_Y_INV_FACTOR (10.f)
#define RX_FRONT_OBJECT_VELOCITY_Y_FACTOR     (0.1f)
#define RX_FRONT_OBJECT_VELOCITY_Y_CAN_MIN    (0x0u)
#define RX_FRONT_OBJECT_VELOCITY_Y_CAN_MAX    (0xFFFu)
#define RX_FRONT_OBJECT_VELOCITY_Y_UNKNOWN    (0xFFFu)
#define RX_FRONT_OBJECT_VELOCITY_Y_DEC2PHYS(value) ((f32_t)(((f32_t) (value) * RX_FRONT_OBJECT_VELOCITY_Y_FACTOR) + RX_FRONT_OBJECT_VELOCITY_Y_OFFSET))

#define RX_REAR_OBJECT_ID_MIN        (0u)
#define RX_REAR_OBJECT_ID_MAX        (63u)
#define RX_REAR_OBJECT_ID_OFFSET     (0u)
#define RX_REAR_OBJECT_ID_INV_FACTOR (1u)
#define RX_REAR_OBJECT_ID_FACTOR     (1u)
#define RX_REAR_OBJECT_ID_CAN_MIN    (0x0u)
#define RX_REAR_OBJECT_ID_CAN_MAX    (0xFFu)
#define RX_REAR_OBJECT_ID_UNKNOWN    (0xFFu)
#define RX_REAR_OBJECT_ID_DEC2PHYS(value) ((u8_t)(((u8_t)(value) * RX_REAR_OBJECT_ID_FACTOR ) + RX_REAR_OBJECT_ID_OFFSET))

#define RX_REAR_OBJECT_DISTANCE_X_MIN        (-200.f)
#define RX_REAR_OBJECT_DISTANCE_X_MAX        (200.f)
#define RX_REAR_OBJECT_DISTANCE_X_OFFSET     (-200.f)
#define RX_REAR_OBJECT_DISTANCE_X_INV_FACTOR (10.f)
#define RX_REAR_OBJECT_DISTANCE_X_FACTOR     (0.1f)
#define RX_REAR_OBJECT_DISTANCE_X_CAN_MIN    (0x0u)
#define RX_REAR_OBJECT_DISTANCE_X_CAN_MAX    (0xFFFu)
#define RX_REAR_OBJECT_DISTANCE_X_UNKNOWN    (0xFFFu)
#define RX_REAR_OBJECT_DISTANCE_X_DEC2PHYS(value) ((f32_t)(((f32_t)(value) * RX_REAR_OBJECT_DISTANCE_X_FACTOR ) + RX_REAR_OBJECT_DISTANCE_X_OFFSET))

#define RX_REAR_OBJECT_DISTANCE_Y_MIN        (-200.f)
#define RX_REAR_OBJECT_DISTANCE_Y_MAX        (200.f)
#define RX_REAR_OBJECT_DISTANCE_Y_OFFSET     (-200.f)
#define RX_REAR_OBJECT_DISTANCE_Y_INV_FACTOR (10.f)
#define RX_REAR_OBJECT_DISTANCE_Y_FACTOR     (0.1f)
#define RX_REAR_OBJECT_DISTANCE_Y_CAN_MIN    (0x0u)
#define RX_REAR_OBJECT_DISTANCE_Y_CAN_MAX    (0xFFFu)
#define RX_REAR_OBJECT_DISTANCE_Y_UNKNOWN    (0xFFFu)
#define RX_REAR_OBJECT_DISTANCE_Y_DEC2PHYS(value) ((f32_t)(((f32_t)(value) * RX_REAR_OBJECT_DISTANCE_Y_FACTOR ) + RX_REAR_OBJECT_DISTANCE_Y_OFFSET))

#define RX_REAR_OBJECT_VELOCITY_X_MIN        (-50.f)
#define RX_REAR_OBJECT_VELOCITY_X_MAX        (50.f)
#define RX_REAR_OBJECT_VELOCITY_X_OFFSET     (-50.f)
#define RX_REAR_OBJECT_VELOCITY_X_INV_FACTOR (10.f)
#define RX_REAR_OBJECT_VELOCITY_X_FACTOR     (0.1f)
#define RX_REAR_OBJECT_VELOCITY_X_CAN_MIN    (0x0u)
#define RX_REAR_OBJECT_VELOCITY_X_CAN_MAX    (0xFFFu)
#define RX_REAR_OBJECT_VELOCITY_X_UNKNOWN    (0xFFFu)
#define RX_REAR_OBJECT_VELOCITY_X_DEC2PHYS(value) ((f32_t)(((f32_t)(value) * RX_REAR_OBJECT_VELOCITY_X_FACTOR) + RX_REAR_OBJECT_VELOCITY_X_OFFSET))

#define RX_REAR_OBJECT_VELOCITY_Y_MIN        (-50.f)
#define RX_REAR_OBJECT_VELOCITY_Y_MAX        (50.f)
#define RX_REAR_OBJECT_VELOCITY_Y_OFFSET     (-50.f)
#define RX_REAR_OBJECT_VELOCITY_Y_INV_FACTOR (10.f)
#define RX_REAR_OBJECT_VELOCITY_Y_FACTOR     (0.1f)
#define RX_REAR_OBJECT_VELOCITY_Y_CAN_MIN    (0x0u)
#define RX_REAR_OBJECT_VELOCITY_Y_CAN_MAX    (0xFFFu)
#define RX_REAR_OBJECT_VELOCITY_Y_UNKNOWN    (0xFFFu)
#define RX_REAR_OBJECT_VELOCITY_Y_DEC2PHYS(value) ((f32_t)(((f32_t)(value) * RX_REAR_OBJECT_VELOCITY_Y_FACTOR) + RX_REAR_OBJECT_VELOCITY_Y_OFFSET))

/******************************* Public Functions ****************************/

/**
  * @brief Initialize the sensor interface.
  * @details Register all the sensors on a sensorList.
  * @param sensorList The list containing the sensor objects.
  * @return Void.
  */
void InitializeSensorInterface(Sensor_t* sensorList);

/**
  * @brief Map a list index to a sensor.
  * @details Each element of a frame/object list is associated to a sensor
  *          Searches all the sensors to find the sensor associated to a given index.
  * @param index The index of a list.
  * @param sensor The sensor to be returned.
  * @return Whether a sensor was found.
  */
u8_t GetSensorFromIndex(const u8_t index, Sensor_t** sensor);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* SENSOR_INTERFACE */
