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

#ifndef BASE_TYPES_H
#define BASE_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"

/***************************** Type Definitions ******************************/

/***********
 *** CAN ***
 **********/

/**
  * @struct CanFrame_t
  * @brief A CAN frame.
  */
typedef struct {
    u16_t id;
    u8_t dlc;

    union {
        u8_t data8[8];
        u16_t data16[4];
        u32_t data32[2];
        u64_t data64;
    };
} CanFrame_t;

/*******************
 *** Base Object ***
 ******************/

/**
  * @struct BaseObject_t
  * @brief A base object.
  */
typedef struct {
    u8_t valid;

    f32_t posX;
    f32_t posY;
    f32_t velX;
    f32_t velY;
} BaseObject_t;

/**************
 *** Sensor ***
 *************/

/**
  * @enum SensorType_t
  * @brief The different types of input sensors.
  */
typedef enum {
    RADAR = 0,
} SensorType_t;

/**
  * @struct SensorTF_t
  * @brief A sensor's transformation to the global origin.
  * @todo Add z and range.
  */
typedef struct {
    f32_t canX;
    f32_t canY;
    f32_t x;
    f32_t y;
    f32_t mounting;
    f32_t fov;
} SensorTF_t;

/**
  * @struct SensorObjects_t
  * @brief Contains info regarding the objects associated to a sensor.
  * @todo Set index to invalid value when length is zero.
  */
typedef struct {
    u8_t index;
    u8_t length;
    void (*GetObject)(const u8_t, const u8_t*, BaseObject_t*);
} SensorObjects_t;

/**
  * @struct Sensor_t
  * @brief An input sensor.
  */
typedef struct {
    SensorType_t type;
    SensorTF_t tf;
    SensorObjects_t objects;
} Sensor_t;

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* BASE_TYPES_H */
