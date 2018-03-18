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

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"

#include "platform_params.h"

/***************************** Macro Definitions *****************************/

/** The size of the prefused and fused frame/message/object lists. */
#define NUM_TX_OBJS (16)

/*******************
 *** Extractions ***
 ******************/

#define SET_TX_VALID(canData, var) \
    ( (canData)[0] = ( (canData)[0] & (u8_t)0xFE) | ((u8_t)((var) & (u8_t)0x1)) )
  
#define SET_TX_DISTANCE_X(canData, var) \
    ( (canData)[1] = ( (canData)[1] & (u8_t)0x3) | ((u8_t)(((var) & (u8_t)0x3f) << 2u)), \
      (canData)[2] = ( (canData)[2] & (u8_t)0xc0) | (((u8_t)((var) >> 6u)) & (u8_t)0x3f) )

#define SET_TX_DISTANCE_Y(canData, var) \
    ( (canData)[2] = ( (canData)[2] & (u8_t)0x3f) | ((u8_t)(((var) & (u8_t)0x3) << 6u)), \
      (canData)[3] = ((u8_t)((var) >> 2u)), \
      (canData)[4] = ( (canData)[4] & (u8_t)0xfc) | (((u8_t)((var) >> 10u)) & (u8_t)0x3) )

#define SET_TX_VELOCITY_X(canData, var) \
    ( (canData)[4] = ( (canData)[4] & (u8_t)0x3) | ((u8_t)(((var) & (u8_t)0x3f) << 2u)), \
      (canData)[5] = ( (canData)[5] & (u8_t)0xe0) | (((u8_t)((var) >> 6u)) & (u8_t)0x1f) )

#define SET_TX_VELOCITY_Y(canData, var) \
    ( (canData)[5] = ( (canData)[5] & (u8_t)0x1f) | ((u8_t)(((var) & (u8_t)0x7) << 5u)), \
      (canData)[6] = ((u8_t)((((var))) >> 3u)) )

/*******************
 *** Conversions ***
 ******************/

#define TX_OBJECT_VALID_MIN        (0u)
#define TX_OBJECT_VALID_MAX        (1u)
#define TX_OBJECT_VALID_OFFSET     (0u)
#define TX_OBJECT_VALID_INV_FACTOR (1u)
#define TX_OBJECT_VALID_FACTOR     (1u)
#define TX_OBJECT_VALID_CAN_MIN    (0x0u)
#define TX_OBJECT_VALID_CAN_MAX    (0x1u)
#define TX_OBJECT_VALID_UNKNOWN    (0x0u)
#define TX_OBJECT_VALID_PHYS2DEC(value) ((u8_t)((((value) - TX_OBJECT_VALID_OFFSET) * TX_OBJECT_VALID_INV_FACTOR)));

#define TX_OBJECT_DISTANCE_X_MIN        (-200.f)
#define TX_OBJECT_DISTANCE_X_MAX        (200.f)
#define TX_OBJECT_DISTANCE_X_OFFSET     (-200.f)
#define TX_OBJECT_DISTANCE_X_INV_FACTOR (10.f)
#define TX_OBJECT_DISTANCE_X_FACTOR     (0.1f)
#define TX_OBJECT_DISTANCE_X_CAN_MIN    (0x0u)
#define TX_OBJECT_DISTANCE_X_CAN_MAX    (0xFFFu)
#define TX_OBJECT_DISTANCE_X_UNKNOWN    (0xFFFu)
#define TX_OBJECT_DISTANCE_X_PHYS2DEC(value) ((u16_t)((((value) - TX_OBJECT_DISTANCE_X_OFFSET) * TX_OBJECT_DISTANCE_X_INV_FACTOR) + 0.5f));

#define TX_OBJECT_DISTANCE_Y_MIN        (-200.f)
#define TX_OBJECT_DISTANCE_Y_MAX        (200.f)
#define TX_OBJECT_DISTANCE_Y_OFFSET     (-200.f)
#define TX_OBJECT_DISTANCE_Y_INV_FACTOR (10.f)
#define TX_OBJECT_DISTANCE_Y_FACTOR     (0.1f)
#define TX_OBJECT_DISTANCE_Y_CAN_MIN    (0x0u)         
#define TX_OBJECT_DISTANCE_Y_CAN_MAX    (0xFFFu)      
#define TX_OBJECT_DISTANCE_Y_UNKNOWN    (0xFFFu)    
#define TX_OBJECT_DISTANCE_Y_PHYS2DEC(value) ((u16_t)((((value) - TX_OBJECT_DISTANCE_Y_OFFSET) * TX_OBJECT_DISTANCE_Y_INV_FACTOR) + 0.5f));

#define TX_OBJECT_VELOCITY_X_MIN        (-100.f)
#define TX_OBJECT_VELOCITY_X_MAX        (100.f)
#define TX_OBJECT_VELOCITY_X_OFFSET     (-100.f)
#define TX_OBJECT_VELOCITY_X_INV_FACTOR (10.f)
#define TX_OBJECT_VELOCITY_X_FACTOR     (0.1f)
#define TX_OBJECT_VELOCITY_X_CAN_MIN    (0x0u)
#define TX_OBJECT_VELOCITY_X_CAN_MAX    (0xFFFu)
#define TX_OBJECT_VELOCITY_X_UNKNOWN    (0xFFFu)
#define TX_OBJECT_VELOCITY_X_PHYS2DEC(value) ((u16_t)((((value) - TX_OBJECT_VELOCITY_X_OFFSET) * TX_OBJECT_VELOCITY_X_INV_FACTOR) + 0.5f));

#define TX_OBJECT_VELOCITY_Y_MIN        (-100.f)
#define TX_OBJECT_VELOCITY_Y_MAX        (100.f)
#define TX_OBJECT_VELOCITY_Y_OFFSET     (-100.f)
#define TX_OBJECT_VELOCITY_Y_INV_FACTOR (10.f)
#define TX_OBJECT_VELOCITY_Y_FACTOR     (0.1f)
#define TX_OBJECT_VELOCITY_Y_CAN_MIN    (0x0u)         
#define TX_OBJECT_VELOCITY_Y_CAN_MAX    (0xFFFu)      
#define TX_OBJECT_VELOCITY_Y_UNKNOWN    (0xFFFu)     
#define TX_OBJECT_VELOCITY_Y_PHYS2DEC(value) ((u16_t)((((value) - TX_OBJECT_VELOCITY_Y_OFFSET) * TX_OBJECT_VELOCITY_Y_INV_FACTOR) + 0.5f));

/***************************** Public Functions ******************************/

/**
  * @brief Map a list index to the id of an Rx frame.
  * @details Searches in a predefined CAN matrix for the given index.
  *          If found, it returns its pair CAN id.
  * @param index The index of a list.
  * @param canId The id of a CAN frame.
  * @return Whether the mapping was successful.
  */
u8_t MapIndexToIdRx(const u8_t index, u16_t* canId);

/**
  * @brief Map a list index to the id of an Rx frame.
  * @details Searches in a predefined CAN matrix for the given CAN id.
  *          If found, it returns its pair index.
  * @param index The index of a list.
  * @param canId The id of a CAN frame.
  * @return Whether the mapping was successful.
  */
u8_t MapIdToIndexRx(u8_t* index, const u16_t canId);

/**
  * @brief Map a list index to the id of an Rx frame.
  * @details Searches in a predefined CAN matrix for the given index.
  *          If found, it returns its pair CAN id.
  * @param index The index of a list.
  * @param canId The id of a CAN frame.
  * @return Whether the mapping was successful.
  */
u8_t MapIndexToIdTx(const u8_t index, u16_t* canId);

/**
  * @brief Map a list index to the id of an Rx frame.
  * @details Searches in a predefined CAN matrix for the given CAN id.
  *          If found, it returns its pair index.
  * @param index The index of a list.
  * @param canId The id of a CAN frame.
  * @return Whether the mapping was successful.
  */
u8_t MapIdToIndexTx(u8_t* index, const u16_t canId);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* CAN_PROTOCOL_H */
