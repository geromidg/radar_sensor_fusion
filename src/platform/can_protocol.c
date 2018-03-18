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
  * Defines the Rx and Tx CAN matrices that hold the received and transmitted
  * CAN IDs. The purpose is to uniquely map each CAN ID to one and only one
  * list element. The list is abstract and can be a frame list, object list etc.
  */

/******************************** Inclusions *********************************/

#include "base_types.h"
#include "platform_params.h"
#include "sensor_interface.h"

#include "can_protocol.h"

/***************************** Static Variables ******************************/

/** The matrix containing the Rx CAN IDs.
  * It is used to map a frame/object list element to a CAN ID */
static const u16_t rxMatrix[NUM_RX_OBJS] =
{
    0x100u,  /* Front Left */
    0x101u,
    0x102u,
    0x103u,
    0x104u,
    0x105u,
    0x200u,  /* Front Right */
    0x201u,
    0x202u,
    0x203u,
    0x204u,
    0x205u,
    0x300u,  /* Rear Right */
    0x301u,
    0x302u,
    0x303u,
    0x304u,
    0x305u,
    0x400u,  /* Rear Left */
    0x401u,
    0x402u,
    0x403u,
    0x404u,
    0x405u,
};

/** The matrix containing the Tx CAN IDs.
  * It is used to map a frame/object list element to a CAN ID */
static const u16_t txMatrix[NUM_TX_OBJS] =
{
    0x500u,
    0x501u,
    0x502u,
    0x503u,
    0x504u,
    0x505u,
    0x506u,
    0x507u,
    0x508u,
    0x509u,
    0x50Au,
    0x50Bu,
    0x50Cu,
    0x50Du,
    0x50Eu,
    0x50Fu,
};

/************************ Static Function Prototypes *************************/

/**
  * @brief Finds the CAN ID associated with an index.
  * @details Searches in all elements of a CAN matrix to find the pair.
  * @param matrix The input CAN matrix to be searched.
  * @param size The size of the matrix.
  * @param index The input index.
  * @param canId The output CAN ID.
  * @return Whether the search was successful.
  */
static u8_t MapIndexToId(const u16_t* matrix, const u8_t size, const u8_t index, u16_t* canId);

/**
  * @brief Finds the index associated with a CAN ID.
  * @details Searches in all elements of a CAN matrix to find the pair.
  * @param matrix The input CAN matrix to be searched.
  * @param size The size of the matrix.
  * @param index The output index.
  * @param canId The input CAN ID.
  * @return Whether the search was successful.
  */
static u8_t MapIdToIndex(const u16_t* matrix, const u8_t size, u8_t* index, const u16_t canId);

/***************************** Static Functions ******************************/

u8_t MapIndexToId(const u16_t* matrix, const u8_t size, const u8_t index, u16_t* canId)
{
    u8_t matrixIndex;
    u8_t found = FALSE;

    for (matrixIndex = 0; matrixIndex < size; matrixIndex++)
    {
        if (matrixIndex == index)
        {
            *canId = matrix[matrixIndex];

            found = TRUE;
            break;
        }
    }

    return found;
}

u8_t MapIdToIndex(const u16_t* matrix, const u8_t size, u8_t* index, const u16_t canId)
{
    u8_t found = FALSE;

    for (*index = 0; *index < size; (*index)++)
    {
        if (canId == matrix[*index])
        {
            found = TRUE;
            break;
        }
    }

    return found;
}

/******************************* Public Functions ****************************/

u8_t MapIndexToIdRx(const u8_t index, u16_t* canId)
{
    return MapIndexToId(rxMatrix, NUM_RX_OBJS, index, canId);
}

u8_t MapIdToIndexRx(u8_t* index, const u16_t canId)
{
    return MapIdToIndex(rxMatrix, NUM_RX_OBJS, index, canId);
}

u8_t MapIndexToIdTx(const u8_t index, u16_t* canId)
{
    return MapIndexToId(txMatrix, NUM_TX_OBJS, index, canId);
}

u8_t MapIdToIndexTx(u8_t* index, const u16_t canId)
{
    return MapIdToIndex(txMatrix, NUM_TX_OBJS, index, canId);
}
