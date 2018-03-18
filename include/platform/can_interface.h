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

#ifndef CAN_INTERFACE_H
#define CAN_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"
#include "base_types.h"

/***************************** Public Functions ******************************/
 
/**
  * @brief Initialized the CAN interface.
  * @details Each module that implements this interface is hosted on a different OS
  *          Therefore an initialization of specific buffers/sockets is needed.
  * @return Void.
  */
void InitializeCanInterface(void);

/**
  * @brief Copy the contents of the prefused (Rx) buffers of the CAN module.
  * @details The received and frame lists are copied from the main CAN module
  *          to an external interface, in order to use the data collected so far.
  *          When a cal to this function is made, all CAN IRQs are suspended
  *          and the mutexes associated with the buffers are locked, in order
  *          to synchronize the incoming data as well as avoid corruption.
  * @param receivedList A list containing a "received" flag for each incoming frame.
  * @param frameList A list containing the actual CAN frames.
  * @return Void.
  */
void CopyPrefusedFrameList(u8_t* receivedList, CanFrame_t* frameList);

/**
  * @brief Transmit a CAN frame to the CAN bus.
  * @details An abstract CAN frame (generic format) is converted to a
  *          native format in order to be transmitted.
  * @param canFrame The CAN frame to be transmitted.
  * @todo Make param const.
  * @return Void.
  */
void TransmitCanFrame(CanFrame_t* canFrame);

/**
  * @brief Resets all the Rx buffers of the CAN module.
  * @details Whenever a cycle of the system is completed, all the buffers holding
  *          input data should be cleared, in order to receive new data.
  * @return Void.
  */
void ResetRxBuffers(void);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* CAN_INTERFACE_H */
