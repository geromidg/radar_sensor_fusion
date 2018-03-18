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

#ifndef RECONFIGURE_H
#define RECONFIGURE_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "common_types.h"

/***************************** Public Functions ******************************/

/**
  * @brief The callback function when a new configuration is received.
  * @details Update only one parameter at a time. The parameter to be configured
  *          is determined by the select signal.
  * @param cfgSelect Determines which parameter will be configured.
  * @param cfgValue Determines what value the parameter will take.
  * @return Void.
  */
void CfgCallback(u8_t cfgSelect, f32_t cfgValue);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* RECONFIGURE_H */
