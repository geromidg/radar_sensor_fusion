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

#ifndef FUSION_UTILS_H
#define FUSION_UTILS_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************** Inclusions *********************************/

#include "algorithm_types.h"

/***************************** Macro Definitions *****************************/

/** Convert degrees to radians. */
#define DEG2RAD(x) (0.017453292519943f * (x))

/** Convert radians to degrees. */
#define RAD2DEG(x) (57.295779513082323f * (x))

/***************************** Static Functions ******************************/

/**
  * @brief Initialize the algo_util module.
  * @details All static variables (e.g. gating weights) are initialized
  *          according to the parameters.
  * @return Void.
  */
void InitializeFusionUtils(void);

/**
  * @brief Get a priority value for an object.
  * @details The priority of an object is the difference between a constant max value (e.g. 150),
  *          and the range of the object. The smaller the range, the higher the priority is.
  * @param target_x The x position of the object.
  * @param target_y The y position of the object.
  * @return The priority value that was calculated.
  */
f32_t GetObjectPriority(const f32_t posX, const f32_t posY);
   
/**
  * @brief Creates a prefused object using the information of an input object.
  * @details A prefused object of the prefused object list is initialized.
  *          Its state (pos/vel) and the sensor that is coming from are needed for it.
  * @param prefusedObject A prefused object used as an input to the algo.
  * @param sensor A sensor of the system. Its type and geometry is contained.
  * @param x The position x of the input object.
  * @param y The position y of the input object.
  * @param vx The velocity x of the input object.
  * @param vy The velocity y of the input object.
  * @return Void.
  */
void CreatePrefusedObject(PrefusedObject_t* prefusedObject, const Sensor_t* pSensor, f32_t posX, f32_t posY, f32_t velX, f32_t velY);

/**
  * @brief Tries to associate a prefused object with the current fused object list.
  * @details The prefused object passes an acceptance gate to determine if it will be fused
  *          with a neighbor or not. If no associationg (pairing) is made, the a new object will be created.
  * @param prefusedObject A prefused object used as an input to the algo.
  * @param fusedObjectList A list containing the fused objects (output of algo).
  * @return Void.
  */
void AssociatePrefusedObject(const PrefusedObject_t* prefusedObject, FusedObject_t* fusedObjectList);

/**
  * @brief Checks if two fused objects are very close and should be pruned.
  * @details Each state (pos/vel) of each object is compared with the other object's state.
  *          If all the states are below a user-defined limit, the object with the lowest priority is pruned (deleted).
  *          This function is similar with the `GetGatingValue` where the similarity of two objects is found.
  *          The difference is that the gating function takes a probabilistic approach, while this one takes
  *          a more intuitive (to the user) approach. Also, the gating function compares a track and a plot,
  *          while this function compares two tracks. This way, a maintainance to all the objects is performed
  *          to find double objects that might have occurred after the fusion.
  * @param fusedObject1 The first fusedObject to be checked for pruning.
  * @param fusedObject2 The second fusedObject to be checked for pruning.
  * @todo Account also for the objects' lifetime.
  * @return Void.
  */
void CheckObjectsForPruning(FusedObject_t* fusedObject1, FusedObject_t* fusedObject2);

/**
  * @brief Performs maintenance on a single fused object.
  * @details The lifetime counter of the object is updated.
  *          In case the oject was lost (not seen by any sensor), its lost counter is increased.
  *          If the lost counter is bigger than the user-defined coasting cycles, the object will be killed.
  *          Otherwise, the object will live and be outputted and on the next cycle it will be predicted (coasted).
  * @param fusedObject A fused object that is outputted from the algo.
  * @return Void.
  */
void MaintainObject(FusedObject_t* fusedObject);

/**
  * @brief Checks if a fused object is ready to be outputted.
  * @details If a fused object (tentative from the point of view of the interface) is valid
  *          and has lived long enough (user-defined parameter) it is confirmed.
  * @param fusedObject A fused object that is outputted from the algo.
  * @todo Refactor to return expression.
  * @todo Use M of N rule to confirm.
  * @return Whether the object is confirmed or not.
  */
u8_t IsTentativeObjectConfirmed(const FusedObject_t* fusedObject);

/*****************************************************************************/

#ifdef __cplusplus
}
#endif

#endif  /* FUSION_UTILS_H */
