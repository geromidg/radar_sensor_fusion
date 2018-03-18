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

#include "stdafx.h"

#include "gtest/gtest.h"

#include "base_types.h"

#include "platform_params.h"
#include "config.h"
#include "radar_utils.h"
#include "fusion_utils.h"
#include "tracking.h"

#include "sensor_interface.c"
#include "kalman_utils.c"
#include "tracker.c"
#include "algo.c"

#define FRONT_LEFT (0u)
#define FRONT_RIGHT (1u)
#define REAR_RIGHT (2u)
#define REAR_LEFT (3u)

namespace
{

   class FusionTest : public testing::Test
   {
   protected:

      FusionTest()
      {
      }

      virtual ~FusionTest()
      {
      }

      virtual void SetUp()
      {
         InitializeSensorInterface(sensorList);

         for (int i = 0; i < NUM_SENSORS; i++)
         {
            sensorList[i].tf.canX = 0.f;  // Set to 0 so no offset is applied
         }

         (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);
         (void)memset(fusedObjectList, 0, sizeof(FusedObject_t) * (u32_t)NUM_FUSED_OBJ);

         InitializeFusion();
      }

      virtual void TearDown()
      {
      }

      Sensor_t sensorList[NUM_SENSORS];
      PrefusedObject_t prefusedObjectList[NUM_PREFUSED_OBJ];
      FusedObject_t fusedObjectList[NUM_FUSED_OBJ];
   };

   TEST_F(FusionTest, maxPrefusedObjects)
   {
      for (int i = 0; i < NUM_PREFUSED_OBJ; i++)
      {
         CreatePrefusedObject(&prefusedObjectList[i], &sensorList[REAR_RIGHT], i * (-10.f), -3.f, -10.f, 0.f);
      }

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      for (int i = 0; i < NUM_FUSED_OBJ; i++)
      {
         EXPECT_NE(fusedObjectList[i].id, 0u);
      }
   }
   
   // TODO: Move to unit tests
   TEST_F(FusionTest, createPrefusedObject)
   {
      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], 4.f, 3.f, 10.f, -0.1f);

      EXPECT_EQ(prefusedObjectList[0].valid, TRUE);
      EXPECT_EQ(prefusedObjectList[0].sensor, &sensorList[FRONT_LEFT]);
      EXPECT_FLOAT_EQ(prefusedObjectList[0].plot.Z[STATE_X], 4.f);
      EXPECT_FLOAT_EQ(prefusedObjectList[0].plot.Z[STATE_Y], 3.f);
      EXPECT_FLOAT_EQ(prefusedObjectList[0].plot.Z[STATE_VX], 10.f);
      EXPECT_FLOAT_EQ(prefusedObjectList[0].plot.Z[STATE_VY], -0.1f);
      EXPECT_FLOAT_EQ(prefusedObjectList[0].priority, 145.f);
   }

   // Case 1
   TEST_F(FusionTest, noOperation)
   {
      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      for (int i = 0; i < NUM_FUSED_OBJ; i++)
      {
         ASSERT_EQ(fusedObjectList[i].id, 0u);
      }
   }
   
   // Case 2
   TEST_F(FusionTest, createObject)
   {
      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], 4.f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 1u);
      EXPECT_EQ(fusedObjectList[0].lifetimeCounter, 1u);
      EXPECT_EQ(fusedObjectList[0].lostCounter, 0u);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_X], 4.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_Y], 3.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VX], 10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VY], 0.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].priority, 0.f);  // Should not be updated in this cycle
   }
   
   // Case 5
   TEST_F(FusionTest, predictObject)
   {
      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], 4.f, -3.f, -10.f, 1.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      ASSERT_EQ(fusedObjectList[0].id, 1u);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 1u);
      EXPECT_EQ(fusedObjectList[0].lifetimeCounter, 2u);
      EXPECT_EQ(fusedObjectList[0].lostCounter, 1u);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_X], 3.6f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_Y], -2.96f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VX], -10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VY], 1.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 1u);
      EXPECT_EQ(fusedObjectList[0].lifetimeCounter, 3u);
      EXPECT_EQ(fusedObjectList[0].lostCounter, 2u);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_X], 3.2f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_Y], -2.92f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VX], -10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VY], 1.f);
   }
   
   // Case 6.1
   TEST_F(FusionTest, associateAndFuseObject)
   {
      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], 4.f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      ASSERT_EQ(fusedObjectList[0].id, 1u);
      ASSERT_EQ(fusedObjectList[1].id, 0u);

      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], 4.4f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 1u);
      EXPECT_EQ(fusedObjectList[0].lifetimeCounter, 2u);
      EXPECT_EQ(fusedObjectList[0].lostCounter, 0u);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_X], 4.4f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_Y], 3.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VX], 10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VY], 0.f);

      // Assert that the object was associated and fused, and no new object was created.
      // This is under the assumption that the new object is created on the next available slot.
      ASSERT_EQ(fusedObjectList[1].id, 0u);
   }
   
   // Case 6.2
   TEST_F(FusionTest, dontAssociateObject)
   {
      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], 4.f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      ASSERT_EQ(fusedObjectList[0].id, 1u);
      ASSERT_EQ(fusedObjectList[1].id, 0u);

      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[REAR_LEFT], -4.f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 1u);
      EXPECT_EQ(fusedObjectList[0].lifetimeCounter, 2u);
      EXPECT_EQ(fusedObjectList[0].lostCounter, 1u);

      EXPECT_EQ(fusedObjectList[1].id, 2u);
      EXPECT_EQ(fusedObjectList[1].lifetimeCounter, 1u);
      EXPECT_EQ(fusedObjectList[1].lostCounter, 0u);
      EXPECT_FLOAT_EQ(fusedObjectList[1].track.X[STATE_X], -4.f);
      EXPECT_FLOAT_EQ(fusedObjectList[1].track.X[STATE_Y], 3.f);
      EXPECT_FLOAT_EQ(fusedObjectList[1].track.X[STATE_VX], 10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[1].track.X[STATE_VY], 0.f);
   }

   // Case 8.1
   TEST_F(FusionTest, dontAssociateAndDelete)
   {
      // Create 15 objects (arrange them in space), so only one slot is left
      // TODO: Create on different sensors (normally FM holds only 4 objects)
      for (int i = 0; (i < NUM_FUSED_OBJ - 1) && (i < NUM_PREFUSED_OBJ); i++)
      {
         CreatePrefusedObject(&prefusedObjectList[i], &sensorList[FRONT_LEFT], i * 10.f, 3.f, 10.f, 0.f);
      }

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      for (int i = 0; (i < NUM_FUSED_OBJ - 1); i++)
      {
         ASSERT_NE(fusedObjectList[i].id, 0u);
      }
      ASSERT_EQ(fusedObjectList[NUM_FUSED_OBJ - 1].id, 0u);
      
      // Update the 15 prefused objects
      for (int i = 0; (i < NUM_FUSED_OBJ - 1); i++)
      {
         CreatePrefusedObject(&prefusedObjectList[i], &sensorList[FRONT_LEFT], (i * 10.f) + 0.4f, 3.f, 10.f, 0.f);
      }

      // Create the 16th prefused object, so one fused object gets deleted
      CreatePrefusedObject(&prefusedObjectList[NUM_FUSED_OBJ - 1], &sensorList[FRONT_LEFT], 5.f, 20.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      for (int i = 0; (i < NUM_FUSED_OBJ - 1); i++)
      {
         ASSERT_NE(fusedObjectList[i].id, 0u);
      }

      // The 16th prefused object, should replace the object with the lowest priority, which is the last of the list.
      EXPECT_EQ(fusedObjectList[NUM_FUSED_OBJ - 1].id, 16u);
      EXPECT_EQ(fusedObjectList[NUM_FUSED_OBJ - 1].lifetimeCounter, 1u);
      EXPECT_EQ(fusedObjectList[NUM_FUSED_OBJ - 1].lostCounter, 0u);
      EXPECT_FLOAT_EQ(fusedObjectList[NUM_FUSED_OBJ - 1].track.X[STATE_X], 5.f);
      EXPECT_FLOAT_EQ(fusedObjectList[NUM_FUSED_OBJ - 1].track.X[STATE_Y], 20.f);
      EXPECT_FLOAT_EQ(fusedObjectList[NUM_FUSED_OBJ - 1].track.X[STATE_VX], 10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[NUM_FUSED_OBJ - 1].track.X[STATE_VY], 0.f);

      // TODO: Check that deleteObject() was called!
   }
   
   // Case 8.2
   TEST_F(FusionTest, associateAndCreate)
   {
      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], 4.f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      ASSERT_EQ(fusedObjectList[0].id, 1u);
      ASSERT_EQ(fusedObjectList[1].id, 0u);
      ASSERT_EQ(fusedObjectList[2].id, 0u);

      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], 4.4f, 3.f, 10.f, 0.f);
      CreatePrefusedObject(&prefusedObjectList[1], &sensorList[REAR_LEFT], -4.f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 1u);
      EXPECT_EQ(fusedObjectList[0].lifetimeCounter, 2u);
      EXPECT_EQ(fusedObjectList[0].lostCounter, 0u);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_X], 4.4f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_Y], 3.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VX], 10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VY], 0.f);

      EXPECT_EQ(fusedObjectList[1].id, 2u);
      EXPECT_EQ(fusedObjectList[1].lifetimeCounter, 1u);
      EXPECT_EQ(fusedObjectList[1].lostCounter, 0u);
      EXPECT_FLOAT_EQ(fusedObjectList[1].track.X[STATE_X], -4.f);
      EXPECT_FLOAT_EQ(fusedObjectList[1].track.X[STATE_Y], 3.f);
      EXPECT_FLOAT_EQ(fusedObjectList[1].track.X[STATE_VX], 10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[1].track.X[STATE_VY], 0.f);

      ASSERT_EQ(fusedObjectList[2].id, 0u);
   }
   
   // Case 8.3.1
   TEST_F(FusionTest, associateDoubleObjectsSameSensor)
   {
      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], -1.9f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      ASSERT_EQ(fusedObjectList[0].id, 1u);
      ASSERT_EQ(fusedObjectList[1].id, 0u);

      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], -1.5f, 3.f, 10.f, 0.f);
      CreatePrefusedObject(&prefusedObjectList[1], &sensorList[FRONT_LEFT], -1.5f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 1u);
      EXPECT_EQ(fusedObjectList[0].lifetimeCounter, 2u);
      EXPECT_EQ(fusedObjectList[0].lostCounter, 0u);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_X], -1.5f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_Y], 3.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VX], 10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VY], 0.f);

      ASSERT_EQ(fusedObjectList[1].id, 0u);
   }
   
   // Case 8.3.2
   TEST_F(FusionTest, associateDoubleObjectsDifferentSensors)
   {
      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], -1.9f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      ASSERT_EQ(fusedObjectList[0].id, 1u);
      ASSERT_EQ(fusedObjectList[1].id, 0u);

      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], -1.5f, 3.f, 10.f, 0.f);
      CreatePrefusedObject(&prefusedObjectList[1], &sensorList[REAR_LEFT], -1.5f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 1u);
      EXPECT_EQ(fusedObjectList[0].lifetimeCounter, 2u);
      EXPECT_EQ(fusedObjectList[0].lostCounter, 0u);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_X], -1.5f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_Y], 3.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VX], 10.f);
      EXPECT_FLOAT_EQ(fusedObjectList[0].track.X[STATE_VY], 0.f);

      ASSERT_EQ(fusedObjectList[1].id, 0u);
   }

   // TODO: Decrease the MAX_COASTING_CYCLES macro (monkey patch), to make the test faster
   TEST_F(FusionTest, coastObject)
   {
      CreatePrefusedObject(&prefusedObjectList[0], &sensorList[FRONT_LEFT], 4.f, 3.f, 10.f, 0.f);

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 1u);
      EXPECT_EQ(fusedObjectList[0].lifetimeCounter, 1u);
      EXPECT_EQ(fusedObjectList[0].lostCounter, 0u);

      for (int i = 0; i < MAX_COASTING_CYCLES; i++)
      {
         RunFusion(prefusedObjectList, fusedObjectList);
         (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

         EXPECT_EQ(fusedObjectList[0].id, 1u);
         EXPECT_EQ(fusedObjectList[0].lifetimeCounter, (u8_t)(i + 2));
         EXPECT_EQ(fusedObjectList[0].lostCounter, (u8_t)(i + 1));
      }

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      EXPECT_EQ(fusedObjectList[0].id, 0u);
   }

   // Stress testing
   // TODO: Add more pairs to each fused object
   // TODO: Move to non-functional tests
   TEST_F(FusionTest, fuseAllObjects)
   {
      for (int i = 0; i < NUM_PREFUSED_OBJ; i++)
      {
         CreatePrefusedObject(&prefusedObjectList[i], &sensorList[FRONT_RIGHT], i * 10.f, -3.f, 10.f, 0.f);
      }

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      for (int i = 0; i < NUM_FUSED_OBJ; i++)
      {
         EXPECT_NE(fusedObjectList[i].id, 0u);
      }

      for (int i = 0; i < NUM_PREFUSED_OBJ; i++)
      {
         CreatePrefusedObject(&prefusedObjectList[i], &sensorList[FRONT_RIGHT], (i * 10.f) + 0.4f, -3.f, 10.f, 0.f);
      }

      RunFusion(prefusedObjectList, fusedObjectList);
      (void)memset(prefusedObjectList, 0, sizeof(PrefusedObject_t) * (u32_t)NUM_PREFUSED_OBJ);

      for (int i = 0; i < NUM_FUSED_OBJ; i++)
      {
         EXPECT_NE(fusedObjectList[i].id, 0u);
      }
   }

}
