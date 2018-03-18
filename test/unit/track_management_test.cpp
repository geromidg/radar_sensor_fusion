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

#include "fusion.h"
#include "fusion_utils.h"

namespace
{

   class TrackManagementTest : public testing::Test
   {
   protected:

      TrackManagementTest()
      {
      }

      virtual ~TrackManagementTest()
      {
      }

      virtual void SetUp()
      {
         InitializeFusion();
      }

      virtual void TearDown()
      {
      }
   };

   TEST_F(TrackManagementTest, objectPruning1)
   {
      FusedObject_t fusedObject1;
      FusedObject_t fusedObject2;

      fusedObject1.id = 1u;
      fusedObject1.priority = 145.f;
      fusedObject1.track.X[STATE_X] = 40.f;
      fusedObject1.track.X[STATE_Y] = 3.f;
      fusedObject1.track.X[STATE_VX] = 10.f;
      fusedObject1.track.X[STATE_VY] = 0.f;

      fusedObject2.id = 2u;
      fusedObject2.priority = 140.f;
      fusedObject2.track.X[STATE_X] = 4.f;
      fusedObject2.track.X[STATE_Y] = 3.f;
      fusedObject2.track.X[STATE_VX] = 10.f;
      fusedObject2.track.X[STATE_VY] = 0.f;

      CheckObjectsForPruning(&fusedObject1, &fusedObject2);

      EXPECT_EQ(fusedObject1.id, 1u);
      EXPECT_EQ(fusedObject2.id, 2u);

      fusedObject1.track.X[STATE_X] = 4.f;

      CheckObjectsForPruning(&fusedObject1, &fusedObject2);

      EXPECT_EQ(fusedObject1.id, 1u);
      EXPECT_EQ(fusedObject2.id, 0u);
   }

   TEST_F(TrackManagementTest, objectPruning2)
   {
      FusedObject_t fusedObject1;
      FusedObject_t fusedObject2;

      fusedObject1.id = 1u;
      fusedObject1.priority = 145.f;
      fusedObject1.track.X[STATE_X] = 4.f;
      fusedObject1.track.X[STATE_Y] = 3.f;
      fusedObject1.track.X[STATE_VX] = 10.f;
      fusedObject1.track.X[STATE_VY] = 0.f;

      fusedObject2.id = 2u;
      fusedObject2.priority = 145.f;
      fusedObject2.track.X[STATE_X] = 4.f;
      fusedObject2.track.X[STATE_Y] = 3.f;
      fusedObject2.track.X[STATE_VX] = 10.f;
      fusedObject2.track.X[STATE_VY] = 0.f;

      CheckObjectsForPruning(&fusedObject1, &fusedObject2);

      EXPECT_EQ(fusedObject1.id, 0u);
      EXPECT_EQ(fusedObject2.id, 2u);
   }

}