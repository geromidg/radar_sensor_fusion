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

#include "can_protocol.h"

#include "constants.h"
#include "platform_params.h"
#include "fusion.h"
#include "radar_utils.h"
#include "tracking.h"
#include "reconfigure.h"

#include "util.c"
#include "reconfigure.c"
#include "algo_util.c"


namespace
{

   class FusionUtilsTest : public testing::Test
   {
   protected:

      FusionUtilsTest()
      {
      }

      virtual ~FusionUtilsTest()
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

   TEST_F(FusionUtilsTest, priorityGEmaxCanValues)
   {
      f32_t maxRange, maxX, maxY;

      maxX = max(fabs(TX_OBJECT_DISTANCE_X_MAX), fabs(TX_OBJECT_DISTANCE_X_MIN));
      maxY = max(fabs(TX_OBJECT_DISTANCE_Y_MAX), fabs(TX_OBJECT_DISTANCE_Y_MIN));
      maxRange = GetRange(maxX, maxY);

      ASSERT_GE(MAX_PRIORITY, maxRange);
   }

   TEST_F(FusionUtilsTest, gatingLimitGTgatingInvalid)
   {
      ASSERT_GT(STATE_GATING_VALUE_MIN_LIMIT, INVALID_GATING_VALUE);
      ASSERT_GT(totalGatingValueMinLimit, INVALID_GATING_VALUE);
   }

   TEST_F(FusionUtilsTest, zeroVarianceGating)
   {
      f32_t gatingQuality;

      PrefusedObject_t prefusedObject;
      FusedObject_t fusedObject;

      prefusedObject.valid = TRUE;
      prefusedObject.plot.Z[STATE_X] = 4.f;
      prefusedObject.plot.Z[STATE_Y] = 3.f;
      prefusedObject.plot.Z[STATE_VX] = 10.f;
      prefusedObject.plot.Z[STATE_VY] = 0.f;
      prefusedObject.plot.R[KALMAN_STATES * STATE_X + STATE_X] = 0.f;
      prefusedObject.plot.R[KALMAN_STATES * STATE_Y + STATE_Y] = 0.f;
      prefusedObject.plot.R[KALMAN_STATES * STATE_VX + STATE_VX] = 0.f;
      prefusedObject.plot.R[KALMAN_STATES * STATE_VY + STATE_VY] = 0.f;

      fusedObject.id = 1u;
      fusedObject.track.X[STATE_X] = 4.f;
      fusedObject.track.X[STATE_Y] = 3.f;
      fusedObject.track.X[STATE_VX] = 10.f;
      fusedObject.track.X[STATE_VY] = 0.f;
      fusedObject.track.P[KALMAN_STATES * STATE_X + STATE_X] = 0.f;
      fusedObject.track.P[KALMAN_STATES * STATE_Y + STATE_Y] = 0.f;
      fusedObject.track.P[KALMAN_STATES * STATE_VX + STATE_VX] = 0.f;
      fusedObject.track.P[KALMAN_STATES * STATE_VY + STATE_VY] = 0.f;

      gatingQuality = GetGatingValue(&prefusedObject, &fusedObject);

      EXPECT_EQ(gatingQuality, INVALID_GATING_VALUE);
   }

}