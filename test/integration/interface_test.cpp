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

#include "platform_params.h"
#include "config.h"
#include "reconfigure.h"

#include "interface.c"


namespace
{

   class InterfaceTest : public testing::Test
   {
   protected:

      InterfaceTest()
      {
      }

      virtual ~InterfaceTest()
      {
      }

      virtual void SetUp()
      {
         InitializeAlgorithm();

         resetInputObjectList();

         for (int i = 0; i < NUM_SENSORS; i++)
         {
            sensorList[i].tf.canX = 0.f;  // Set to 0 so no offset is applied
         }
      }

      virtual void TearDown()
      {
      }

      void resetInputObjectList(void)
      {
         (void)memset(inputObjectList, 0, sizeof(BaseObject_t) * (u32_t)NUM_PREFUSED_OBJ);
      }

      BaseObject_t inputObjectList[NUM_PREFUSED_OBJ];
      BaseObject_t outputObjectList[NUM_FUSED_OBJ];
   };
   
   // All tests are based on this assumption
   TEST_F(InterfaceTest, prefusedListLenghtSum)
   {
       u8_t sum = 0;

       for (int i = 0; i < NUM_SENSORS; i++)
       {
           sum += sensorList[i].objects.length;
       }

      ASSERT_EQ(NUM_PREFUSED_OBJ, sum);
   }
   
   // All tests are based on this assumption
   TEST_F(InterfaceTest, prefusedGEfused)
   {
      ASSERT_GE(NUM_PREFUSED_OBJ, NUM_FUSED_OBJ);
   }

   TEST_F(InterfaceTest, fullCycleOneObject)
   {
      inputObjectList[0].valid = 1u;
      inputObjectList[0].posX = 4.f;
      inputObjectList[0].posY = 3.f;
      inputObjectList[0].velX = 10.f;
      inputObjectList[0].velY = 0.f;

      RunAlgorithm(inputObjectList, outputObjectList);
      resetInputObjectList();

      for (int i = 0; i < (MIN_LIFETIME_TX_CYCLES - 1); i++)
      {
         ASSERT_EQ(outputObjectList[0].valid, 0u);

         RunAlgorithm(inputObjectList, outputObjectList);
      }

      EXPECT_TRUE(outputObjectList[0].valid);
      EXPECT_FLOAT_EQ(outputObjectList[0].posX, 4.8f);
      EXPECT_FLOAT_EQ(outputObjectList[0].posY, 3.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velX, 10.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velY, 0.f);

      ASSERT_EQ(outputObjectList[1].valid, 0u);
   }

   TEST_F(InterfaceTest, fuseLeftSide)
   {
      inputObjectList[0].valid = 1u;
      inputObjectList[0].posX = -2.f;
      inputObjectList[0].posY = 3.f;
      inputObjectList[0].velX = 10.f;
      inputObjectList[0].velY = 0.f;

      inputObjectList[16].valid = 2u;
      inputObjectList[16].posX = -2.f;
      inputObjectList[16].posY = 3.f;
      inputObjectList[16].velX = 10.f;
      inputObjectList[16].velY = 0.f;

      RunAlgorithm(inputObjectList, outputObjectList);
      resetInputObjectList();
      RunAlgorithm(inputObjectList, outputObjectList);
      RunAlgorithm(inputObjectList, outputObjectList);

      EXPECT_TRUE(outputObjectList[0].valid);
      EXPECT_FLOAT_EQ(outputObjectList[0].posX, -1.2f);
      EXPECT_FLOAT_EQ(outputObjectList[0].posY, 3.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velX, 10.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velY, 0.f);

      ASSERT_EQ(outputObjectList[1].valid, 0u);
   }

   TEST_F(InterfaceTest, fuseRightSide)
   {
      inputObjectList[4].valid = 1u;
      inputObjectList[4].posX = -2.f;
      inputObjectList[4].posY = -3.f;
      inputObjectList[4].velX = 10.f;
      inputObjectList[4].velY = 0.f;

      inputObjectList[8].valid = 2u;
      inputObjectList[8].posX = -2.f;
      inputObjectList[8].posY = -3.f;
      inputObjectList[8].velX = 10.f;
      inputObjectList[8].velY = 0.f;

      RunAlgorithm(inputObjectList, outputObjectList);
      resetInputObjectList();
      RunAlgorithm(inputObjectList, outputObjectList);
      RunAlgorithm(inputObjectList, outputObjectList);

      EXPECT_TRUE(outputObjectList[0].valid);
      EXPECT_FLOAT_EQ(outputObjectList[0].posX, -1.2f);
      EXPECT_FLOAT_EQ(outputObjectList[0].posY, -3.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velX, 10.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velY, 0.f);

      ASSERT_EQ(outputObjectList[1].valid, 0u);
   }

   TEST_F(InterfaceTest, fuseFrontSide)
   {
      inputObjectList[0].valid = 1u;
      inputObjectList[0].posX = 15.f;
      inputObjectList[0].posY = 0.f;
      inputObjectList[0].velX = 0.f;
      inputObjectList[0].velY = 10.f;

      inputObjectList[4].valid = 2u;
      inputObjectList[4].posX = 15.f;
      inputObjectList[4].posY = 0.f;
      inputObjectList[4].velX = 0.f;
      inputObjectList[4].velY = 10.f;

      RunAlgorithm(inputObjectList, outputObjectList);
      resetInputObjectList();
      RunAlgorithm(inputObjectList, outputObjectList);
      RunAlgorithm(inputObjectList, outputObjectList);

      EXPECT_TRUE(outputObjectList[0].valid);
      EXPECT_FLOAT_EQ(outputObjectList[0].posX, 15.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].posY, 0.8f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velX, 0.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velY, 10.f);

      ASSERT_EQ(outputObjectList[1].valid, 0u);
   }

   TEST_F(InterfaceTest, fuseRearSide)
   {
      inputObjectList[8].valid = 1u;
      inputObjectList[8].posX = -15.f;
      inputObjectList[8].posY = 0.5f;
      inputObjectList[8].velX = 0.f;
      inputObjectList[8].velY = -10.f;

      inputObjectList[16].valid = 2u;
      inputObjectList[16].posX = -15.f;
      inputObjectList[16].posY = 0.5f;
      inputObjectList[16].velX = 0.f;
      inputObjectList[16].velY = -10.f;

      RunAlgorithm(inputObjectList, outputObjectList);
      resetInputObjectList();
      RunAlgorithm(inputObjectList, outputObjectList);
      RunAlgorithm(inputObjectList, outputObjectList);

      EXPECT_TRUE(outputObjectList[0].valid);
      EXPECT_FLOAT_EQ(outputObjectList[0].posX, -15.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].posY, -0.3f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velX, 0.f);
      EXPECT_FLOAT_EQ(outputObjectList[0].velY, -10.f);

      ASSERT_EQ(outputObjectList[1].valid, 0u);
   }
   
}