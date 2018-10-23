/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <gtest/gtest.h>
#include <dlux_global_planner/kernel_function.h>

using dlux_global_planner::CardinalDirection;

TEST(KernelFunction, cardinal_addition)
{
  EXPECT_EQ(CardinalDirection::NORTHWEST, CardinalDirection::NORTH + CardinalDirection::WEST);
  EXPECT_EQ(CardinalDirection::SOUTHWEST, CardinalDirection::SOUTH + CardinalDirection::WEST);
  EXPECT_EQ(CardinalDirection::NORTHEAST, CardinalDirection::NORTH + CardinalDirection::EAST);
  EXPECT_EQ(CardinalDirection::SOUTHEAST, CardinalDirection::SOUTH + CardinalDirection::EAST);
}

TEST(KernelFunction, cardinal_and)
{
  EXPECT_TRUE(CardinalDirection::NORTHWEST & CardinalDirection::NORTH);
  EXPECT_TRUE(CardinalDirection::NORTHWEST & CardinalDirection::NORTHWEST);
  EXPECT_TRUE(CardinalDirection::NORTHWEST & CardinalDirection::WEST);
  EXPECT_FALSE(CardinalDirection::NORTHWEST & CardinalDirection::SOUTH);
  EXPECT_FALSE(CardinalDirection::NORTHWEST & CardinalDirection::SOUTHEAST);
  EXPECT_FALSE(CardinalDirection::NORTHWEST & CardinalDirection::EAST);
}

TEST(KernelFunction, some_kernel_values)
{
  nav_grid::NavGridInfo info;
  info.width = 6;
  info.height = 6;
  dlux_global_planner::PotentialGrid p(dlux_global_planner::HIGH_POTENTIAL);
  p.setInfo(info);
  p.reset();

  p.setValue(4, 4, 0.0f);
  CardinalDirection upstream;
  EXPECT_FLOAT_EQ(5.0f, dlux_global_planner::calculateKernel(p, 5, 4, 3, &upstream));
  EXPECT_EQ(CardinalDirection::NORTH, upstream);

  EXPECT_FLOAT_EQ(5.0f, dlux_global_planner::calculateKernel(p, 5, 3, 4, &upstream));
  EXPECT_EQ(CardinalDirection::EAST, upstream);

  p.setValue(4, 3, 5.0f);
  // One parent
  EXPECT_FLOAT_EQ(11.0f, dlux_global_planner::calculateKernel(p, 6, 3, 3, &upstream));
  EXPECT_EQ(CardinalDirection::EAST, upstream);

  p.setValue(3, 4, 5.0f);
  // Two parents
  EXPECT_FLOAT_EQ(9.224, dlux_global_planner::calculateKernel(p, 6, 3, 3, &upstream));
  EXPECT_EQ(CardinalDirection::NORTHEAST, upstream);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
