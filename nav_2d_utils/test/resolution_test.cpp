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
#include <nav_2d_utils/path_ops.h>

using nav_2d_utils::adjustPlanResolution;
using nav_2d_utils::addPose;

TEST(ResolutionTest, simple_example)
{
  nav_2d_msgs::Path2D path;
  // Space between points is one meter
  addPose(path, 0.0, 0.0);
  addPose(path, 0.0, 1.0);

  // resolution>=1, path won't change
  EXPECT_EQ(2U, adjustPlanResolution(path, 2.0).poses.size());
  EXPECT_EQ(2U, adjustPlanResolution(path, 1.0).poses.size());

  // 0.5 <= resolution < 1.0, one point should be added in the middle
  EXPECT_EQ(3U, adjustPlanResolution(path, 0.8).poses.size());
  EXPECT_EQ(3U, adjustPlanResolution(path, 0.5).poses.size());

  // 0.333 <= resolution < 0.5, two points need to be added
  EXPECT_EQ(4U, adjustPlanResolution(path, 0.34).poses.size());

  // 0.25 <= resolution < 0.333, three points need to be added
  EXPECT_EQ(5U, adjustPlanResolution(path, 0.32).poses.size());
}

TEST(ResolutionTest, real_example)
{
  // This test is based on a real-world example
  nav_2d_msgs::Path2D path;
  addPose(path, 17.779193, -0.972024);
  addPose(path, 17.799171, -0.950775);
  addPose(path, 17.851942, -0.903709);
  EXPECT_EQ(3U, adjustPlanResolution(path, 0.2).poses.size());
  EXPECT_EQ(4U, adjustPlanResolution(path, 0.05).poses.size());
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
