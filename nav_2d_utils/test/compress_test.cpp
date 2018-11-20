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

using nav_2d_utils::compressPlan;

geometry_msgs::Pose2D make_pose(double x, double y)
{
  geometry_msgs::Pose2D pose;
  pose.x = x;
  pose.y = y;
  return pose;
}

void add_pose(nav_2d_msgs::Path2D& path, double x, double y)
{
  path.poses.push_back(make_pose(x, y));
}

TEST(CompressTest, compress_test)
{
  nav_2d_msgs::Path2D path;
  // Dataset borrowed from https://karthaus.nl/rdp/
  add_pose(path, 24, 173);
  add_pose(path, 26, 170);
  add_pose(path, 24, 166);
  add_pose(path, 27, 162);
  add_pose(path, 37, 161);
  add_pose(path, 45, 157);
  add_pose(path, 48, 152);
  add_pose(path, 46, 143);
  add_pose(path, 40, 140);
  add_pose(path, 34, 137);
  add_pose(path, 26, 134);
  add_pose(path, 24, 130);
  add_pose(path, 24, 125);
  add_pose(path, 28, 121);
  add_pose(path, 36, 118);
  add_pose(path, 46, 117);
  add_pose(path, 63, 121);
  add_pose(path, 76, 125);
  add_pose(path, 82, 120);
  add_pose(path, 86, 111);
  add_pose(path, 88, 103);
  add_pose(path, 90, 91);
  add_pose(path, 95, 87);
  add_pose(path, 107, 89);
  add_pose(path, 107, 104);
  add_pose(path, 106, 117);
  add_pose(path, 109, 129);
  add_pose(path, 119, 131);
  add_pose(path, 131, 131);
  add_pose(path, 139, 134);
  add_pose(path, 138, 143);
  add_pose(path, 131, 152);
  add_pose(path, 119, 154);
  add_pose(path, 111, 149);
  add_pose(path, 105, 143);
  add_pose(path, 91, 139);
  add_pose(path, 80, 142);
  add_pose(path, 81, 152);
  add_pose(path, 76, 163);
  add_pose(path, 67, 161);
  add_pose(path, 59, 149);
  add_pose(path, 63, 138);

  EXPECT_EQ(41, compressPlan(path, 0.1).poses.size());
  EXPECT_EQ(34, compressPlan(path, 1.3).poses.size());
  EXPECT_EQ(12, compressPlan(path, 9.5).poses.size());
  EXPECT_EQ(8, compressPlan(path, 19.9).poses.size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
