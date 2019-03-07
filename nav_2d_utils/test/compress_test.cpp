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
using nav_2d_utils::addPose;

TEST(CompressTest, compress_test)
{
  nav_2d_msgs::Path2D path;
  // Dataset borrowed from https://karthaus.nl/rdp/
  addPose(path, 24, 173);
  addPose(path, 26, 170);
  addPose(path, 24, 166);
  addPose(path, 27, 162);
  addPose(path, 37, 161);
  addPose(path, 45, 157);
  addPose(path, 48, 152);
  addPose(path, 46, 143);
  addPose(path, 40, 140);
  addPose(path, 34, 137);
  addPose(path, 26, 134);
  addPose(path, 24, 130);
  addPose(path, 24, 125);
  addPose(path, 28, 121);
  addPose(path, 36, 118);
  addPose(path, 46, 117);
  addPose(path, 63, 121);
  addPose(path, 76, 125);
  addPose(path, 82, 120);
  addPose(path, 86, 111);
  addPose(path, 88, 103);
  addPose(path, 90, 91);
  addPose(path, 95, 87);
  addPose(path, 107, 89);
  addPose(path, 107, 104);
  addPose(path, 106, 117);
  addPose(path, 109, 129);
  addPose(path, 119, 131);
  addPose(path, 131, 131);
  addPose(path, 139, 134);
  addPose(path, 138, 143);
  addPose(path, 131, 152);
  addPose(path, 119, 154);
  addPose(path, 111, 149);
  addPose(path, 105, 143);
  addPose(path, 91, 139);
  addPose(path, 80, 142);
  addPose(path, 81, 152);
  addPose(path, 76, 163);
  addPose(path, 67, 161);
  addPose(path, 59, 149);
  addPose(path, 63, 138);

  EXPECT_EQ(41U, compressPlan(path, 0.1).poses.size());
  EXPECT_EQ(34U, compressPlan(path, 1.3).poses.size());
  EXPECT_EQ(12U, compressPlan(path, 9.5).poses.size());
  EXPECT_EQ(8U, compressPlan(path, 19.9).poses.size());
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
