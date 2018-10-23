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
#include <nav_core2/bounds.h>

using nav_core2::Bounds;

TEST(Bounds, test_bounds_simple)
{
  Bounds b;
  EXPECT_TRUE(b.isEmpty());

  b.touch(5.0, 6.0);
  EXPECT_EQ(5.0, b.getMinX());
  EXPECT_EQ(5.0, b.getMinX());
  EXPECT_EQ(5.0, b.getMaxX());
  EXPECT_EQ(6.0, b.getMinY());
  EXPECT_EQ(6.0, b.getMaxY());
  EXPECT_FALSE(b.isEmpty());

  Bounds b2 = b;
  EXPECT_EQ(5.0, b2.getMinX());
  EXPECT_EQ(5.0, b2.getMaxX());
  EXPECT_EQ(6.0, b2.getMinY());
  EXPECT_EQ(6.0, b2.getMaxY());

  b.reset();
  EXPECT_EQ(5.0, b2.getMinX());
  EXPECT_EQ(5.0, b2.getMaxX());
  EXPECT_EQ(6.0, b2.getMinY());
  EXPECT_EQ(6.0, b2.getMaxY());
  EXPECT_TRUE(b.isEmpty());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
