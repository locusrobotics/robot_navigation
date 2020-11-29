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
using nav_core2::UIntBounds;

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
  EXPECT_TRUE(b.contains(5.0, 6.0));
  EXPECT_FALSE(b.contains(5.5, 6.0));
  EXPECT_FALSE(b.contains(5.5, 4.0));
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
  EXPECT_FALSE(b.contains(5.0, 6.0));
  EXPECT_FALSE(b.contains(5.5, 6.0));
  EXPECT_TRUE(b2.contains(5.0, 6.0));
  EXPECT_FALSE(b.contains(5.5, 6.0));
  EXPECT_TRUE(b.isEmpty());

  Bounds b3;
  b3.touch(1.0, 5.0);
  b3.touch(4.0, 2.0);
  EXPECT_TRUE(b3.contains(3.0, 3.0));
  EXPECT_FALSE(b3.contains(0.0, 3.0));
  EXPECT_FALSE(b3.contains(5.0, 3.0));
  EXPECT_FALSE(b3.contains(3.0, 6.0));
  EXPECT_FALSE(b3.contains(3.0, 1.0));
}

TEST(Bounds, test_dimensions)
{
  UIntBounds empty;
  UIntBounds square(0, 0, 5, 5);
  UIntBounds rectangle(1, 4, 3, 15);
  EXPECT_EQ(empty.getWidth(), 0u);
  EXPECT_EQ(empty.getHeight(), 0u);

  EXPECT_EQ(square.getWidth(), 6u);
  EXPECT_EQ(square.getHeight(), 6u);

  EXPECT_EQ(rectangle.getWidth(), 3u);
  EXPECT_EQ(rectangle.getHeight(), 12u);
}

TEST(Bounds, test_bounds_overlap)
{
  UIntBounds b0(0, 0, 5, 5);
  UIntBounds b1(0, 0, 5, 5);
  UIntBounds b2(0, 0, 3, 3);
  UIntBounds b3(3, 0, 4, 4);
  UIntBounds b4(4, 0, 4, 4);
  UIntBounds b5(1, 4, 3, 15);
  UIntBounds b6(10, 10, 10, 10);
  EXPECT_TRUE(b0.overlaps(b0));
  EXPECT_TRUE(b0.overlaps(b1));
  EXPECT_TRUE(b0.overlaps(b2));
  EXPECT_TRUE(b2.overlaps(b0));
  EXPECT_TRUE(b0.overlaps(b3));
  EXPECT_TRUE(b2.overlaps(b3));
  EXPECT_FALSE(b2.overlaps(b4));
  EXPECT_TRUE(b0.overlaps(b5));
  EXPECT_TRUE(b5.overlaps(b0));
  EXPECT_FALSE(b0.overlaps(b6));
  EXPECT_FALSE(b6.overlaps(b0));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
