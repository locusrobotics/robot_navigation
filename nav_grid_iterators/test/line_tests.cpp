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
#include <nav_grid_iterators/line/bresenham.h>
#include <nav_grid_iterators/line/ray_trace.h>

template<class iterator_type>
int countIterations(iterator_type it, int max_iterations = 1000)
{
  int count = 0;
  iterator_type end = it.end();
  for ( ; it != end; ++it)
  {
    ++count;
    if (count >= max_iterations) break;
  }
  return count;
}

TEST(Lines, line)
{
  for (int i = 0; i <= 1; i++)
  {
    bool include_last_point = i == 0;
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, 0, 0, 0, include_last_point)), 1 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, 0, 3, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, 0, -3, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(3, 0, 0, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(-3, 0, 0, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, 0, 0, 3, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, 0, 0, -3, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, 3, 0, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, -3, 0, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, 0, 1, 9, include_last_point)), 10 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, 0, -1, -9, include_last_point)), 10 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(1, 9, 0, 0, include_last_point)), 10 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(-1, -9, 0, 0, include_last_point)), 10 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(9, 1, 0, 0, include_last_point)), 10 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(0, 0, 9, 1, include_last_point)), 10 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(-5, -5, 5, 5, include_last_point)), 11 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(-5, 5, 15, 5, include_last_point)), 21 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(15, 5, -5, 5, include_last_point)), 21 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::Bresenham(15, 5, 15, 7, include_last_point)), 3 - i);

    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, 0, 0, 0, include_last_point)), 1 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, 0, 3, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, 0, -3, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(3, 0, 0, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(-3, 0, 0, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, 0, 0, 3, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, 0, 0, -3, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, 3, 0, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, -3, 0, 0, include_last_point)), 4 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, 0, 1, 9, include_last_point)), 11 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, 0, -1, -9, include_last_point)), 11 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(1, 9, 0, 0, include_last_point)), 11 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(-1, -9, 0, 0, include_last_point)), 11 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(9, 1, 0, 0, include_last_point)), 11 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(0, 0, 9, 1, include_last_point)), 11 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(-5, -5, 5, 5, include_last_point)), 21 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(-5, 5, 15, 5, include_last_point)), 21 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(15, 5, -5, 5, include_last_point)), 21 - i);
    EXPECT_EQ(countIterations(nav_grid_iterators::RayTrace(15, 5, 15, 7, include_last_point)), 3 - i);
  }
}

TEST(Lines, border_conditions)
{
  int N = 100;
  int N2 = N / 2;
  for (int include_last_point = 0; include_last_point <= 1; ++include_last_point)
  {
    for (int i=0; i < N; ++i)
    {
      EXPECT_LT(countIterations(nav_grid_iterators::Bresenham(0, 0,  N2, i - N2, include_last_point)), N*3);
      EXPECT_LT(countIterations(nav_grid_iterators::Bresenham(0, 0, -N2, i - N2, include_last_point)), N*3);
      EXPECT_LT(countIterations(nav_grid_iterators::Bresenham(0, 0, i - N2,  N2, include_last_point)), N*3);
      EXPECT_LT(countIterations(nav_grid_iterators::Bresenham(0, 0, i - N2, -N2, include_last_point)), N*3);
      EXPECT_LT(countIterations(nav_grid_iterators::RayTrace(0, 0,  N2, i - N2, include_last_point)), N*3);
      EXPECT_LT(countIterations(nav_grid_iterators::RayTrace(0, 0, -N2, i - N2, include_last_point)), N*3);
      EXPECT_LT(countIterations(nav_grid_iterators::RayTrace(0, 0, i - N2,  N2, include_last_point)), N*3);
      EXPECT_LT(countIterations(nav_grid_iterators::RayTrace(0, 0, i - N2, -N2, include_last_point)), N*3);
    }
  }
}

TEST(Lines, equality)
{
  nav_grid_iterators::Bresenham it1(0, 0, 5, 5);
  nav_grid_iterators::Bresenham it2(0, 0, 1, 1);
  ASSERT_FALSE(it1 == it2);
}

TEST(Lines, test_copy)
{
  // This test will fail to compile if you cannot use the copy operator
  nav_grid_iterators::Bresenham bres(0, 0, 0, 0);
  bres = bres.begin();
  nav_grid_iterators::RayTrace rt(0, 0, 0, 0);
  rt = rt.begin();
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
