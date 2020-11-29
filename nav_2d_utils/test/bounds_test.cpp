/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
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
#include <nav_2d_utils/bounds.h>
#include <nav_grid/vector_nav_grid.h>
#include <vector>

using nav_2d_utils::divideBounds;
using nav_core2::UIntBounds;

/**
 * @brief Count the values in a grid.
 * @param[in] The grid
 * @param[out] match Number of values == 1
 * @param[out] missed Number of values == 0
 * @param[out] multiple Number of other values
 */
void countValues(const nav_grid::VectorNavGrid<unsigned char>& grid,
                 unsigned int& match, unsigned int& missed, unsigned int& multiple)
{
  match = 0;
  missed = 0;
  multiple = 0;

  nav_grid::NavGridInfo info = grid.getInfo();

  // No iterator to avoid tricky depenencies
  for (unsigned int x = 0; x < info.width; x++)
  {
    for (unsigned int y = 0; y < info.height; y++)
    {
      switch (grid(x, y))
      {
      case 0:
        missed++;
        break;
      case 1:
        match++;
        break;
      default:
        multiple++;
        break;
      }
    }
  }
}

TEST(DivideBounds, zeroes)
{
  UIntBounds bounds(2, 2, 5, 5);
  // Number of rows/cols has to be positive
  EXPECT_THROW(divideBounds(bounds, 0, 2), std::invalid_argument);
  EXPECT_THROW(divideBounds(bounds, 2, 0), std::invalid_argument);
  EXPECT_THROW(divideBounds(bounds, 0, 0), std::invalid_argument);
  EXPECT_NO_THROW(divideBounds(bounds, 2, 2));

  bounds.reset();
  // check for errors with empty bounds
  EXPECT_NO_THROW(divideBounds(bounds, 2, 2));
}

/**
 * This test is for the divideBounds method and takes grids of various sizes
 * (cycled through with the outer two loops) and tries to divide them into subgrids of
 * various sizes (cycled through with the next two loops). The resulting vector of
 * bounds should cover every cell in the original grid, so each of the divided bounds is
 * iterated over, adding one to each grid cell. If everything works perfectly, each cell
 * should be touched exactly once.
 */
TEST(DivideBounds, iterative_tests)
{
  nav_grid::VectorNavGrid<unsigned char> full_grid;
  nav_grid::NavGridInfo info;

  // count variables
  unsigned int match, missed, multiple;

  for (info.width = 1; info.width < 15; info.width++)
  {
    for (info.height = 1; info.height < 15; info.height++)
    {
      full_grid.setInfo(info);
      UIntBounds full_bounds = nav_2d_utils::getFullUIntBounds(info);
      for (unsigned int rows = 1; rows < 11u; rows++)
      {
        for (unsigned int cols = 1; cols < 11u; cols++)
        {
          full_grid.reset();
          std::vector<UIntBounds> divided = divideBounds(full_bounds, cols, rows);
          ASSERT_LE(divided.size(), rows * cols) << info.width << "x" << info.height << " " << rows << "x" << cols;
          for (const UIntBounds& sub : divided)
          {
            EXPECT_FALSE(sub.isEmpty());
            // Can't use nav_grid_iterator for circular dependencies
            for (unsigned int x = sub.getMinX(); x <= sub.getMaxX(); x++)
            {
              for (unsigned int y = sub.getMinY(); y <= sub.getMaxY(); y++)
              {
                full_grid.setValue(x, y, full_grid(x, y) + 1);
              }
            }
          }

          countValues(full_grid, match, missed, multiple);
          ASSERT_EQ(match, info.width * info.height) << "Full grid: " << info.width << "x" << info.height
                                                     << "  Requested divisions: " << rows << "x" << cols;
          EXPECT_EQ(missed, 0u);
          EXPECT_EQ(multiple, 0u);
        }
      }
    }
  }
}

/**
 * This test is for the divideBounds method and calls it recursively to
 * ensure that the method works when the minimum values in the original bounds
 * are not zero.
 */
TEST(DivideBounds, recursive_tests)
{
  nav_grid::VectorNavGrid<unsigned char> full_grid;
  nav_grid::NavGridInfo info;
  info.width = 100;
  info.height = 100;
  full_grid.setInfo(info);

  UIntBounds full_bounds = nav_2d_utils::getFullUIntBounds(info);

  std::vector<UIntBounds> level_one = divideBounds(full_bounds, 2, 2);
  ASSERT_EQ(level_one.size(), 4u);
  for (const UIntBounds& sub : level_one)
  {
    std::vector<UIntBounds> level_two = divideBounds(sub, 2, 2);
    ASSERT_EQ(level_two.size(), 4u);
    for (const UIntBounds& subsub : level_two)
    {
      EXPECT_GE(subsub.getMinX(), sub.getMinX());
      EXPECT_LE(subsub.getMaxX(), sub.getMaxX());
      EXPECT_GE(subsub.getMinY(), sub.getMinY());
      EXPECT_LE(subsub.getMaxY(), sub.getMaxY());
      // Can't use nav_grid_iterator for circular dependencies
      for (unsigned int x = subsub.getMinX(); x <= subsub.getMaxX(); x++)
      {
        for (unsigned int y = subsub.getMinY(); y <= subsub.getMaxY(); y++)
        {
          full_grid.setValue(x, y, full_grid(x, y) + 1);
        }
      }
    }
  }

  // Count values
  unsigned int match = 0,
               missed = 0,
               multiple = 0;
  countValues(full_grid, match, missed, multiple);
  ASSERT_EQ(match, info.width * info.height);
  EXPECT_EQ(missed, 0u);
  EXPECT_EQ(multiple, 0u);
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
