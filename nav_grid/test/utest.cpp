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
#include <nav_grid/vector_nav_grid.h>
#include <nav_grid/coordinate_conversion.h>
#include <algorithm>

TEST(VectorNavGrid, info_equality)
{
  nav_grid::NavGridInfo info0;
  nav_grid::NavGridInfo info1;
  nav_grid::NavGridInfo width_info;
  width_info.width = 3;

  nav_grid::NavGridInfo height_info;
  height_info.height = 3;

  nav_grid::NavGridInfo res_info;
  res_info.resolution = 3.0;

  nav_grid::NavGridInfo frame_info;
  frame_info.frame_id = "foobar";

  nav_grid::NavGridInfo originx_info;
  originx_info.origin_x = 3.0;

  nav_grid::NavGridInfo originy_info;
  originy_info.origin_y = 3.0;


  EXPECT_EQ(info0, info0);
  EXPECT_EQ(info0, info1);
  EXPECT_NE(info0, width_info);
  EXPECT_NE(info0, height_info);
  EXPECT_NE(info0, res_info);
  EXPECT_NE(info0, frame_info);
  EXPECT_NE(info0, originx_info);
  EXPECT_NE(info0, originy_info);
}

TEST(VectorNavGrid, basic_test)
{
  nav_grid::VectorNavGrid<int> grid(-3);
  nav_grid::NavGridInfo info;
  info.width = 2;
  info.height = 3;
  grid.setInfo(info);
  EXPECT_EQ(grid(0, 0), -3);
  grid.setValue(1, 1, 10);
  EXPECT_EQ(grid(0, 0), -3);
  EXPECT_EQ(grid(1, 1), 10);
}

TEST(VectorNavGrid, basic_index_test)
{
  nav_grid::VectorNavGrid<int> grid(-3);
  nav_grid::NavGridInfo info;
  info.width = 2;
  info.height = 3;
  grid.setInfo(info);

  nav_grid::Index index0(0, 0), index1(1, 1);
  EXPECT_EQ(grid(index0), -3);
  grid.setValue(index1, 10);
  EXPECT_EQ(grid(index0), -3);
  EXPECT_EQ(grid(index1), 10);
  EXPECT_EQ(grid(0, 0), -3);
  EXPECT_EQ(grid(1, 1), 10);
}

TEST(VectorNavGrid, easy_coordinates_test)
{
  nav_grid::NavGridInfo info;
  info.width = 2;
  info.height = 3;

  double wx, wy;
  gridToWorld(info, 0, 0, wx, wy);
  EXPECT_DOUBLE_EQ(wx, 0.5);
  EXPECT_DOUBLE_EQ(wy, 0.5);
  gridToWorld(info, 1, 2, wx, wy);
  EXPECT_DOUBLE_EQ(wx, 1.5);
  EXPECT_DOUBLE_EQ(wy, 2.5);

  unsigned int umx, umy;
  int mx, my;
  double dmx, dmy;
  ASSERT_TRUE(worldToGridBounded(info, wx, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_EQ(umy, 2);
  worldToGrid(info, wx, wy, mx, my);
  EXPECT_EQ(mx, 1);
  EXPECT_EQ(my, 2);
  worldToGrid(info, wx, wy, dmx, dmy);
  EXPECT_DOUBLE_EQ(dmx, wx);
  EXPECT_DOUBLE_EQ(dmy, wy);

  // Invalid Coordinate
  wx = 2.5;
  EXPECT_FALSE(worldToGridBounded(info, wx, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_EQ(umy, 2);
  worldToGrid(info, wx, wy, mx, my);
  EXPECT_EQ(mx, 2);
  EXPECT_EQ(my, 2);

  // Border Cases
  EXPECT_TRUE(worldToGridBounded(info, 0.0, wy, umx, umy));
  EXPECT_EQ(umx, 0);
  EXPECT_TRUE(worldToGridBounded(info, 0.25, wy, umx, umy));
  EXPECT_EQ(umx, 0);
  EXPECT_TRUE(worldToGridBounded(info, 0.75, wy, umx, umy));
  EXPECT_EQ(umx, 0);
  EXPECT_TRUE(worldToGridBounded(info, 0.9999, wy, umx, umy));
  EXPECT_EQ(umx, 0);
  EXPECT_TRUE(worldToGridBounded(info, 1.0, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_TRUE(worldToGridBounded(info, 1.25, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_TRUE(worldToGridBounded(info, 1.75, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_TRUE(worldToGridBounded(info, 1.9999, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_FALSE(worldToGridBounded(info, 2.0, wy, umx, umy));
  EXPECT_EQ(umx, 1);
}

TEST(VectorNavGrid, hard_coordinates_test)
{
  nav_grid::NavGridInfo info;
  info.width = 2;
  info.height = 3;
  info.resolution = 0.1;
  info.origin_x = -0.2;
  info.origin_y = 0.2;

  double wx, wy;
  gridToWorld(info, 0, 0, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -0.15);
  EXPECT_DOUBLE_EQ(wy, 0.25);
  gridToWorld(info, 1, 2, wx, wy);
  EXPECT_DOUBLE_EQ(wx, -0.05);
  EXPECT_DOUBLE_EQ(wy, 0.45);

  unsigned int umx, umy;
  int mx, my;
  double dmx, dmy;
  EXPECT_TRUE(worldToGridBounded(info, wx, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_EQ(umy, 2);
  worldToGrid(info, wx, wy, mx, my);
  EXPECT_EQ(mx, 1);
  EXPECT_EQ(my, 2);
  worldToGrid(info, wx, wy, dmx, dmy);
  EXPECT_DOUBLE_EQ(dmx, 1.5);
  EXPECT_DOUBLE_EQ(dmy, 2.5);

  // Invalid Coordinate
  wx = 2.5;
  EXPECT_FALSE(worldToGridBounded(info, wx, wy, umx, umy));
  EXPECT_EQ(umx, 1);
  EXPECT_EQ(umy, 2);
  worldToGrid(info, wx, wy, mx, my);
  EXPECT_EQ(mx, 27);
  EXPECT_EQ(my, 2);
}


TEST(VectorNavGrid, speed_test)
{
  nav_grid::NavGridInfo info;

  const int N = 1000;
  const int EXTRA = 300;

  info.width = N;
  info.height = N;

  double wx, wy;
  unsigned int umx, umy;
  int mx, my;
  double dmx, dmy;

  for (int x = -EXTRA; x < N + EXTRA; x++)
  {
    for (int y = -EXTRA; y < N + EXTRA; y++)
    {
      gridToWorld(info, x, y, wx, wy);
      if (x < 0 || y < 0 || x >= N || y >= N)
      {
        EXPECT_FALSE(isWithinGrid(info, wx, wy));
        EXPECT_FALSE(worldToGridBounded(info, wx, wy, umx, umy));
        EXPECT_EQ(umx, std::min(std::max(0, x), N - 1));
        EXPECT_EQ(umy, std::min(std::max(0, y), N - 1));
      }
      else
      {
        EXPECT_TRUE(isWithinGrid(info, wx, wy));
        EXPECT_TRUE(worldToGridBounded(info, wx, wy, umx, umy));
        EXPECT_EQ(umx, x);
        EXPECT_EQ(umy, y);
      }
      worldToGrid(info, wx, wy, mx, my);
      EXPECT_EQ(mx, x);
      EXPECT_EQ(my, y);
      worldToGrid(info, wx, wy, dmx, dmy);
      EXPECT_DOUBLE_EQ(dmx, x + 0.5);
      EXPECT_DOUBLE_EQ(dmy, y + 0.5);
    }
  }
}

int testGridValue(double x, double y)
{
  return static_cast<int>(100 * floor(x) + floor(y));
}

/**
 * Initialize Grid Values with values based on the grid/world coordinates
 * which are initially the same
 *
 *   x -->
 * 000 100 200 300 400 500 600 700 800 900  y
 * 001 101 201 301 401 501 601 701 801 901  |
 * 002 102 202 302 402 502 602 702 802 902  |
 * 003 103 203 303 403 503 603 703 803 903  V
 * 004 104 204 304 404 504 604 704 804 904

 */
void initializeTestGrid(nav_grid::VectorNavGrid<int>& grid)
{
  grid.setDefaultValue(-10);
  nav_grid::NavGridInfo info;
  info.width = 10;
  info.height = 5;
  grid.setInfo(info);
  double mx, my;
  for (unsigned int j = 0; j < info.height; j++)
  {
    for (unsigned int i = 0; i < info.width; i++)
    {
      gridToWorld(info, i, j, mx, my);
      grid.setValue(i, j, testGridValue(mx, my));
    }
  }
}

/**
 * Check to make sure all the grid values are now the same based on their grid coordinates
 */
void checkSetGridValues(const nav_grid::VectorNavGrid<int>& grid,
                        unsigned int x0, unsigned int x1, unsigned int y0, unsigned int y1)
{
  for (unsigned int x = 0; x < grid.getWidth(); x++)
  {
    for (unsigned int y = 0; y < grid.getHeight(); y++)
    {
      if (x >= x0 && x < x1 && y >= y0 && y < y1)
      {
        EXPECT_EQ(grid(x, y), testGridValue(x, y));  // testGridValue based on Grid Coordinates
      }
      else
      {
        EXPECT_EQ(grid(x, y), -10);
      }
    }
  }
}

/**
 * Check to make sure all the grid values are now the same based on their world coordinates
 */
void checkUpdateGridValues(const nav_grid::VectorNavGrid<int>& grid,
                           unsigned int x0, unsigned int x1, unsigned int y0, unsigned int y1)
{
  double mx, my;
  for (unsigned int x = 0; x < grid.getWidth(); x++)
  {
    for (unsigned int y = 0; y < grid.getHeight(); y++)
    {
      if (x >= x0 && x < x1 && y >= y0 && y < y1)
      {
        gridToWorld(grid.getInfo(), x, y, mx, my);
        EXPECT_EQ(grid(x, y), testGridValue(mx, my));  // testGridValue based on World Coordinates
      }
      else
      {
        EXPECT_EQ(grid(x, y), -10);
      }
    }
  }
}

void debugGridValues(const nav_grid::VectorNavGrid<int>& grid)
{
  for (unsigned int j = 0; j < grid.getHeight(); j++)
  {
    for (unsigned int i = 0; i < grid.getWidth(); i++)
    {
      printf("%d ", grid(i, j));
    }
    printf("\n");
  }
  printf("\n");
}

TEST(VectorNavGrid, resizing_grid_with_set)
{
  nav_grid::VectorNavGrid<int> grid;
  initializeTestGrid(grid);
  checkSetGridValues(grid, 0, grid.getWidth(), 0, grid.getHeight());

  nav_grid::NavGridInfo decreased_width_info = grid.getInfo();
  decreased_width_info.width = 5;
  grid.setInfo(decreased_width_info);
  checkSetGridValues(grid, 0, grid.getWidth(), 0, grid.getHeight());

  nav_grid::NavGridInfo increased_width_info = grid.getInfo();
  increased_width_info.width = 9;
  grid.setInfo(increased_width_info);
  checkSetGridValues(grid, 0, 5, 0, grid.getHeight());

  initializeTestGrid(grid);
  checkSetGridValues(grid, 0, grid.getWidth(), 0, grid.getHeight());

  nav_grid::NavGridInfo increased_height_info = grid.getInfo();
  increased_height_info.height = 9;
  grid.setInfo(increased_height_info);
  checkSetGridValues(grid, 0, grid.getWidth(), 0, 5);

  nav_grid::NavGridInfo decreased_height_info = grid.getInfo();
  decreased_height_info.height = 4;
  grid.setInfo(decreased_height_info);
  checkSetGridValues(grid, 0, grid.getWidth(), 0, 4);
}

TEST(VectorNavGrid, resizing_grid_with_update)
{
  nav_grid::VectorNavGrid<int> grid;
  initializeTestGrid(grid);
  checkUpdateGridValues(grid, 0, grid.getWidth(), 0, grid.getHeight());

  nav_grid::NavGridInfo decreased_width_info = grid.getInfo();
  decreased_width_info.width = 5;
  grid.updateInfo(decreased_width_info);
  checkUpdateGridValues(grid, 0, grid.getWidth(), 0, grid.getHeight());

  nav_grid::NavGridInfo increased_width_info = grid.getInfo();
  increased_width_info.width = 9;
  grid.updateInfo(increased_width_info);
  checkUpdateGridValues(grid, 0, 5, 0, grid.getHeight());

  initializeTestGrid(grid);
  checkUpdateGridValues(grid, 0, grid.getWidth(), 0, grid.getHeight());

  nav_grid::NavGridInfo increased_height_info = grid.getInfo();
  increased_height_info.height = 9;
  grid.updateInfo(increased_height_info);
  checkUpdateGridValues(grid, 0, grid.getWidth(), 0, 5);

  nav_grid::NavGridInfo decreased_height_info = grid.getInfo();
  decreased_height_info.height = 4;
  grid.updateInfo(decreased_height_info);
  checkUpdateGridValues(grid, 0, grid.getWidth(), 0, 4);
}

TEST(VectorNavGrid, change_origin)
{
  nav_grid::VectorNavGrid<int> grid;
  initializeTestGrid(grid);

  nav_grid::NavGridInfo bump_right_info = grid.getInfo();
  bump_right_info.origin_x = 3;
  grid.updateInfo(bump_right_info);
  checkUpdateGridValues(grid, 0, 7, 0, grid.getHeight());

  nav_grid::NavGridInfo bump_up_info = grid.getInfo();
  bump_up_info.origin_y = 2;
  grid.updateInfo(bump_up_info);
  checkUpdateGridValues(grid, 0, 7, 0, 3);

  nav_grid::NavGridInfo bump_left_info = grid.getInfo();
  bump_left_info.origin_x = -1;
  grid.updateInfo(bump_left_info);
  checkUpdateGridValues(grid, 4, grid.getWidth(), 0, 3);

  nav_grid::NavGridInfo bump_down_info = grid.getInfo();
  bump_down_info.origin_y = 0;
  grid.updateInfo(bump_down_info);
  checkUpdateGridValues(grid, 4, grid.getWidth(), 2, grid.getHeight());


  initializeTestGrid(grid);
  nav_grid::NavGridInfo bump_far_right_info = grid.getInfo();
  bump_far_right_info.origin_x = 30;
  grid.updateInfo(bump_far_right_info);
  checkUpdateGridValues(grid, 0, 0, 0, 0);
}

TEST(VectorNavGrid, combined_changes)
{
  // This is not a complete set of possible combined changes, just enough to satisfy my curiousity
  nav_grid::VectorNavGrid<int> grid;
  initializeTestGrid(grid);
  checkUpdateGridValues(grid, 0, grid.getWidth(), 0, grid.getHeight());

  nav_grid::NavGridInfo info1 = grid.getInfo();
  info1.width = 15;
  info1.origin_x = -5.0;
  grid.updateInfo(info1);
  checkUpdateGridValues(grid, 5, grid.getWidth(), 0, grid.getHeight());

  initializeTestGrid(grid);
  nav_grid::NavGridInfo info2 = grid.getInfo();
  info2.width = 17;
  info2.origin_x = -5.0;
  grid.updateInfo(info2);
  checkUpdateGridValues(grid, 5, grid.getWidth() - 2, 0, grid.getHeight());

  initializeTestGrid(grid);
  nav_grid::NavGridInfo info3 = grid.getInfo();
  info3.width = 2;
  info3.origin_x = 2.0;
  grid.updateInfo(info3);
  checkUpdateGridValues(grid, 0, grid.getWidth(), 0, grid.getHeight());

  initializeTestGrid(grid);
  nav_grid::NavGridInfo info4 = grid.getInfo();
  info4.width = 20;
  info4.height = 20;
  info4.origin_x = -2.0;
  info4.origin_y = -5.0;
  grid.updateInfo(info4);
  checkUpdateGridValues(grid, 2, 12, 5, 10);
}

TEST(Index, comparison_tests)
{
  unsigned int N = 5;
  for (unsigned int x0 = 0; x0 < N; ++x0)
  {
    for (unsigned int y0 = 0; y0 < N; ++y0)
    {
      nav_grid::Index index0(x0, y0);

      for (unsigned int x1 = 0; x1 < N; ++x1)
      {
        for (unsigned int y1 = 0; y1 < N; ++y1)
        {
          nav_grid::Index index1(x1, y1);
          // Check equality and the test for equality that sets use
          // See https://stackoverflow.com/a/1114862
          if (x0 == x1 && y0 == y1)
          {
            EXPECT_EQ(index0, index1);
            EXPECT_TRUE(!(index0 < index1) && !(index1 < index0));
            EXPECT_GE(index0, index1);
            EXPECT_LE(index0, index1);
            EXPECT_GE(index1, index0);
            EXPECT_LE(index1, index0);
          }
          else
          {
            EXPECT_NE(index0, index1);
            EXPECT_FALSE(!(index0 < index1) && !(index1 < index0));
            if (x0 < x1 || (x0 == x1 && y0 < y1))
            {
              EXPECT_LT(index0, index1);
              EXPECT_GT(index1, index0);
              EXPECT_LE(index0, index1);
              EXPECT_GE(index1, index0);
            }
            else
            {
              EXPECT_GT(index0, index1);
              EXPECT_LT(index1, index0);
              EXPECT_GE(index0, index1);
              EXPECT_LE(index1, index0);
            }
          }
        }
      }
    }
  }
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
