/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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
#include <nav_core2/basic_costmap.h>
#include <costmap_queue/costmap_queue.h>
#include <costmap_queue/limited_costmap_queue.h>
#include <ros/ros.h>
#include <memory>
#include <algorithm>


nav_core2::BasicCostmap costmap;

TEST(CostmapQueue, basicQueue)
{
  costmap_queue::CostmapQueue q(costmap);
  int count = 0;
  q.enqueueCell(0, 0);
  while (!q.isEmpty())
  {
    costmap_queue::CellData cell = q.getNextCell();
    EXPECT_FLOAT_EQ(cell.distance_, hypot(cell.x_, cell.y_));
    count++;
  }
  EXPECT_EQ(count, 25);
}

TEST(CostmapQueue, reverseQueue)
{
  costmap_queue::CostmapQueue q(costmap);
  int count = 0;
  q.enqueueCell(4, 4);
  while (!q.isEmpty())
  {
    costmap_queue::CellData cell = q.getNextCell();
    EXPECT_FLOAT_EQ(cell.distance_, hypot(4.0 - static_cast<double>(cell.x_),
                                          4.0 - static_cast<double>(cell.y_)));
    count++;
  }
  EXPECT_EQ(count, 25);
}

TEST(CostmapQueue, bigTest)
{
  nav_grid::NavGridInfo big_info;
  big_info.width = 500;
  big_info.height = 500;

  nav_core2::BasicCostmap big_map;
  big_map.setInfo(big_info);
  costmap_queue::CostmapQueue q(big_map);
  int count = 0;
  q.enqueueCell(0, 0);
  while (!q.isEmpty())
  {
    costmap_queue::CellData cell = q.getNextCell();
    EXPECT_FLOAT_EQ(cell.distance_, hypot(cell.x_, cell.y_));
    count++;
  }
  EXPECT_EQ(count, 500 * 500);
}

TEST(CostmapQueue, linearQueue)
{
  costmap_queue::CostmapQueue q(costmap);
  int count = 0;
  q.enqueueCell(0, 0);
  q.enqueueCell(0, 1);
  q.enqueueCell(0, 2);
  q.enqueueCell(0, 3);
  q.enqueueCell(0, 4);
  while (!q.isEmpty())
  {
    costmap_queue::CellData cell = q.getNextCell();
    EXPECT_FLOAT_EQ(cell.distance_, cell.x_);
    count++;
  }
  EXPECT_EQ(count, 25);
}

TEST(CostmapQueue, crossQueue)
{
  costmap_queue::CostmapQueue q(costmap);
  int count = 0;
  int xs[] = {1, 2, 2, 3};
  int ys[] = {2, 1, 3, 2};
  int N = 4;
  for (int i = 0; i < N; i++)
  {
    q.enqueueCell(xs[i], ys[i]);
  }

  while (!q.isEmpty())
  {
    costmap_queue::CellData cell = q.getNextCell();
    double min_d = 1000;

    for (int i = 0; i < N; i++)
    {
      double dd = hypot(xs[i] - static_cast<float>(cell.x_), ys[i] - static_cast<float>(cell.y_));
      min_d = std::min(min_d, dd);
    }
    EXPECT_FLOAT_EQ(cell.distance_, min_d);
    count++;
  }
  EXPECT_EQ(count, 25);
}

TEST(CostmapQueue, limitedQueue)
{
  costmap_queue::LimitedCostmapQueue q(costmap, 5);
  int count = 0;
  q.enqueueCell(0, 0);
  while (!q.isEmpty())
  {
    costmap_queue::CellData cell = q.getNextCell();
    EXPECT_FLOAT_EQ(cell.distance_, hypot(cell.x_, cell.y_));
    count++;
  }
  EXPECT_EQ(count, 24);

  costmap_queue::LimitedCostmapQueue q2(costmap, 3);
  count = 0;
  q2.enqueueCell(0, 0);
  while (!q2.isEmpty())
  {
    q2.getNextCell();
    count++;
  }
  EXPECT_EQ(count, 11);
}


TEST(CostmapQueue, changingSize)
{
  nav_grid::NavGridInfo info0;
  info0.width = 2;
  info0.height = 3;

  nav_grid::NavGridInfo info1;
  info1.width = 6;
  info1.height = 7;

  nav_core2::BasicCostmap size_map;
  size_map.setInfo(info0);
  costmap_queue::CostmapQueue q(size_map);
  unsigned int count = 0;
  q.enqueueCell(0, 0);
  while (!q.isEmpty())
  {
    q.getNextCell();
    count++;
  }

  EXPECT_EQ(count, info0.width * info0.height);

  size_map.setInfo(info1);
  q.reset();
  count = 0;
  q.enqueueCell(0, 0);
  while (!q.isEmpty())
  {
    q.getNextCell();
    count++;
  }
  EXPECT_EQ(count, info1.width * info1.height);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  nav_grid::NavGridInfo info;
  info.width = 5;
  info.height = 5;
  costmap.setInfo(info);
  return RUN_ALL_TESTS();
}
