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
#include <nav_grid_iterators/iterators.h>
#include <algorithm>
#include <vector>

using nav_grid::Index;

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

TEST(WholeGrid, whole_grid)
{
  nav_grid::NavGridInfo info;
  info.width = 5;
  info.height = 8;
  int count = 0;
  for (nav_grid_iterators::WholeGrid it(info); it != it.end(); ++it)
  {
    Index i = *it;
    ASSERT_EQ(i.x, count % info.width);
    ASSERT_EQ(i.y, count / info.width);
    ++count;
  }
  ASSERT_EQ(count, 40);
}

TEST(WholeGrid, whole_grid_range)
{
  nav_grid::NavGridInfo info;
  info.width = 3;
  info.height = 6;
  int count = 0;
  for (Index i : nav_grid_iterators::WholeGrid(info))
  {
    ASSERT_EQ(i.x, count % info.width);
    ASSERT_EQ(i.y, count / info.width);
    ++count;
  }
  ASSERT_EQ(count, 18);
}

TEST(WholeGrid, std_stuff)
{
  nav_grid::NavGridInfo info;
  info.width = 8;
  info.height = 2;
  nav_grid_iterators::WholeGrid wg(info);

  std::vector<Index> vec;
  std::copy(wg.begin(), wg.end(), std::back_inserter(vec));
  for (int count = 0; count < 16; ++count)
  {
    Index& i = vec[count];
    ASSERT_EQ(i.x, count % info.width);
    ASSERT_EQ(i.y, count / info.width);
  }
}

TEST(SubGrid, sub_grid)
{
  nav_grid::NavGridInfo info;
  info.width = 5;
  info.height = 8;
  int count = 0;
  for (Index i : nav_grid_iterators::SubGrid(&info, 1, 2, 2, 3))
  {
    ASSERT_EQ(i.x, static_cast<unsigned int>(1 + count % 2));
    ASSERT_EQ(i.y, static_cast<unsigned int>(2 + count / 2));
    ++count;
  }
  ASSERT_EQ(count, 6);

  ASSERT_EQ(countIterations(nav_grid_iterators::SubGrid(&info, 1, 3, 4, 1)), 4);
  ASSERT_EQ(countIterations(nav_grid_iterators::SubGrid(&info, 1, 3, 2, 2)), 4);
  nav_core2::UIntBounds bounds(1, 3, 4, 3);
  ASSERT_EQ(countIterations(nav_grid_iterators::SubGrid(&info, bounds)), 4);

  // Empty Bounds
  bounds.reset();
  ASSERT_EQ(countIterations(nav_grid_iterators::SubGrid(&info, bounds)), 0);

  // Partially Overlapping Bounds
  bounds.touch(3, 2);
  bounds.touch(6, 3);
  ASSERT_EQ(countIterations(nav_grid_iterators::SubGrid(&info, bounds)), 4);

  bounds.reset();
  bounds.touch(1, 6);
  bounds.touch(3, 9);
  ASSERT_EQ(countIterations(nav_grid_iterators::SubGrid(&info, bounds)), 6);

  // Different empty bounds
  nav_core2::UIntBounds empty(1, 0, 0, 0);
  ASSERT_EQ(countIterations(nav_grid_iterators::SubGrid(&info, empty)), 0);
}

TEST(SubGrid, equality)
{
  nav_grid::NavGridInfo info;
  info.width = 5;
  info.height = 8;
  nav_grid_iterators::SubGrid it1(&info, 1, 2, 2, 3);
  nav_grid_iterators::SubGrid it2(&info, 1, 2, 1, 1);
  ASSERT_FALSE(it1 == it2);
}

TEST(CircleFill, circle)
{
  nav_grid::NavGridInfo info;
  info.width = 8;
  info.height = 8;
  info.resolution = 1.0;

  ASSERT_EQ(countIterations(nav_grid_iterators::CircleFill(&info, 4.0, 4.0, 3.0)), 32);
  ASSERT_EQ(countIterations(nav_grid_iterators::CircleFill(&info, 4.3, 4.0, 3.0)), 28);
  ASSERT_EQ(countIterations(nav_grid_iterators::CircleFill(&info, 4.0, 4.0, 8.0)), 64);
  ASSERT_EQ(countIterations(nav_grid_iterators::CircleFill(&info, 14.0, 4.0, 1.0)), 0);
  ASSERT_EQ(countIterations(nav_grid_iterators::CircleFill(&info, 0.0, 4.0, 4.0)), 26);
  ASSERT_EQ(countIterations(nav_grid_iterators::CircleFill(&info, 4.0, 4.0, 3.0).begin()), 32);
}

TEST(CircleFill, equality)
{
  nav_grid::NavGridInfo info;
  info.width = 5;
  info.height = 8;
  nav_grid_iterators::CircleFill it1(&info, 4.0, 4.0, 8.0);
  nav_grid_iterators::CircleFill it2(&info, 1.0, 1.0, 100.0);
  ASSERT_FALSE(it1 == it2);
}

TEST(CircleOutline, circle_outline)
{
  nav_grid::NavGridInfo info;
  info.width = 8;
  info.height = 8;
  info.resolution = 1.0;

  unsigned int size = 0;
  EXPECT_EQ(countIterations(nav_grid_iterators::CircleOutline(&info, 4.0, 4.0, size++)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::CircleOutline(&info, 4.0, 4.0, size++)), 8);
  EXPECT_EQ(countIterations(nav_grid_iterators::CircleOutline(&info, 4.0, 4.0, size++)), 16);
  EXPECT_EQ(countIterations(nav_grid_iterators::CircleOutline(&info, 4.0, 4.0, size++)), 20);
  EXPECT_EQ(countIterations(nav_grid_iterators::CircleOutline(&info, 4.0, 4.0, size++)), 14);
  EXPECT_EQ(countIterations(nav_grid_iterators::CircleOutline(&info, 4.0, 4.0, size++)), 5);
  EXPECT_EQ(countIterations(nav_grid_iterators::CircleOutline(&info, 4.0, 4.0, size++)), 0);
  EXPECT_EQ(countIterations(nav_grid_iterators::CircleOutline(&info, 4.0, 4.0, size++)), 0);
}

TEST(CircleOutline, equality)
{
  nav_grid::NavGridInfo info;
  info.width = 5;
  info.height = 8;
  nav_grid_iterators::CircleOutline it1(&info, 3.0, 1.0, 1.0);
  nav_grid_iterators::CircleOutline it2(&info, 1.0, 1.0, 3.0);
  ASSERT_FALSE(it1 == it2);
}

TEST(Spiral, spiral)
{
  nav_grid::NavGridInfo info;
  info.width = 8;
  info.height = 8;
  info.resolution = 1.0;
  info.origin_x = 0.0;
  info.origin_y = 0.0;
  ASSERT_EQ(countIterations(nav_grid_iterators::Spiral(&info, 4.0, 4.0, 3.0)), 32);
  ASSERT_EQ(countIterations(nav_grid_iterators::Spiral(&info, 4.3, 4.0, 3.0)), 28);
  ASSERT_EQ(countIterations(nav_grid_iterators::Spiral(&info, 4.0, 4.0, 8.0)), 64);
  ASSERT_EQ(countIterations(nav_grid_iterators::Spiral(&info, 14.0, 4.0, 1.0)), 0);
  ASSERT_EQ(countIterations(nav_grid_iterators::Spiral(&info, 0.0, 4.0, 4.0)), 26);
}

TEST(Spiral, equality)
{
  nav_grid::NavGridInfo info;
  info.width = 5;
  info.height = 8;
  nav_grid_iterators::Spiral it1(&info, 1.0, 1.0, 1.0);
  nav_grid_iterators::Spiral it2(&info, 1.0, 1.0, 3.0);
  ASSERT_FALSE(it1 == it2);
}

TEST(Line, signed_line)
{
  nav_grid::NavGridInfo info;
  info.width = 10;
  info.height = 10;
  info.resolution = 1.0;
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 0, 0)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 3, 0)), 4);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, -3, 0)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 3, 0, 0, 0)), 4);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, -3, 0, 0, 0)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, -3, 0, 0, 0, false)), 0);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 0, 3)), 4);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 0, -3)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 3, 0, 0)), 4);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, -3, 0, 0)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 1, 9)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 1, 9, 0, 0)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 9, 1, 0, 0)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 9, 1)), 10);

  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, -5, 5, 15, 5)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 15, 5, -5, 5)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 15, 5, 15, 7)), 0);

  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, -1, -5, -1, -7)), 0);
}

TEST(Line, signed_line_diff_res)
{
  // This is the same test as above with a reduced resolution. Most of the coordinates are divided by 10
  // (with an additional 0.05 in some places to avoid floating point errors)
  nav_grid::NavGridInfo info;
  info.width = 10;
  info.height = 10;
  info.resolution = 0.1;
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 0, 0)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 0.35, 0)), 4);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, -0.35, 0)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0.35, 0, 0, 0)), 4);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, -0.35, 0, 0, 0)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, -0.35, 0, 0, 0, false)), 0);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 0, 0.35)), 4);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 0, -0.35)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0.35, 0, 0)), 4);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, -0.35, 0, 0)), 1);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 0.15, 0.95)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0.15, 0.95, 0, 0)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0.95, 0.15, 0, 0)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 0, 0, 0.95, 0.15)), 10);

  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, -0.5, 0.5, 1.5, 0.5)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 1.5, 0.5, -0.55, 0.55)), 10);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 1.5, 0.5, 1.5, 0.75)), 0);

  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, -0.1, -0.5, -0.1, -0.7)), 0);
}

TEST(Line, random_test_case)
{
  nav_grid::NavGridInfo info;
  info.width = 795;
  info.height = 925;
  info.resolution = 0.05;
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 6.2402, 30.651832, 2.805347, 22.8941, false, false)), 224);
  EXPECT_EQ(countIterations(nav_grid_iterators::Line(&info, 6.2402, 30.651832, 2.805347, 22.8941, true, false)), 225);
}

TEST(Line, equality)
{
  nav_grid::NavGridInfo info;
  info.width = 5;
  info.height = 8;
  nav_grid_iterators::Line it1(&info, 0, 0, 5, 5);
  nav_grid_iterators::Line it2(&info, 0, 0, 1, 1);
  ASSERT_FALSE(it1 == it2);
}

nav_2d_msgs::Point2D make_point(double x, double y)
{
  nav_2d_msgs::Point2D pt;
  pt.x = x;
  pt.y = y;
  return pt;
}

TEST(Polygon, polygon)
{
  nav_grid::NavGridInfo info;
  // First check to make sure it works when the polygon is completely on the grid
  info.width = 10;
  info.height = 10;
  info.resolution = 1.0;
  nav_2d_msgs::Polygon2D simple_square;
  simple_square.points.push_back(make_point(1.4, 1.4));
  simple_square.points.push_back(make_point(1.4, 3.6));
  simple_square.points.push_back(make_point(3.6, 3.6));
  simple_square.points.push_back(make_point(3.6, 1.4));

  EXPECT_EQ(countIterations(nav_grid_iterators::PolygonOutline(&info, simple_square)), 8);
  EXPECT_EQ(countIterations(nav_grid_iterators::PolygonFill(&info, simple_square)), 9);

  // Then do it again when it is only partially on the grid
  info.height = 3;
  EXPECT_EQ(countIterations(nav_grid_iterators::PolygonOutline(&info, simple_square)), 5);
  EXPECT_EQ(countIterations(nav_grid_iterators::PolygonFill(&info, simple_square)), 6);

  // Then check when it is completely off the grid
  info.resolution = 0.1;
  EXPECT_EQ(countIterations(nav_grid_iterators::PolygonOutline(&info, simple_square)), 0);
  EXPECT_EQ(countIterations(nav_grid_iterators::PolygonFill(&info, simple_square)), 0);
}

TEST(Polygon, empty_polygon)
{
  nav_grid::NavGridInfo info;
  info.width = 10;
  info.height = 10;
  info.resolution = 1.0;
  nav_2d_msgs::Polygon2D empty_polygon;

  EXPECT_EQ(countIterations(nav_grid_iterators::PolygonOutline(&info, empty_polygon)), 0);
  EXPECT_EQ(countIterations(nav_grid_iterators::PolygonFill(&info, empty_polygon)), 0);
}

TEST(Polygon, equality)
{
  nav_grid::NavGridInfo info;
  info.width = 10;
  info.height = 10;
  info.resolution = 1.0;
  nav_2d_msgs::Polygon2D simple_square;
  simple_square.points.push_back(make_point(1.4, 1.4));
  simple_square.points.push_back(make_point(1.4, 3.6));
  simple_square.points.push_back(make_point(3.6, 3.6));
  simple_square.points.push_back(make_point(3.6, 1.4));

  nav_2d_msgs::Polygon2D triangle;
  triangle.points.push_back(make_point(1.4, 1.4));
  triangle.points.push_back(make_point(1.4, 3.6));
  triangle.points.push_back(make_point(3.6, 3.6));


  nav_grid_iterators::PolygonOutline it1(&info, simple_square);
  nav_grid_iterators::PolygonOutline it2(&info, triangle);
  ASSERT_FALSE(it1 == it2);

  nav_grid_iterators::PolygonFill it3(&info, simple_square);
  nav_grid_iterators::PolygonFill it4(&info, triangle);
  ASSERT_FALSE(it3 == it4);
}

TEST(Iterators, test_copy)
{
  // This test will fail to compile if you cannot use the copy operator
  nav_grid::NavGridInfo info;
  info.width = 10;
  info.height = 10;
  info.resolution = 1.0;

  nav_2d_msgs::Polygon2D simple_square;
  simple_square.points.push_back(make_point(1.4, 1.4));
  simple_square.points.push_back(make_point(1.4, 3.6));
  simple_square.points.push_back(make_point(3.6, 3.6));
  simple_square.points.push_back(make_point(3.6, 1.4));

  nav_grid_iterators::WholeGrid whole_grid(info);
  whole_grid = whole_grid.begin();
  nav_grid_iterators::SubGrid sub_grid(&info, 1, 2, 2, 3);
  sub_grid = sub_grid.begin();
  nav_grid_iterators::CircleFill cf(&info, 4.0, 4.0, 3.0);
  cf = cf.begin();
  nav_grid_iterators::CircleOutline co(&info, 4.0, 4.0, 3.0);
  co = co.begin();
  nav_grid_iterators::Spiral spiral(&info, 4.0, 4.0, 3.0);
  spiral = spiral.begin();
  nav_grid_iterators::Line line(&info, 0, 0, 0, 0);
  line = line.begin();
  nav_grid_iterators::PolygonOutline po(&info, simple_square);
  po = po.begin();
  nav_grid_iterators::PolygonFill pf(&info, simple_square);
  pf = pf.begin();
}

TEST(Iterators, test_assignment)
{
  nav_grid::NavGridInfo info;
  info.width = 10;
  info.height = 10;
  info.resolution = 1.0;
  nav_grid_iterators::CircleFill iter1(&info, 3.0, 3.0, 1.0);
  // Sequence should be (2, 2) (3, 2) (2, 3) (3, 3)
  EXPECT_EQ((*iter1).x, 2U);
  EXPECT_EQ((*iter1).y, 2U);

  nav_grid_iterators::CircleFill iter2 = iter1;

  // Effective Copy
  EXPECT_EQ((*iter1).x, 2U);
  EXPECT_EQ((*iter1).y, 2U);
  EXPECT_EQ((*iter2).x, 2U);
  EXPECT_EQ((*iter2).y, 2U);

  // Increment only iter2
  ++iter2;
  EXPECT_EQ((*iter1).x, 2U);
  EXPECT_EQ((*iter1).y, 2U);
  EXPECT_EQ((*iter2).x, 3U);
  EXPECT_EQ((*iter2).y, 2U);

  // Increment first to match
  ++iter1;
  EXPECT_EQ((*iter1).x, 3U);
  EXPECT_EQ((*iter1).y, 2U);
  EXPECT_EQ((*iter2).x, 3U);
  EXPECT_EQ((*iter2).y, 2U);

  // Increment only iter2
  ++iter2;
  EXPECT_EQ((*iter1).x, 3U);
  EXPECT_EQ((*iter1).y, 2U);
  EXPECT_EQ((*iter2).x, 2U);
  EXPECT_EQ((*iter2).y, 3U);

  // Check copy when not at the start
  nav_grid_iterators::CircleFill iter3 = iter1;
  EXPECT_EQ((*iter1).x, 3U);
  EXPECT_EQ((*iter1).y, 2U);
  EXPECT_EQ((*iter2).x, 2U);
  EXPECT_EQ((*iter2).y, 3U);
  EXPECT_EQ((*iter3).x, 3U);
  EXPECT_EQ((*iter3).y, 2U);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
