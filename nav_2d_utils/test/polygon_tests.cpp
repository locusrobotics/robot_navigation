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
#include <nav_2d_utils/polygons.h>
#include <vector>

using nav_2d_utils::parseVVD;
using nav_2d_msgs::Polygon2D;
using nav_2d_utils::polygonFromString;
using nav_2d_utils::polygonFromParallelArrays;

TEST(array_parser, basic_operation)
{
  std::vector<std::vector<double> > vvd;
  vvd = parseVVD("[[1, 2.2], [.3, -4e4]]");
  EXPECT_DOUBLE_EQ(2U, vvd.size());
  EXPECT_DOUBLE_EQ(2U, vvd[0].size());
  EXPECT_DOUBLE_EQ(2U, vvd[1].size());
  EXPECT_DOUBLE_EQ(1.0, vvd[0][0]);
  EXPECT_DOUBLE_EQ(2.2, vvd[0][1]);
  EXPECT_DOUBLE_EQ(0.3, vvd[1][0]);
  EXPECT_DOUBLE_EQ(-40000.0, vvd[1][1]);
}

TEST(array_parser, missing_open)
{
  EXPECT_THROW(parseVVD("[1, 2.2], [.3, -4e4]]"), nav_2d_utils::PolygonParseException);
}

TEST(array_parser, missing_close)
{
  EXPECT_THROW(parseVVD("[[1, 2.2], [.3, -4e4]"), nav_2d_utils::PolygonParseException);
}

TEST(array_parser, wrong_depth)
{
  EXPECT_THROW(parseVVD("[1, 2.2], [.3, -4e4]"), nav_2d_utils::PolygonParseException);
}

TEST(Polygon2D, radius_param)
{
  Polygon2D footprint = nav_2d_utils::polygonFromRadius(10.0);
  // Circular robot has 16-point footprint auto-generated.
  ASSERT_EQ(16U, footprint.points.size());

  // Check the first point
  EXPECT_EQ(10.0, footprint.points[0].x);
  EXPECT_EQ(0.0, footprint.points[0].y);

  // Check the 4th point, which should be 90 degrees around the circle from the first.
  EXPECT_NEAR(0.0, footprint.points[4].x, 0.0001);
  EXPECT_NEAR(10.0, footprint.points[4].y, 0.0001);
}

TEST(Polygon2D, string_param)
{
  Polygon2D footprint = polygonFromString("[[1, 1], [-1, 1], [-1, -1]]");
  ASSERT_EQ(3U, footprint.points.size());

  EXPECT_EQ(1.0, footprint.points[ 0 ].x);
  EXPECT_EQ(1.0, footprint.points[ 0 ].y);

  EXPECT_EQ(-1.0, footprint.points[ 1 ].x);
  EXPECT_EQ(1.0, footprint.points[ 1 ].y);

  EXPECT_EQ(-1.0, footprint.points[ 2 ].x);
  EXPECT_EQ(-1.0, footprint.points[ 2 ].y);
}

TEST(Polygon2D, broken_string_param)
{
  // Not enough points
  EXPECT_THROW(polygonFromString("[[1, 1], [-1, 1]]"), nav_2d_utils::PolygonParseException);

  // Too many numbers in point
  EXPECT_THROW(polygonFromString("[[1, 1, 1], [-1, 1], [-1, -1]]"), nav_2d_utils::PolygonParseException);

  // Unexpected character
  EXPECT_THROW(polygonFromString("[[x, 1], [-1, 1], [-1, -1]]"), nav_2d_utils::PolygonParseException);

  // Empty String
  EXPECT_THROW(polygonFromString(""), nav_2d_utils::PolygonParseException);

  // Empty List
  EXPECT_THROW(polygonFromString("[]"), nav_2d_utils::PolygonParseException);

  // Empty Point
  EXPECT_THROW(polygonFromString("[[]]"), nav_2d_utils::PolygonParseException);
}

TEST(Polygon2D, arrays)
{
  std::vector<double> xs = {1, -1, -1};
  std::vector<double> ys = {1, 1, -1};
  Polygon2D footprint = polygonFromParallelArrays(xs, ys);
  ASSERT_EQ(3U, footprint.points.size());

  EXPECT_EQ(1.0, footprint.points[ 0 ].x);
  EXPECT_EQ(1.0, footprint.points[ 0 ].y);

  EXPECT_EQ(-1.0, footprint.points[ 1 ].x);
  EXPECT_EQ(1.0, footprint.points[ 1 ].y);

  EXPECT_EQ(-1.0, footprint.points[ 2 ].x);
  EXPECT_EQ(-1.0, footprint.points[ 2 ].y);
}

TEST(Polygon2D, broken_arrays)
{
  std::vector<double> shorty = {1, -1};
  std::vector<double> three = {1, 1, -1};
  std::vector<double> four = {1, 1, -1, -1};
  EXPECT_THROW(polygonFromParallelArrays(shorty, shorty), nav_2d_utils::PolygonParseException);
  EXPECT_THROW(polygonFromParallelArrays(three, four), nav_2d_utils::PolygonParseException);
}

TEST(Polygon2D, test_move)
{
  Polygon2D square = polygonFromString("[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]");
  geometry_msgs::Pose2D pose;
  Polygon2D square2 = nav_2d_utils::movePolygonToPose(square, pose);
  EXPECT_TRUE(nav_2d_utils::equals(square, square2));
  pose.x = 15;
  pose.y = -10;
  pose.theta = M_PI / 4;
  Polygon2D diamond = nav_2d_utils::movePolygonToPose(square, pose);
  ASSERT_EQ(4U, diamond.points.size());
  double side = 1.0 / sqrt(2);

  EXPECT_DOUBLE_EQ(pose.x,        diamond.points[ 0 ].x);
  EXPECT_DOUBLE_EQ(pose.y + side, diamond.points[ 0 ].y);
  EXPECT_DOUBLE_EQ(pose.x + side, diamond.points[ 1 ].x);
  EXPECT_DOUBLE_EQ(pose.y,        diamond.points[ 1 ].y);
  EXPECT_DOUBLE_EQ(pose.x,        diamond.points[ 2 ].x);
  EXPECT_DOUBLE_EQ(pose.y - side, diamond.points[ 2 ].y);
  EXPECT_DOUBLE_EQ(pose.x - side, diamond.points[ 3 ].x);
  EXPECT_DOUBLE_EQ(pose.y,        diamond.points[ 3 ].y);
}

TEST(Polygon2D, inside)
{
  Polygon2D square = polygonFromString("[[0.5, 0.5], [0.5, -0.5], [-0.5, -0.5], [-0.5, 0.5]]");
  EXPECT_TRUE(nav_2d_utils::isInside(square, 0.00, 0.00));
  EXPECT_TRUE(nav_2d_utils::isInside(square, 0.45, 0.45));
  EXPECT_FALSE(nav_2d_utils::isInside(square, 0.50, 0.50));
  EXPECT_FALSE(nav_2d_utils::isInside(square, 0.00, 0.50));
  EXPECT_FALSE(nav_2d_utils::isInside(square, 0.55, 0.55));
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
