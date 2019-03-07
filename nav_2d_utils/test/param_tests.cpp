/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
*                2018, Locus Robotics
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Dave Hershberger
*         David V. Lu!! (nav_2d_utils version)
*********************************************************************/
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nav_2d_utils/polygons.h>

using nav_2d_utils::polygonFromParams;
using nav_2d_msgs::Polygon2D;

TEST(Polygon2D, unpadded_footprint_from_string_param)
{
  ros::NodeHandle nh("~unpadded");
  Polygon2D footprint = polygonFromParams(nh, "footprint");
  ASSERT_EQ(3U, footprint.points.size());

  EXPECT_EQ(1.0f, footprint.points[ 0 ].x);
  EXPECT_EQ(1.0f, footprint.points[ 0 ].y);

  EXPECT_EQ(-1.0f, footprint.points[ 1 ].x);
  EXPECT_EQ(1.0f, footprint.points[ 1 ].y);

  EXPECT_EQ(-1.0f, footprint.points[ 2 ].x);
  EXPECT_EQ(-1.0f, footprint.points[ 2 ].y);
}

TEST(Polygon2D, check_search_capabilities)
{
  ros::NodeHandle nh("~unpadded/unneccessarily/long_namespace");
  Polygon2D footprint = polygonFromParams(nh, "footprint");
  ASSERT_EQ(3U, footprint.points.size());
  EXPECT_THROW(polygonFromParams(nh, "footprint", false), nav_2d_utils::PolygonParseException);
}

TEST(Polygon2D, footprint_from_xmlrpc_param)
{
  ros::NodeHandle nh("~xmlrpc");
  Polygon2D footprint = polygonFromParams(nh, "footprint");
  ASSERT_EQ(4U, footprint.points.size());

  EXPECT_FLOAT_EQ(0.1f, footprint.points[ 0 ].x);
  EXPECT_FLOAT_EQ(0.1f, footprint.points[ 0 ].y);

  EXPECT_FLOAT_EQ(-0.1f, footprint.points[ 1 ].x);
  EXPECT_FLOAT_EQ(0.1f, footprint.points[ 1 ].y);

  EXPECT_FLOAT_EQ(-0.1f, footprint.points[ 2 ].x);
  EXPECT_FLOAT_EQ(-0.1f, footprint.points[ 2 ].y);

  EXPECT_FLOAT_EQ(0.1f, footprint.points[ 3 ].x);
  EXPECT_FLOAT_EQ(-0.1f, footprint.points[ 3 ].y);

  Polygon2D footprint2 = polygonFromParams(nh, "footprint2");
  ASSERT_TRUE(nav_2d_utils::equals(footprint, footprint2));
}

TEST(Polygon2D, footprint_from_same_level_param)
{
  ros::NodeHandle nh("~same_level");
  Polygon2D footprint = polygonFromParams(nh, "footprint");
  ASSERT_EQ(3U, footprint.points.size());

  EXPECT_EQ(1.0f, footprint.points[ 0 ].x);
  EXPECT_EQ(2.0f, footprint.points[ 0 ].y);

  EXPECT_EQ(3.0f, footprint.points[ 1 ].x);
  EXPECT_EQ(4.0f, footprint.points[ 1 ].y);

  EXPECT_EQ(5.0f, footprint.points[ 2 ].x);
  EXPECT_EQ(6.0f, footprint.points[ 2 ].y);
}

TEST(Polygon2D, footprint_from_xmlrpc_param_failure)
{
  ros::NodeHandle nh("~xmlrpc_fail");
  EXPECT_THROW(polygonFromParams(nh, "footprint"), nav_2d_utils::PolygonParseException);
  EXPECT_THROW(polygonFromParams(nh, "footprint2"), nav_2d_utils::PolygonParseException);
  EXPECT_THROW(polygonFromParams(nh, "footprint3"), nav_2d_utils::PolygonParseException);
  EXPECT_THROW(polygonFromParams(nh, "footprint4"), nav_2d_utils::PolygonParseException);
  EXPECT_THROW(polygonFromParams(nh, "footprint5"), nav_2d_utils::PolygonParseException);
  EXPECT_THROW(polygonFromParams(nh, "footprint6"), nav_2d_utils::PolygonParseException);
  EXPECT_THROW(polygonFromParams(nh, "footprint7"), nav_2d_utils::PolygonParseException);
  EXPECT_THROW(polygonFromParams(nh, "footprint8"), nav_2d_utils::PolygonParseException);
  EXPECT_THROW(polygonFromParams(nh, "footprint9"), nav_2d_utils::PolygonParseException);
}

TEST(Polygon2D, footprint_empty)
{
  ros::NodeHandle nh("~empty");
  EXPECT_THROW(polygonFromParams(nh, "footprint"), nav_2d_utils::PolygonParseException);
}

TEST(Polygon2D, test_write)
{
  ros::NodeHandle nh("~unpadded");
  Polygon2D footprint = polygonFromParams(nh, "footprint");
  nh.setParam("another_footprint", nav_2d_utils::polygonToXMLRPC(footprint));
  Polygon2D another_footprint = polygonFromParams(nh, "another_footprint");
  EXPECT_TRUE(nav_2d_utils::equals(footprint, another_footprint));

  nh.setParam("third_footprint", nav_2d_utils::polygonToXMLRPC(footprint, false));
  another_footprint = polygonFromParams(nh, "third_footprint");
  EXPECT_TRUE(nav_2d_utils::equals(footprint, another_footprint));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "param_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
