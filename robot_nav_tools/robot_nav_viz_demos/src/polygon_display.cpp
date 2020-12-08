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

#include <ros/ros.h>
#include <angles/angles.h>
#include <geometry_msgs/PolygonStamped.h>
#include <nav_2d_msgs/Polygon2DStamped.h>
#include <nav_2d_msgs/Polygon2DCollection.h>
#include <nav_2d_utils/conversions.h>
#include <nav_2d_utils/polygons.h>
#include <color_util/named_colors.h>
#include <color_util/convert.h>
#include <vector>

nav_2d_msgs::Polygon2D makeStar(double base_angle, double outer = 3.0, double inner = 1.0, unsigned int N = 5)
{
  nav_2d_msgs::Polygon2D polygon;
  polygon.points.resize(2 * N + 1);

  double inc = M_PI / N;
  for (unsigned int i = 0 ; i < 2 * N; ++i)
  {
    double length = i % 2 ? outer : inner;
    polygon.points[i].x = length * cos(base_angle + i * inc);
    polygon.points[i].y = length * sin(base_angle + i * inc);
  }
  polygon.points[2 * N] = polygon.points[0];
  return polygon;
}

using color_util::NamedColor;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "polygon_display");
  ros::NodeHandle nh("~");
  ros::Rate r(33);

  ros::Publisher pub0 = nh.advertise<nav_2d_msgs::Polygon2DStamped>("polygon", 1);
  ros::Publisher pub1 = nh.advertise<nav_2d_msgs::Polygon2DCollection>("polygons", 1);
  ros::Publisher pub2 = nh.advertise<geometry_msgs::PolygonStamped>("polygon3d", 1);
  double degrees = 0.0;

  std::vector<color_util::ColorRGBA24> rainbow =
  {
    color_util::get(NamedColor::RED),
    color_util::get(NamedColor::ORANGE),
    color_util::get(NamedColor::YELLOW),
    color_util::get(NamedColor::GREEN),
    color_util::get(NamedColor::BLUE),
    color_util::get(NamedColor::PURPLE)
  };

  while (ros::ok())
  {
    nav_2d_msgs::Polygon2DStamped polygon;
    polygon.header.stamp = ros::Time::now();
    polygon.header.frame_id = "map";
    polygon.polygon.points.resize(11);

    double rad = angles::from_degrees(degrees);
    polygon.polygon = makeStar(rad);
    pub0.publish(polygon);

    nav_2d_msgs::Polygon2DCollection polygons;
    polygons.header.stamp = ros::Time::now();
    polygons.header.frame_id = "map";

    for (unsigned int i = 0; i < rainbow.size(); ++i)
    {
      nav_2d_msgs::ComplexPolygon2D complex;
      double w = i * 0.2;
      complex.outer = makeStar(rad, 4.2 + w, 2.2 + w);
      complex.inner.push_back(makeStar(rad, 4.0 + w, 2.0 + w));

      polygons.polygons.push_back(complex);
      polygons.colors.push_back(color_util::toMsg(rainbow[i]));
    }
    pub1.publish(polygons);

    geometry_msgs::Pose2D pose;
    pose.x = 5.0 * cos(rad);
    pose.y = 5.0 * sin(rad);

    nav_2d_msgs::Polygon2DStamped small_star;
    small_star.header.stamp = ros::Time::now();
    small_star.header.frame_id = "map";
    small_star.polygon = nav_2d_utils::movePolygonToPose(makeStar(-rad, 0.5, 0.25), pose);

    pub2.publish(nav_2d_utils::polygon2Dto3D(small_star));

    r.sleep();
    ros::spinOnce();
    degrees += 1;
  }

  return 0;
}
