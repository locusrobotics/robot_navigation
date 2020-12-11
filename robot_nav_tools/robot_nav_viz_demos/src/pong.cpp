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

#include <nav_grid/vector_nav_grid.h>
#include <nav_grid_iterators/circle_fill.h>
#include <nav_grid_pub_sub/nav_grid_publisher.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pong");
  ros::NodeHandle nh("~");
  nav_grid::VectorNavGrid<double> grid;
  nav_grid::NavGridInfo info;

  int size = 100;
  nh.param("size", size, size);
  info.width = size * 2;
  info.height = size;
  info.resolution = 0.1;
  grid.setInfo(info);

  double radius = 4.0;
  nh.param("radius", radius, radius);
  radius *= info.resolution;

  double x = 0.0, y = size * info.resolution / 3;
  double dx = info.resolution, dy = info.resolution;
  double counter = 1.0;

  double max_x = info.resolution * info.width;
  double max_y = info.resolution * info.height;

  bool publish_updates = true;
  nh.param("updates", publish_updates, publish_updates);

  nav_grid_pub_sub::ScaleGridPublisher<double> pub(grid);
  pub.init(nh);

  ros::Rate r(33);

  while (ros::ok())
  {
    nav_core2::UIntBounds bounds;
    for (const nav_grid::Index& i : nav_grid_iterators::CircleFill(&info, x, y, radius))
    {
      grid.setValue(i, counter);
      bounds.touch(i.x, i.y);
    }
    counter += 1.0;
    x += dx;
    y += dy;
    if (x > max_x || x < 0.0) dx *= -1;
    if (y > max_y || y < 0.0) dy *= -1;
    if (publish_updates)
        pub.publish(bounds);
    else
        pub.publish();
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
