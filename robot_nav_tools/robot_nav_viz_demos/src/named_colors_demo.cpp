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
#include <color_util/named_colors.h>
#include <nav_grid/vector_nav_grid.h>
#include <nav_grid_pub_sub/nav_grid_publisher.h>
#include <vector>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "named_colors_demo");
  ros::NodeHandle nh("~");
  nav_grid::VectorNavGrid<unsigned char> grid;
  nav_grid::NavGridInfo info;

  std::vector<color_util::ColorRGBA24> named_colors = color_util::getNamedColors();
  unsigned int N = named_colors.size();
  ROS_INFO("%u", N);

  info.width = (N - 1) / 3;
  info.height = 3;
  info.resolution = 0.5;
  grid.setInfo(info);

  unsigned char v = 1;
  for (unsigned int y = 0; y < info.height; y++)
  {
    for (unsigned int x = 0; x < info.width; x++)
    {
      grid.setValue(x, y, v++);
    }
  }

  nav_grid_pub_sub::NavGridPublisher pub(grid);
  pub.init(nh);

  ros::Rate r(1);

  while (ros::ok())
  {
    pub.publish();
    r.sleep();
    ros::spinOnce();
  }

  return 0;
}
