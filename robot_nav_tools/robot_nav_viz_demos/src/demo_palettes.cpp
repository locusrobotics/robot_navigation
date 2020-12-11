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

#include <robot_nav_rviz_plugins/nav_grid_palette.h>
#include <string>
#include <vector>

namespace robot_nav_viz_demos
{
using color_util::ColorRGBA24;

class MegaPalette : public robot_nav_rviz_plugins::NavGridPalette
{
public:
  std::string getName() const override { return "mega"; }
  bool hasTransparency() const override { return true; }
  std::vector<ColorRGBA24> getColors() const override
  {
    std::vector<ColorRGBA24> colors(6);
    colors[0] = ColorRGBA24(0, 0, 0, 0);
    colors[1] = ColorRGBA24(0, 0, 0, 255);
    colors[2] = ColorRGBA24(0, 112, 236, 255);
    colors[3] = ColorRGBA24(0, 232, 216, 255);
    colors[4] = ColorRGBA24(252, 228, 160, 255);
    colors[5] = ColorRGBA24(255, 255, 255, 255);
    return colors;
  }
};

class GreenPalette : public robot_nav_rviz_plugins::NavGridPalette
{
public:
  std::string getName() const override { return "green"; }
  bool hasTransparency() const override { return true; }
  std::vector<ColorRGBA24> getColors() const override
  {
    std::vector<ColorRGBA24> colors(6);
    colors[0] = ColorRGBA24(0, 0, 0, 0);
    colors[1] = ColorRGBA24(0, 0, 0, 255);
    colors[2] = ColorRGBA24(0, 148, 0, 255);
    colors[3] = ColorRGBA24(252, 252, 252, 255);
    colors[4] = ColorRGBA24(252, 228, 160, 255);
    colors[5] = ColorRGBA24(255, 255, 255, 255);
    return colors;
  }
};

}  // namespace robot_nav_viz_demos

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_nav_viz_demos::MegaPalette, robot_nav_rviz_plugins::NavGridPalette)
PLUGINLIB_EXPORT_CLASS(robot_nav_viz_demos::GreenPalette, robot_nav_rviz_plugins::NavGridPalette)
