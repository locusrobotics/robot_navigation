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
#include <robot_nav_rviz_plugins/spectrum_palette.h>
#include <color_util/convert.h>
#include <color_util/named_colors.h>
#include <string>
#include <vector>

namespace robot_nav_rviz_plugins
{
using color_util::ColorRGBA24;

/**
 * @brief Same as rviz::MapDisplay map palette.
 * See https://github.com/ros-visualization/rviz/blob/4b6c0f447159044bfaa633e140e6094b19516b02/src/rviz/default_plugin/map_display.cpp#L286
 */
class MapPalette : public NavGridPalette
{
public:
  std::string getName() const override { return "map"; }
  bool hasTransparency() const override { return false; }
  std::vector<ColorRGBA24> getColors() const override
  {
    std::vector<ColorRGBA24> colors(NUM_COLORS);
    // Standard gray map palette values
    for (int i = 0; i <= 100; i++)
    {
      unsigned char v = 255 - (255 * i) / 100;
      colors[i] = ColorRGBA24(v, v, v);
    }
    // illegal positive values in green
    for (int i = 101; i <= 127; i++)
    {
      colors[i] = ColorRGBA24(0, 255, 0);
    }
    // illegal negative (char) values in shades of red/yellow
    for (int i = 128; i <= 254; i++)
    {
      colors[i] = ColorRGBA24(255, (255 * (i - 128)) / (254 - 128), 0);
    }

    // legal 255 value is tasteful blueish greenish grayish color
    colors[255] = ColorRGBA24(0x70, 0x89, 0x86);

    return colors;
  }
};
/**
 * @brief Same as rviz::MapDisplay costmap palette.
 * See https://github.com/ros-visualization/rviz/blob/4b6c0f447159044bfaa633e140e6094b19516b02/src/rviz/default_plugin/map_display.cpp#324
 */
class CostmapPalette : public NavGridPalette
{
public:
  std::string getName() const override { return "costmap"; }
  bool hasTransparency() const override { return true; }
  std::vector<ColorRGBA24> getColors() const override
  {
    std::vector<ColorRGBA24> colors(NUM_COLORS);

    // zero values have alpha=0
    colors[0] = ColorRGBA24(0, 0, 0, 0);

    // Blue to red spectrum for most normal cost values
    for (int i = 1; i <= 98; i++)
    {
      unsigned char v = (255 * i) / 100;
      colors[i] = ColorRGBA24(v, 0, 255 - v);
    }

    // inscribed obstacle values (99) in cyan
    colors[99] = ColorRGBA24(0, 255, 255);

    // lethal obstacle values (100) in purple
    colors[100] = ColorRGBA24(255, 0, 255);

    // illegal positive values in green
    for (int i = 101; i <= 127; i++)
    {
      colors[i] = ColorRGBA24(0, 255, 0);
    }

    // illegal negative (char) values in shades of red/yellow
    for (int i = 128; i <= 254; i++)
    {
      colors[i] = ColorRGBA24(255, (255 * (i - 128)) / (254 - 128), 0);
    }

    // legal 255 value is tasteful blueish greenish grayish color
    colors[255] = ColorRGBA24(0x70, 0x89, 0x86);

    return colors;
  }
};

/**
 * @brief Same as rviz::MapDisplay raw palette.
 * See https://github.com/ros-visualization/rviz/blob/4b6c0f447159044bfaa633e140e6094b19516b02/src/rviz/default_plugin/map_display.cpp#379
 */
class RawPalette : public NavGridPalette
{
public:
  std::string getName() const override { return "raw"; }
  bool hasTransparency() const override { return false; }
  std::vector<ColorRGBA24> getColors() const override
  {
    std::vector<ColorRGBA24> colors(NUM_COLORS);
    for (unsigned int i = 0; i < NUM_COLORS; i++)
    {
      colors[i] = ColorRGBA24(i, i, i);
    }
    return colors;
  }
};

/**
 * @brief Rainbow hued palette from purple to red with transparent 0
 */
class RainbowPalette : public NavGridPalette
{
public:
  std::string getName() const override { return "rainbow"; }
  bool hasTransparency() const override { return true; }
  std::vector<ColorRGBA24> getColors() const override
  {
    std::vector<ColorRGBA24> colors(NUM_COLORS);
    colors[0] = ColorRGBA24(0, 0, 0, 0);
    for (unsigned int i = 1; i < NUM_COLORS; i++)
    {
      double fraction = static_cast<double>(255 - i) / 254;
      // multiplying by 0.75 scales it to purple to red
      color_util::ColorHSVA color(fraction * 0.75, 1.0, 1.0, 1.0);
      colors[i] = color_util::toInt(color_util::changeColorspace(color));
    }
    return colors;
  }
};

/**
 * @brief Rainbow hued palette from red to purple with transparent 0
 */
class Rainbow2Palette : public NavGridPalette
{
public:
  std::string getName() const override { return "rainbow2"; }
  bool hasTransparency() const override { return true; }
  std::vector<ColorRGBA24> getColors() const override
  {
    std::vector<ColorRGBA24> colors(NUM_COLORS);
    colors[0] = ColorRGBA24(0, 0, 0, 0);
    for (unsigned int i = 1; i < NUM_COLORS; i++)
    {
      double fraction = static_cast<double>(i - 1) / 254;
      // multiplying by 0.75 scales it to red to purple
      color_util::ColorHSVA color(fraction * 0.75, 1.0, 1.0, 1.0);
      colors[i] = color_util::toInt(color_util::changeColorspace(color));
    }
    return colors;
  }
};

/**
 * @brief Palette with a gradient of blues from dark to light
 */
class BluesPalette : public SpectrumPalette
{
public:
  // Blend from black to blue
  BluesPalette()
    : SpectrumPalette(color_util::ColorRGBA24(0, 0, 0), color_util::ColorRGBA24(0, 0, 255), true) {}
  std::string getName() const override { return "blues"; }
};

/**
 * @brief List of distinct colors for displaying individual categories.
 */
class DistinctPalette : public NavGridPalette
{
public:
  std::string getName() const override { return "distinct"; }
  bool hasTransparency() const override { return true; }

  std::vector<ColorRGBA24> getColors() const override
  {
    return color_util::getNamedColors();
  }
};


}  // namespace robot_nav_rviz_plugins

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::MapPalette,      robot_nav_rviz_plugins::NavGridPalette)
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::CostmapPalette,  robot_nav_rviz_plugins::NavGridPalette)
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::RawPalette,      robot_nav_rviz_plugins::NavGridPalette)
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::RainbowPalette,  robot_nav_rviz_plugins::NavGridPalette)
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::Rainbow2Palette, robot_nav_rviz_plugins::NavGridPalette)
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::BluesPalette,    robot_nav_rviz_plugins::NavGridPalette)
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::DistinctPalette, robot_nav_rviz_plugins::NavGridPalette)
