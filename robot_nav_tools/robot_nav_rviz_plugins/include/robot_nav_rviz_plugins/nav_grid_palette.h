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

#ifndef ROBOT_NAV_RVIZ_PLUGINS_NAV_GRID_PALETTE_H
#define ROBOT_NAV_RVIZ_PLUGINS_NAV_GRID_PALETTE_H

#include <color_util/types.h>
#include <cmath>
#include <string>
#include <vector>

namespace robot_nav_rviz_plugins
{
/**
 * @brief A simple datastructure representing a palette of up to 256 24-bit colors
 *
 * Designed to be loaded via pluginlib
 */
class NavGridPalette
{
public:
  static const unsigned int NUM_COLORS = 256, NUM_CHANNELS = 4;

  virtual ~NavGridPalette() {}

  /**
   * @brief Unique descriptive name for this particular palette
   */
  virtual std::string getName() const = 0;

  /**
   * @brief The actual definition of the colors
   * @return vector of up to 256 colors. Undefined behavior if more are returned.
   */
  virtual std::vector<color_util::ColorRGBA24> getColors() const = 0;

  /**
   * @brief See if the palette has any transparent colors.
   *
   * Can be overridden by implementing classes to save iterations
   */
  virtual bool hasTransparency() const
  {
    for (const auto& color : getColors())
    {
      if (color.a < 255) return true;
    }
    return false;
  }
};

}  // namespace robot_nav_rviz_plugins

#endif  // ROBOT_NAV_RVIZ_PLUGINS_NAV_GRID_PALETTE_H
