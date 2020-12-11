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

#ifndef ROBOT_NAV_RVIZ_PLUGINS_SPECTRUM_PALETTE_H
#define ROBOT_NAV_RVIZ_PLUGINS_SPECTRUM_PALETTE_H

#include <robot_nav_rviz_plugins/nav_grid_palette.h>
#include <vector>

namespace robot_nav_rviz_plugins
{
/**
 * @brief Easy class to generate palettes that simply blend two colors together
 *
 * Implementing classes need only override the default constructor to call this class's
 * constructor and the getName function.
 */
class SpectrumPalette : public NavGridPalette
{
public:
  /**
   * @brief Constructor for a blend of colors from color_a to color_b
   * @param color_a Color used for low values
   * @param color_b Color used for high values
   * @param transparent_minimum Whether the lowest value should be completely transparent
   */
  SpectrumPalette(const color_util::ColorRGBA24& color_a,
                  const color_util::ColorRGBA24& color_b,
                  bool transparent_minimum = true);

  /**
   * @brief Constructor for a blend of colors from color_a to color_b
   * @param color_a Color used for low values
   * @param color_b Color used for high values
   * @param transparent_minimum Whether the lowest value should be completely transparent
   */
  SpectrumPalette(const color_util::ColorHSVA24& color_a,
                  const color_util::ColorHSVA24& color_b,
                  bool transparent_minimum = true);

  bool hasTransparency() const override;

  std::vector<color_util::ColorRGBA24> getColors() const override;
protected:
  color_util::ColorHSVA color_a_, color_b_;
  bool transparent_minimum_;
};
}  // namespace robot_nav_rviz_plugins

#endif  // ROBOT_NAV_RVIZ_PLUGINS_SPECTRUM_PALETTE_H
