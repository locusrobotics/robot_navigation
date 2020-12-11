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

#include <robot_nav_rviz_plugins/spectrum_palette.h>
#include <color_util/convert.h>
#include <color_util/blend.h>
#include <vector>

namespace robot_nav_rviz_plugins
{

SpectrumPalette::SpectrumPalette(const color_util::ColorRGBA24& color_a,
                                 const color_util::ColorRGBA24& color_b,
                                 bool transparent_minimum)
  : transparent_minimum_(transparent_minimum)
{
  color_a_ = color_util::toFloat(color_util::changeColorspace(color_a));
  color_b_ = color_util::toFloat(color_util::changeColorspace(color_b));
}

SpectrumPalette::SpectrumPalette(const color_util::ColorHSVA24& color_a,
                                 const color_util::ColorHSVA24& color_b,
                                 bool transparent_minimum)
  : transparent_minimum_(transparent_minimum)
{
  color_a_ = color_util::toFloat(color_a);
  color_b_ = color_util::toFloat(color_b);
}


bool SpectrumPalette::hasTransparency() const
{
  return transparent_minimum_ || color_a_.a < 1.0 || color_b_.a < 1.0;
}

std::vector<color_util::ColorRGBA24> SpectrumPalette::getColors() const
{
  std::vector<color_util::ColorRGBA24> colors(NUM_COLORS);
  unsigned int start;
  double denominator;

  if (transparent_minimum_)
  {
    colors[0] = color_util::ColorRGBA24(0, 0, 0, 0);
    start = 1;
    denominator = NUM_COLORS - 2;
  }
  else
  {
    start = 0;
    denominator = NUM_COLORS - 1;
  }

  for (unsigned int i = start; i < NUM_COLORS; i++)
  {
    double ratio = static_cast<double>(i - start) / denominator;
    colors[i] = color_util::toInt(color_util::changeColorspace(color_util::hueBlendPlus(color_a_, color_b_, ratio)));
  }
  return colors;
}

}  // namespace robot_nav_rviz_plugins
