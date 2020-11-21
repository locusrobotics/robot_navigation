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

#ifndef COLOR_UTIL_BLEND_H
#define COLOR_UTIL_BLEND_H

#include <color_util/types.h>
#include <algorithm>

namespace color_util
{
/**
 * @brief Return a color that is a linear blending of color_a and color_b in rgba space
 * @param color_a
 * @param color_b
 * @param ratio value in range [0.0, 1.0]
 * @return color_a * (1 - ratio) + color_b * ratio
 */
template <typename rgba>
inline rgba rgbaBlend(const rgba& color_a, const rgba& color_b, double ratio)
{
  ratio = std::min(ratio, 1.0);
  ratio = std::max(ratio, 0.0);

  double i_ratio = 1.0 - ratio;

  rgba color;
  color.r = color_a.r * i_ratio + color_b.r * ratio;
  color.g = color_a.g * i_ratio + color_b.g * ratio;
  color.b = color_a.b * i_ratio + color_b.b * ratio;
  color.a = color_a.a * i_ratio + color_b.a * ratio;
  return color;
}

/**
 * @brief Return a color that is a linear blending of color_a and color_b in hsv space
 * @param color_a
 * @param color_b
 * @param ratio value in range [0.0, 1.0]
 * @return color_a * (1 - ratio) + color_b * ratio
 */
template <typename hsva>
inline hsva hueBlend(const hsva& color_a, const hsva& color_b, double ratio)
{
  ratio = std::min(ratio, 1.0);
  ratio = std::max(ratio, 0.0);

  double i_ratio = 1.0 - ratio;

  hsva color;
  color.h = color_a.h * i_ratio + color_b.h * ratio;
  color.s = color_a.s * i_ratio + color_b.s * ratio;
  color.v = color_a.v * i_ratio + color_b.v * ratio;
  color.a = color_a.a * i_ratio + color_b.a * ratio;
  return color;
}

/**
 * @brief Return a color that blends color_a and color_b in hsv space, using the shortest distance between the hues
 *
 * Note the shortest distance between the hues may wrap around 1.0
 *
 * Only available in floating point because it assumes the hue is [0.0, 1.0]
 *
 * @param color_a
 * @param color_b
 * @param ratio value in range [0.0, 1.0]
 * @return blend between color_a and color_b
 */
inline color_util::ColorHSVA hueBlendPlus(const color_util::ColorHSVA& color_a,
                                          const color_util::ColorHSVA& color_b,
                                          double ratio)
{
  ratio = std::min(ratio, 1.0);
  ratio = std::max(ratio, 0.0);

  double i_ratio = 1.0 - ratio;

  // Direct interpolation for saturation/value/alpha
  color_util::ColorHSVA color;
  color.s = color_a.s * i_ratio + color_b.s * ratio;
  color.v = color_a.v * i_ratio + color_b.v * ratio;
  color.a = color_a.a * i_ratio + color_b.a * ratio;

  // Hue interpolation
  double start_h, end_h, diff;
  if (color_a.h > color_b.h)
  {
    start_h = color_b.h;
    end_h = color_a.h;
    ratio = i_ratio;
  }
  else
  {
    start_h = color_a.h;
    end_h = color_b.h;
  }
  diff = end_h - start_h;

  // If the hue difference is greater than 0.5, interpolate the other way around
  if (diff > 0.5)  // 180deg
  {
    start_h = start_h + 1;  // 360deg
    double ipart;
    color.h = modf(start_h + ratio * (end_h - start_h), &ipart);  // 360deg
  }
  else
  {
    color.h = start_h + ratio * diff;
  }
  return color;
}


}  // namespace color_util

#endif  // COLOR_UTIL_BLEND_H
