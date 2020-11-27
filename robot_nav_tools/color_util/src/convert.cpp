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

#include <color_util/convert.h>

namespace color_util
{

inline float toFloat(unsigned char n)
{
  return static_cast<float>(n) / 255.0;
}

color_util::ColorRGBA toFloat(const color_util::ColorRGBA24& int_color)
{
  color_util::ColorRGBA float_color;
  float_color.r = toFloat(int_color.r);
  float_color.g = toFloat(int_color.g);
  float_color.b = toFloat(int_color.b);
  float_color.a = toFloat(int_color.a);
  return float_color;
}

color_util::ColorHSVA toFloat(const color_util::ColorHSVA24& int_color)
{
  color_util::ColorHSVA float_color;
  float_color.h = toFloat(int_color.h);
  float_color.s = toFloat(int_color.s);
  float_color.v = toFloat(int_color.v);
  float_color.a = toFloat(int_color.a);
  return float_color;
}

inline unsigned char toInt(float n)
{
  return static_cast<unsigned char>(n * 255.0);
}

color_util::ColorRGBA24 toInt(const color_util::ColorRGBA& float_color)
{
  color_util::ColorRGBA24 int_color;
  int_color.r = toInt(float_color.r);
  int_color.g = toInt(float_color.g);
  int_color.b = toInt(float_color.b);
  int_color.a = toInt(float_color.a);
  return int_color;
}

color_util::ColorHSVA24 toInt(const color_util::ColorHSVA& float_color)
{
  color_util::ColorHSVA24 int_color;
  int_color.h = toInt(float_color.h);
  int_color.s = toInt(float_color.s);
  int_color.v = toInt(float_color.v);
  int_color.a = toInt(float_color.a);
  return int_color;
}

// Colorspace conversions based on
// https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
color_util::ColorHSVA changeColorspace(const color_util::ColorRGBA& rgba)
{
  color_util::ColorHSVA out;

  // Alpha channel
  out.a = rgba.a;

  // Three way max/min
  double min = rgba.r < rgba.g ? rgba.r : rgba.g;
  min = min    < rgba.b ? min    : rgba.b;

  double max = rgba.r > rgba.g ? rgba.r : rgba.g;
  max = max    > rgba.b ? max    : rgba.b;

  // Value Channel
  out.v = max;

  double delta = max - min;
  if (max == 0.0 || delta < 0.00001)
  {
    // Grayscale
    out.s = 0;
    out.h = 0;  // undefined hue
    return out;
  }

  // Saturation Channel (note max!=0)
  out.s = (delta / max);

  if (rgba.r >= max)                          // > is invalid for valid input
  {
    out.h = (rgba.g - rgba.b) / delta;        // between yellow & magenta
  }
  else if (rgba.g >= max)
  {
    out.h = 2.0 + (rgba.b - rgba.r) / delta;  // between cyan & yellow
  }
  else
  {
    out.h = 4.0 + (rgba.r - rgba.g) / delta;  // between magenta & cyan
  }

  out.h *= 60.0;                              // convert to degrees

  if (out.h < 0.0)                            // convert to positive degrees
  {
    out.h += 360.0;
  }
  out.h /= 360.0;                             // convert to be [0, 1]

  return out;
}

color_util::ColorHSVA24 changeColorspace(const color_util::ColorRGBA24& rgba)
{
  return toInt(changeColorspace(toFloat(rgba)));
}

color_util::ColorRGBA changeColorspace(const color_util::ColorHSVA& hsva)
{
  color_util::ColorRGBA out;

  // Alpha channel
  out.a = hsva.a;

  if (hsva.s <= 0.0)  // < is invalid for valid input
  {
    // Grayscale
    out.r = hsva.v;
    out.g = hsva.v;
    out.b = hsva.v;
    return out;
  }

  double hh = hsva.h * 360.0;
  if (hh >= 360.0) hh = 0.0;
  hh /= 60.0;

  int i = static_cast<int>(hh);
  double ff = hh - i;
  double p = hsva.v * (1.0 - hsva.s);
  double q = hsva.v * (1.0 - (hsva.s * ff));
  double t = hsva.v * (1.0 - (hsva.s * (1.0 - ff)));

  switch (i)
  {
  case 0:
    out.r = hsva.v;
    out.g = t;
    out.b = p;
    break;
  case 1:
    out.r = q;
    out.g = hsva.v;
    out.b = p;
    break;
  case 2:
    out.r = p;
    out.g = hsva.v;
    out.b = t;
    break;
  case 3:
    out.r = p;
    out.g = q;
    out.b = hsva.v;
    break;
  case 4:
    out.r = t;
    out.g = p;
    out.b = hsva.v;
    break;
  case 5:
  default:
    out.r = hsva.v;
    out.g = p;
    out.b = q;
    break;
  }
  return out;
}

color_util::ColorRGBA24 changeColorspace(const color_util::ColorHSVA24& hsva)
{
  return toInt(changeColorspace(toFloat(hsva)));
}

std_msgs::ColorRGBA toMsg(const color_util::ColorRGBA& rgba)
{
  std_msgs::ColorRGBA msg;
  msg.r = rgba.r;
  msg.g = rgba.g;
  msg.b = rgba.b;
  msg.a = rgba.a;
  return msg;
}

std_msgs::ColorRGBA toMsg(const color_util::ColorRGBA24& rgba)
{
  return toMsg(toFloat(rgba));
}

std_msgs::ColorRGBA toMsg(const color_util::ColorHSVA& hsva)
{
  return toMsg(changeColorspace(hsva));
}

std_msgs::ColorRGBA toMsg(const color_util::ColorHSVA24& hsva)
{
  return toMsg(changeColorspace(hsva));
}


}  // namespace color_util
