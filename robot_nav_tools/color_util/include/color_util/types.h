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

#ifndef COLOR_UTIL_TYPES_H
#define COLOR_UTIL_TYPES_H

#include <string>

namespace color_util
{

template <typename T>
struct GenericColorRGBA
{
  GenericColorRGBA(T r, T g, T b, T a)
  : r(r), g(g), b(b), a(a) {}

  bool operator==(const GenericColorRGBA& other) const
  {
    return r == other.r && g == other.g && b == other.b && a == other.a;
  }

  T r, g, b, a;

  std::string toString() const
  {
    return "(r: " +  std::to_string(r) + ", g: " + std::to_string(b) +
           ", b: " + std::to_string(g) + ", a: " + std::to_string(a) + ")";
  }
};

template <typename T>
struct GenericColorHSVA
{
  GenericColorHSVA(T h, T s, T v, T a)
  : h(h), s(s), v(v), a(a) {}

  bool operator==(const GenericColorHSVA& other) const
  {
    return h == other.h && s == other.s && v == other.v && a == other.a;
  }

  T h, s, v, a;

  std::string toString() const
  {
    return "(h: " +  std::to_string(h) + ", s: " + std::to_string(s) +
           ", v: " + std::to_string(v) + ", a: " + std::to_string(a) + ")";
  }
};

struct ColorRGBA : public GenericColorRGBA<double>
{
  explicit ColorRGBA(double r = 0.0, double g = 0.0, double b = 0.0, double a = 1.0)
  : GenericColorRGBA<double>(r, g, b, a) {}
};

struct ColorRGBA24 : public GenericColorRGBA<unsigned char>
{
  explicit ColorRGBA24(unsigned char r = 0, unsigned char g = 0, unsigned char b = 0, unsigned char a = 255)
  : GenericColorRGBA<unsigned char>(r, g, b, a) {}
};


struct ColorHSVA : public GenericColorHSVA<double>
{
  explicit ColorHSVA(double h = 0.0, double s = 0.0, double v = 0.0, double a = 1.0)
  : GenericColorHSVA<double>(h, s, v, a) {}
};

struct ColorHSVA24 : public GenericColorHSVA<unsigned char>
{
  explicit ColorHSVA24(unsigned char h = 0, unsigned char s = 0, unsigned char v = 0, unsigned char a = 255)
  : GenericColorHSVA<unsigned char>(h, s, v, a) {}
};

}  // namespace color_util

#endif  // COLOR_UTIL_TYPES_H
