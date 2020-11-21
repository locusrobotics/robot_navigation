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
#include <gtest/gtest.h>
#include <color_util/convert.h>

using color_util::toFloat;
using color_util::toInt;
using color_util::changeColorspace;

TEST(color_util, full_color)
{
  // use unsigned ints because unsigned char would overflow
  unsigned int inc = 1;  // can increase this to speed up tests
  for (unsigned int i = 0; i < 256; i+=inc)
  {
    for (unsigned int j = 0; j < 256; j+=inc)
    {
      for (unsigned int k = 0; k < 256; k+=inc)
      {
        for (unsigned int a = 0; a < 256; a+=50)  // don't test all alpha values
        {
          color_util::ColorRGBA24 rgba24(i, j, k, a);
          color_util::ColorHSVA24 hsva24(i, j, k, a);

          color_util::ColorRGBA rgba = toFloat(rgba24);
          color_util::ColorHSVA hsva = toFloat(hsva24);

          ASSERT_EQ(rgba24, toInt(rgba));
          ASSERT_EQ(hsva24, toInt(hsva));
        }
      }
    }
  }
}

void checkConversion(double r, double g, double b, double h, double s, double v, double epsilon = 0.001)
{
  double a = 0.9;  // arbitrary value

  color_util::ColorRGBA rgba(r, g, b, a);
  color_util::ColorHSVA hsva(h, s, v, a);

  color_util::ColorRGBA converted_rgba = changeColorspace(hsva);

  ASSERT_NEAR(converted_rgba.r, r, epsilon);
  ASSERT_NEAR(converted_rgba.g, g, epsilon);
  ASSERT_NEAR(converted_rgba.b, b, epsilon);
  ASSERT_NEAR(converted_rgba.a, a, epsilon);

  color_util::ColorHSVA converted_hsva = changeColorspace(rgba);
  ASSERT_NEAR(converted_hsva.h, h, epsilon);
  ASSERT_NEAR(converted_hsva.s, s, epsilon);
  ASSERT_NEAR(converted_hsva.v, v, epsilon);
  ASSERT_NEAR(converted_hsva.a, a, epsilon);
}

TEST(color_util, random_color_check)
{
  // black
  checkConversion(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  // white
  checkConversion(1.0, 1.0, 1.0, 0.0, 0.0, 1.0);

  // red
  checkConversion(1.0, 0.0, 0.0, 0.0, 1.0, 1.0);

  // green
  checkConversion(0.0, 1.0, 0.0, 0.33333, 1.0, 1.0);

  // blue
  checkConversion(0.0, 0.0, 1.0, 0.66667, 1.0, 1.0);

  // cyan
  checkConversion(0.0, 1.0, 1.0, 0.5, 1.0, 1.0);

  // magenta
  checkConversion(1.0, 0.0, 1.0, 0.83333, 1.0, 1.0);

  // yellow
  checkConversion(1.0, 1.0, 0.0, 0.16667, 1.0, 1.0);

  // gray
  checkConversion(0.8, 0.8, 0.8, 0.0, 0.0, 0.8);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
