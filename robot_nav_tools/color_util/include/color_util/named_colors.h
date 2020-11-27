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

#ifndef COLOR_UTIL_NAMED_COLORS_H
#define COLOR_UTIL_NAMED_COLORS_H

#include <color_util/types.h>
#include <vector>

namespace color_util
{
enum class NamedColor
{
  TRANSPARENT,

  RED,
  GREEN,
  YELLOW,
  BLUE,
  ORANGE,
  PURPLE,
  CYAN,
  MAGENTA,
  LIME,
  PINK,
  TEAL,
  LAVENDER,
  BROWN,
  MAROON,
  MINT,
  OLIVE,
  NAVY,
  GREY,

  LIGHT_RED,
  LIGHT_GREEN,
  LIGHT_YELLOW,
  LIGHT_BLUE,
  LIGHT_ORANGE,
  LIGHT_PURPLE,
  LIGHT_CYAN,
  LIGHT_MAGENTA,
  LIGHT_LIME,
  LIGHT_PINK,
  LIGHT_TEAL,
  LIGHT_LAVENDER,
  LIGHT_BROWN,
  LIGHT_MAROON,
  LIGHT_MINT,
  LIGHT_OLIVE,
  LIGHT_NAVY,
  LIGHT_GREY,

  DARK_RED,
  DARK_GREEN,
  DARK_YELLOW,
  DARK_BLUE,
  DARK_ORANGE,
  DARK_PURPLE,
  DARK_CYAN,
  DARK_MAGENTA,
  DARK_LIME,
  DARK_PINK,
  DARK_TEAL,
  DARK_LAVENDER,
  DARK_BROWN,
  DARK_MAROON,
  DARK_MINT,
  DARK_OLIVE,
  DARK_NAVY,
  DARK_GREY,
};

inline const std::vector<ColorRGBA24>& getNamedColors()
{
  // Source: Slightly modified from https://sashat.me/2017/01/11/list-of-20-simple-distinct-colors/
  static const std::vector<ColorRGBA24> colors =
  {
    ColorRGBA24(0x00, 0x00, 0x00, 0x00),  // transparent

    ColorRGBA24(0xe6, 0x19, 0x4B),  // #e6194B red
    ColorRGBA24(0x3c, 0xb4, 0x4b),  // #3cb44b green
    ColorRGBA24(0xff, 0xe1, 0x19),  // #ffe119 yellow
    ColorRGBA24(0x43, 0x63, 0xd8),  // #4363d8 blue
    ColorRGBA24(0xf5, 0x82, 0x31),  // #f58231 orange
    ColorRGBA24(0x91, 0x1e, 0xb4),  // #911eb4 purple
    ColorRGBA24(0x42, 0xd4, 0xf4),  // #42d4f4 cyan
    ColorRGBA24(0xf0, 0x32, 0xe6),  // #f032e6 magenta
    ColorRGBA24(0xbf, 0xef, 0x45),  // #bfef45 lime
    ColorRGBA24(0xFB, 0x19, 0x7B),  // #FB197B pink
    ColorRGBA24(0x46, 0x99, 0x90),  // #469990 teal
    ColorRGBA24(0xb8, 0x45, 0xff),  // #b845ff lavender
    ColorRGBA24(0x9A, 0x63, 0x24),  // #9A6324 brown
    ColorRGBA24(0x80, 0x00, 0x00),  // #800000 maroon
    ColorRGBA24(0x89, 0xcc, 0x9d),  // #89cc9d mint
    ColorRGBA24(0x95, 0x95, 0x47),  // #959547 olive
    ColorRGBA24(0x00, 0x00, 0x75),  // #000075 navy
    ColorRGBA24(0xa9, 0xa9, 0xa9),  // #a9a9a9 grey

    ColorRGBA24(0xEA, 0x60, 0x82),  // #EA6082 light red
    ColorRGBA24(0x7A, 0xC1, 0x82),  // #7AC182 light green
    ColorRGBA24(0xFF, 0xFE, 0x95),  // #FFFE95 light yellow
    ColorRGBA24(0xB0, 0xBA, 0xDF),  // #B0BADF light blue
    ColorRGBA24(0xF7, 0xC8, 0xA8),  // #F7C8A8 light orange
    ColorRGBA24(0xB2, 0x7E, 0xC1),  // #B27EC1 light purple
    ColorRGBA24(0xB9, 0xEB, 0xF7),  // #B9EBF7 light cyan
    ColorRGBA24(0xF2, 0xA7, 0xEE),  // #F2A7EE light magenta
    ColorRGBA24(0xE2, 0xF2, 0xBB),  // #E2F2BB light lime
    ColorRGBA24(0xfa, 0x90, 0xbe),  // #fa90be light pink
    ColorRGBA24(0xAE, 0xE5, 0xDF),  // #AEE5DF light teal
    ColorRGBA24(0xe6, 0xbe, 0xff),  // #e6beff light lavender
    ColorRGBA24(0xAA, 0x93, 0x79),  // #AA9379 light brown
    ColorRGBA24(0xCC, 0x00, 0x00),  // #CC0000 light maroon
    ColorRGBA24(0xaa, 0xff, 0xc3),  // #aaffc3 light mint
    ColorRGBA24(0xCC, 0xCC, 0x00),  // #CCCC00 light olive
    ColorRGBA24(0x3A, 0x3A, 0xC2),  // #3A3AC2 light navy
    ColorRGBA24(0xDB, 0xDB, 0xDB),  // #DBDBDB light grey

    ColorRGBA24(0x66, 0x0B, 0x22),  // #660B22 dark red
    ColorRGBA24(0x12, 0x36, 0x16),  // #123616 dark green
    ColorRGBA24(0x80, 0x70, 0x0D),  // #80700D dark yellow
    ColorRGBA24(0x1C, 0x29, 0x59),  // #1C2959 dark blue
    ColorRGBA24(0x75, 0x3F, 0x17),  // #753F17 dark orange
    ColorRGBA24(0x2B, 0x09, 0x36),  // #2B0936 dark purple
    ColorRGBA24(0x20, 0x66, 0x75),  // #206675 dark cyan
    ColorRGBA24(0x70, 0x18, 0x6C),  // #70186C dark magenta
    ColorRGBA24(0x5A, 0x70, 0x21),  // #5A7021 dark lime
    ColorRGBA24(0x7A, 0x47, 0x5D),  // #7A475D dark pink
    ColorRGBA24(0x2F, 0x66, 0x60),  // #2F6660 dark teal
    ColorRGBA24(0x73, 0x60, 0x80),  // #736080 dark lavender
    ColorRGBA24(0x66, 0x41, 0x17),  // #664117 dark brown
    ColorRGBA24(0x4D, 0x00, 0x00),  // #4D0000 dark maroon
    ColorRGBA24(0x2F, 0x80, 0x47),  // #2F8047 dark mint
    ColorRGBA24(0x80, 0x80, 0x00),  // #808000 dark olive
    ColorRGBA24(0x00, 0x00, 0x42),  // #000042 dark navy
    ColorRGBA24(0x75, 0x75, 0x75),  // #757575 dark grey
  };

  return colors;
}

inline const ColorRGBA24& get(const NamedColor& name)
{
  return getNamedColors()[static_cast<unsigned int>(name)];
}


}  // namespace color_util

#endif  // COLOR_UTIL_NAMED_COLORS_H
