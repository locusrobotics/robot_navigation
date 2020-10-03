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

#include <nav_2d_utils/bounds.h>
#include <nav_grid/coordinate_conversion.h>

namespace nav_2d_utils
{
nav_core2::Bounds getFullBounds(const nav_grid::NavGridInfo& info)
{
  return nav_core2::Bounds(info.origin_x, info.origin_y,
                           info.origin_x + info.resolution * info.width, info.origin_y + info.resolution * info.height);
}

nav_core2::UIntBounds getFullUIntBounds(const nav_grid::NavGridInfo& info)
{
  // bounds are inclusive, so we subtract one
  return nav_core2::UIntBounds(0, 0, info.width - 1, info.height - 1);
}

nav_core2::UIntBounds translateBounds(const nav_grid::NavGridInfo& info, const nav_core2::Bounds& bounds)
{
  unsigned int g_min_x, g_min_y, g_max_x, g_max_y;
  worldToGridBounded(info, bounds.getMinX(), bounds.getMinY(), g_min_x, g_min_y);
  worldToGridBounded(info, bounds.getMaxX(), bounds.getMaxY(), g_max_x, g_max_y);
  return nav_core2::UIntBounds(g_min_x, g_min_y, g_max_x, g_max_y);
}

nav_core2::Bounds translateBounds(const nav_grid::NavGridInfo& info, const nav_core2::UIntBounds& bounds)
{
  double min_x, min_y, max_x, max_y;
  gridToWorld(info, bounds.getMinX(), bounds.getMinY(), min_x, min_y);
  gridToWorld(info, bounds.getMaxX(), bounds.getMaxY(), max_x, max_y);
  return nav_core2::Bounds(min_x, min_y, max_x, max_y);
}
}  // namespace nav_2d_utils
