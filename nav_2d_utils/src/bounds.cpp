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
#include <algorithm>
#include <stdexcept>
#include <vector>

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

std::vector<nav_core2::UIntBounds> divideBounds(const nav_core2::UIntBounds& original_bounds,
                                                unsigned int n_cols, unsigned int n_rows)
{
  if (n_cols * n_rows == 0)
  {
    throw std::invalid_argument("Number of rows and columns must be positive (not zero)");
  }
  unsigned int full_width = original_bounds.getWidth(),
               full_height = original_bounds.getHeight();

  unsigned int small_width = static_cast<unsigned int>(ceil(static_cast<double>(full_width) / n_cols)),
               small_height = static_cast<unsigned int>(ceil(static_cast<double>(full_height) / n_rows));

  std::vector<nav_core2::UIntBounds> divided;

  for (unsigned int row = 0; row < n_rows; row++)
  {
    unsigned int min_y = original_bounds.getMinY() + small_height * row;
    unsigned int max_y = std::min(min_y + small_height - 1, original_bounds.getMaxY());

    for (unsigned int col = 0; col < n_cols; col++)
    {
      unsigned int min_x = original_bounds.getMinX() + small_width * col;
      unsigned int max_x = std::min(min_x + small_width - 1, original_bounds.getMaxX());
      nav_core2::UIntBounds sub(min_x, min_y, max_x, max_y);
      if (!sub.isEmpty())
        divided.push_back(sub);
    }
  }
  return divided;
}
}  // namespace nav_2d_utils
