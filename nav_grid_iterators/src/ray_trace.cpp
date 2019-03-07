/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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

#include <nav_grid_iterators/line/ray_trace.h>
#include <cmath>
#include <limits>

namespace nav_grid_iterators
{
RayTrace::RayTrace(double x0, double y0, double x1, double y1, bool include_last_index)
  : AbstractLineIterator(), x0_(x0), y0_(y0), x1_(x1), y1_(y1), include_last_index_(include_last_index)
{
  dx_ = std::abs(x1 - x0);
  dy_ = std::abs(y1 - y0);
  index_.x = static_cast<int>(floor(x0));
  index_.y = static_cast<int>(floor(y0));

  if (dx_ == 0)
  {
    loop_inc_x_ = 0;
    error_ = std::numeric_limits<double>::max();
  }
  else if (x1 > x0)
  {
    loop_inc_x_ = 1;
    error_ = (floor(x0) + 1 - x0) * dy_;
  }
  else
  {
    loop_inc_x_ = -1;
    error_ = (x0 - floor(x0)) * dy_;
  }

  if (dy_ == 0)
  {
    loop_inc_y_ = 0;
    error_ -= std::numeric_limits<double>::max();
  }
  else if (y1 > y0)
  {
    loop_inc_y_ = 1;
    error_ -= (floor(y0) + 1 - y0) * dx_;
  }
  else
  {
    loop_inc_y_ = -1;
    error_ -= (y0 - floor(y0)) * dx_;
  }

  /* Since we check if the index is equal to the second point in the line,
   * we have to check for this one edge case to ensure we don't get into a rounding
   * problem, resulting in an off-by-one error.
   */
  if (!include_last_index && x1 < x0 && y1 - floor(y1) == 0.0)
  {
    error_ += 1e-10;
  }

  initial_error_ = error_;

  // Special use case when start and end point are the same AND we want to include that point
  if (include_last_index && loop_inc_x_ == 0 && loop_inc_y_ == 0)
  {
    loop_inc_x_ = 1;
  }
}

RayTrace::RayTrace(const nav_grid::SignedIndex& index,
     double x0, double y0, double x1, double y1, bool include_last_index,
     double dx, double dy, double initial_error, int loop_inc_x, int loop_inc_y)
  : AbstractLineIterator(index), x0_(x0), y0_(y0), x1_(x1), y1_(y1), include_last_index_(include_last_index),
    dx_(dx), dy_(dy), error_(initial_error), initial_error_(initial_error),
    loop_inc_x_(loop_inc_x), loop_inc_y_(loop_inc_y)
{
}

RayTrace RayTrace::begin() const
{
  return RayTrace(nav_grid::SignedIndex(x0_, y0_), x0_, y0_, x1_, y1_, include_last_index_,
                  dx_, dy_, initial_error_, loop_inc_x_, loop_inc_y_);
}

RayTrace RayTrace::end() const
{
  int x_diff = abs(static_cast<int>(x0_) - static_cast<int>(x1_));
  int y_diff = abs(static_cast<int>(y0_) - static_cast<int>(y1_));
  double final_error = initial_error_ - dx_ * y_diff + dy_ * x_diff;
  RayTrace end(nav_grid::SignedIndex(x1_, y1_), x0_, y0_, x1_, y1_, include_last_index_,
               dx_, dy_, final_error, loop_inc_x_, loop_inc_y_);

  // If we want the last_index, return an iterator that is whatever is one-past the end coordinates
  if (include_last_index_)
    end.increment();
  return end;
}

void RayTrace::increment()
{
  if (error_ > 0.0)
  {
    index_.y += loop_inc_y_;
    error_ -= dx_;
  }
  else
  {
    index_.x += loop_inc_x_;
    error_ += dy_;
  }
}

nav_grid::SignedIndex RayTrace::getFinalIndex() const
{
  return end().index_;
}
}  // namespace nav_grid_iterators
