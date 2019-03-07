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

#include <nav_grid_iterators/line/bresenham.h>
#include <cmath>

namespace nav_grid_iterators
{

Bresenham::Bresenham(int x0, int y0, int x1, int y1, bool include_last_index)
  : AbstractLineIterator(nav_grid::SignedIndex(x0, y0)), x0_(x0), y0_(y0), x1_(x1), y1_(y1),
    include_last_index_(include_last_index)
{
  int dx = std::abs(x1_ - x0_);
  int dy = std::abs(y1_ - y0_);
  int xsign = x1_ >= x0_ ? 1 : -1;
  int ysign = y1_ >= y0_ ? 1 : -1;

  if (dx >= dy)      // There is at least one x-value for every y-value
  {
    loop_inc_x_ = xsign;
    error_inc_x_ = 0;       // Don't change the x when numerator >= denominator
    loop_inc_y_ = 0;        // Don't change the y for every iteration
    error_inc_y_ = ysign;

    denominator_ = dx;
    numerator_ = dx / 2;
    numerator_inc_ = dy;
  }
  else               // There is at least one y-value for every x-value
  {
    loop_inc_x_ = 0;        // Don't change the x for every iteration
    error_inc_x_ = xsign;
    loop_inc_y_ = ysign;
    error_inc_y_ = 0;       // Don't change the y when numerator >= denominator

    denominator_ = dy;
    numerator_ = dy / 2;
    numerator_inc_ = dx;
  }
}

Bresenham::Bresenham(const nav_grid::SignedIndex& index,
           int x0, int y0, int x1, int y1, bool include_last_index,
           int error_inc_x, int loop_inc_x, int error_inc_y, int loop_inc_y,
           int denominator, int numerator, int numerator_inc)
  : AbstractLineIterator(index), x0_(x0), y0_(y0), x1_(x1), y1_(y1), include_last_index_(include_last_index),
    error_inc_x_(error_inc_x), loop_inc_x_(loop_inc_x), error_inc_y_(error_inc_y), loop_inc_y_(loop_inc_y),
    denominator_(denominator), numerator_(numerator), numerator_inc_(numerator_inc)
{
}

Bresenham Bresenham::begin() const
{
  return Bresenham(nav_grid::SignedIndex(x0_, y0_), x0_, y0_, x1_, y1_, include_last_index_,
              error_inc_x_, loop_inc_x_, error_inc_y_, loop_inc_y_,
              denominator_, numerator_, numerator_inc_);
}

Bresenham Bresenham::end() const
{
  Bresenham end(nav_grid::SignedIndex(x1_, y1_), x0_, y0_, x1_, y1_, include_last_index_,
           error_inc_x_, loop_inc_x_, error_inc_y_, loop_inc_y_,
           denominator_, numerator_, numerator_inc_);

  // If we want the last_index, return an iterator that is whatever is one-past the end coordinates
  if (include_last_index_)
    end.increment();
  return end;
}

void Bresenham::increment()
{
  numerator_ += numerator_inc_;       // Increase the numerator by the top of the fraction
  if (numerator_ >= denominator_)     // Check if numerator >= denominator
  {
    numerator_ -= denominator_;       // Calculate the new numerator value
    index_.x += error_inc_x_;         // Change the x as appropriate
    index_.y += error_inc_y_;         // Change the y as appropriate
  }
  index_.x += loop_inc_x_;           // Change the x as appropriate
  index_.y += loop_inc_y_;           // Change the y as appropriate
}

nav_grid::SignedIndex Bresenham::getFinalIndex() const
{
  return end().index_;
}
}  // namespace nav_grid_iterators
