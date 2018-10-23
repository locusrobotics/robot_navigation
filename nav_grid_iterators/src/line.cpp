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

#include <nav_grid_iterators/line.h>
#include <nav_grid_iterators/line/bresenham.h>
#include <nav_grid_iterators/line/ray_trace.h>
#include <nav_grid/coordinate_conversion.h>

namespace nav_grid_iterators
{
Line::Line(const nav_grid::NavGridInfo* info, double x0, double y0, double x1, double y1,
           bool include_last_index, bool bresenham)
  : BaseIterator(info), x0_(x0), y0_(y0), x1_(x1), y1_(y1), include_last_index_(include_last_index),
    bresenham_(bresenham), start_index_(0, 0), end_index_(0, 0)
{
  constructIterator();

  // Convenience variables to avoid mismatched comparisons
  signed_width_ = static_cast<int>(info->width);
  signed_height_ = static_cast<int>(info->height);

  // Cache the end index
  nav_grid::SignedIndex end = internal_iterator_->getFinalIndex();
  end_index_.x = end.x;
  end_index_.y = end.y;

  // Iterate to first valid index
  nav_grid::SignedIndex sindex = **internal_iterator_;
  while (!internal_iterator_->isFinished() && !inBounds(sindex))
  {
    internal_iterator_->increment();
    sindex = **internal_iterator_;
  }

  // If all the indices are invalid, explicitly set the start index to be invalid
  if (internal_iterator_->isFinished())
  {
    start_index_ = end_index_;
  }
  else
  {
    start_index_.x = sindex.x;
    start_index_.y = sindex.y;
  }

  index_ = start_index_;
}

Line::Line(const Line& other)
  : Line(other.info_, other.index_, other.x0_, other.y0_, other.x1_, other.y1_, other.include_last_index_,
         other.bresenham_, other.start_index_, other.end_index_)
{
}

Line::Line(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, double x0, double y0, double x1, double y1,
           bool include_last_index, bool bresenham, nav_grid::Index start_index, nav_grid::Index end_index)
  : BaseIterator(info, index), x0_(x0), y0_(y0), x1_(x1), y1_(y1), include_last_index_(include_last_index),
    bresenham_(bresenham), start_index_(start_index), end_index_(end_index)
{
  constructIterator();
  signed_width_ = static_cast<int>(info->width);
  signed_height_ = static_cast<int>(info->height);
}

Line& Line::operator=(const Line& other)
{
  info_ = other.info_;
  index_ = other.index_;
  x0_ = other.x0_;
  y0_ = other.y0_;
  x1_ = other.x1_;
  y1_ = other.y1_;
  include_last_index_ = other.include_last_index_;
  bresenham_ = other.bresenham_;
  start_index_ = other.start_index_;
  end_index_ = other.end_index_;
  signed_width_ = other.signed_width_;
  signed_height_ = other.signed_height_;
  constructIterator();
  return *this;
}

void Line::constructIterator()
{
  // translate coordinates into grid coordinates
  double local_x0, local_y0, local_x1, local_y1;
  worldToGrid(*info_, x0_, y0_, local_x0, local_y0);
  worldToGrid(*info_, x1_, y1_, local_x1, local_y1);

  if (bresenham_)
  {
    internal_iterator_.reset(new Bresenham(local_x0, local_y0, local_x1, local_y1, include_last_index_));
  }
  else
  {
    internal_iterator_.reset(new RayTrace(local_x0, local_y0, local_x1, local_y1, include_last_index_));
  }
}

bool Line::inBounds(const nav_grid::SignedIndex& sindex)
{
  return sindex.x >= 0 && sindex.y >= 0 && sindex.x < signed_width_ && sindex.y < signed_height_;
}

Line Line::begin() const
{
  return Line(info_, start_index_, x0_, y0_, x1_, y1_, include_last_index_, bresenham_, start_index_, end_index_);
}

Line Line::end() const
{
  return Line(info_, end_index_, x0_, y0_, x1_, y1_, include_last_index_, bresenham_, start_index_, end_index_);
}

void Line::increment()
{
  internal_iterator_->increment();
  nav_grid::SignedIndex sindex = **internal_iterator_;
  if (!internal_iterator_->isFinished() && !inBounds(sindex))
  {
    index_ = end_index_;
  }
  else
  {
    index_.x = sindex.x;
    index_.y = sindex.y;
  }
}

bool Line::fieldsEqual(const Line& other)
{
  return x0_ == other.x0_ && y0_ == other.y0_ && x1_ == other.x1_ && y1_ == other.y1_ &&
         include_last_index_ == other.include_last_index_ && bresenham_ == other.bresenham_;
}

}  // namespace nav_grid_iterators
