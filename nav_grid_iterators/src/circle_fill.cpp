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

#include <nav_grid_iterators/circle_fill.h>
#include <nav_grid/coordinate_conversion.h>

namespace nav_grid_iterators
{
CircleFill::CircleFill(const nav_grid::NavGridInfo* info, double center_x, double center_y, double radius)
  : BaseIterator(info), center_x_(center_x), center_y_(center_y), start_index_(0, 0)
{
  radius_sq_ = radius * radius;

  double min_x = center_x_ - radius;
  double max_x = center_x_ + radius;
  double min_y = center_y_ - radius;
  double max_y = center_y_ + radius;

  // Calculate and save the minimum coordinates
  worldToGridBounded(*info_, min_x, min_y, min_x_, min_y_);

  // Calculate the max coordinates, and save the width/height
  unsigned int max_x_grid, max_y_grid;
  worldToGridBounded(*info_, max_x, max_y, max_x_grid, max_y_grid);

  width_ = max_x_grid - min_x_ + 1;
  height_ = max_y_grid - min_y_ + 1;

  // Initialize internal iterator
  internal_iterator_.reset(new SubGrid(info_, min_x_, min_y_, width_, height_));
  index_.x = min_x_;
  index_.y = min_y_;

  // Iterate to first valid index
  if (!isInside(min_x_, min_y_)) ++(*this);
  start_index_ = **internal_iterator_;
  index_ = start_index_;
}

CircleFill::CircleFill(const nav_grid_iterators::CircleFill& other)
  : CircleFill(other.info_, other.index_, other.center_x_, other.center_y_, other.radius_sq_,
               other.min_x_, other.min_y_, other.width_, other.height_, other.start_index_)
{
}

CircleFill::CircleFill(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, double center_x,
               double center_y, double radius_sq, unsigned int min_x, unsigned int min_y, unsigned int width,
               unsigned int height, const nav_grid::Index& start_index)
  : BaseIterator(info, index), center_x_(center_x), center_y_(center_y), radius_sq_(radius_sq),
    min_x_(min_x), min_y_(min_y), width_(width), height_(height), start_index_(start_index)
{
  internal_iterator_.reset(new SubGrid(info_, index_, min_x_, min_y_, width_, height_));
}

CircleFill& CircleFill::operator=(const CircleFill& other)
{
  info_ = other.info_;
  index_ = other.index_;
  center_x_ = other.center_x_;
  center_y_ = other.center_y_;
  radius_sq_ = other.radius_sq_;
  min_x_ = other.min_x_;
  min_y_ = other.min_y_;
  width_ = other.width_;
  height_ = other.height_;
  start_index_ = other.start_index_;
  internal_iterator_.reset(new SubGrid(info_, index_, min_x_, min_y_, width_, height_));
  return *this;
}

bool CircleFill::isInside(unsigned int x, unsigned int y) const
{
  double wx, wy;
  gridToWorld(*info_, x, y, wx, wy);
  double dx = wx - center_x_;
  double dy = wy - center_y_;
  return (dx * dx + dy * dy) < radius_sq_;
}

CircleFill CircleFill::begin() const
{
  return CircleFill(info_, start_index_, center_x_, center_y_, radius_sq_, min_x_, min_y_, width_, height_,
                    start_index_);
}

CircleFill CircleFill::end() const
{
  return CircleFill(info_, *internal_iterator_->end(), center_x_, center_y_, radius_sq_, min_x_, min_y_,
                    width_, height_, start_index_);
}

void CircleFill::increment()
{
  ++(*internal_iterator_);
  index_ = **internal_iterator_;
  while (*internal_iterator_ != internal_iterator_->end())
  {
    if (isInside(index_.x, index_.y))
      break;
    ++(*internal_iterator_);
    index_ = **internal_iterator_;
  }
}

bool CircleFill::fieldsEqual(const CircleFill& other)
{
  return center_x_ == other.center_x_ && center_y_ == other.center_y_ && radius_sq_ == other.radius_sq_;
}

}  // namespace nav_grid_iterators
