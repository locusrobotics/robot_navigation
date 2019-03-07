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

#include <nav_grid_iterators/spiral.h>
#include <nav_grid/coordinate_conversion.h>

namespace nav_grid_iterators
{
Spiral::Spiral(const nav_grid::NavGridInfo* info, double center_x, double center_y, double radius)
  : BaseIterator(info), center_x_(center_x), center_y_(center_y), distance_(0), start_index_(0, 0)
{
  radius_sq_ = radius * radius;
  max_distance_ = ceil(radius / info->resolution);
  loadRing();
  index_ = **internal_iterator_;
  start_index_ = index_;
}

Spiral::Spiral(const Spiral& other)
  : Spiral(other.info_, other.index_, other.center_x_, other.center_y_, other.radius_sq_,
           other.distance_, other.max_distance_, other.start_index_)
{
}

Spiral::Spiral(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, double center_x, double center_y,
               double radius_sq, unsigned int distance, unsigned int max_distance,
               const nav_grid::Index& start_index)
  : BaseIterator(info, index), center_x_(center_x), center_y_(center_y), radius_sq_(radius_sq),
    distance_(distance), max_distance_(max_distance), start_index_(start_index)
{
  loadRing();
  if (distance_ < max_distance_)
  {
    index_ = **internal_iterator_;
    start_index_ = index_;
  }
}

Spiral& Spiral::operator=(const Spiral& other)
{
  info_ = other.info_;
  index_ = other.index_;
  center_x_ = other.center_x_;
  center_y_ = other.center_y_;
  radius_sq_ = other.radius_sq_;
  distance_ = other.distance_;
  max_distance_ = other.max_distance_;
  start_index_ = other.start_index_;
  loadRing();
  if (distance_ < max_distance_)
  {
    index_ = **internal_iterator_;
    start_index_ = index_;
  }
  return *this;
}

Spiral Spiral::begin() const
{
  return Spiral(info_, start_index_, center_x_, center_y_, radius_sq_, 0, max_distance_, start_index_);
}

Spiral Spiral::end() const
{
  return Spiral(info_, start_index_, center_x_, center_y_, radius_sq_, max_distance_ + 1, max_distance_,
                start_index_);
}

void Spiral::increment()
{
  while (distance_ <= max_distance_)
  {
    ++(*internal_iterator_);
    if (*internal_iterator_ == internal_iterator_->end())
    {
      ++distance_;
      if (distance_ > max_distance_)
      {
        index_ = start_index_;
        return;
      }
      loadRing();
    }
    index_ = **internal_iterator_;
    if (isInside(index_.x, index_.y))
    {
      break;
    }
  }
  if (distance_ > max_distance_)
  {
    index_ = start_index_;
  }
}

bool Spiral::fieldsEqual(const Spiral& other)
{
  return center_x_ == other.center_x_ && center_y_ == other.center_y_ &&
         radius_sq_ == other.radius_sq_ && distance_ == other.distance_;
}

void Spiral::loadRing()
{
  while (distance_ <= max_distance_)
  {
    internal_iterator_.reset(new CircleOutline(info_, center_x_, center_y_, distance_));

    if (*internal_iterator_ != internal_iterator_->end())
      break;
    ++distance_;
  }
}

bool Spiral::isInside(unsigned int x, unsigned int y) const
{
  double wx, wy;
  gridToWorld(*info_, x, y, wx, wy);
  double dx = wx - center_x_;
  double dy = wy - center_y_;
  return (dx * dx + dy * dy) < radius_sq_;
}

}  // namespace nav_grid_iterators
