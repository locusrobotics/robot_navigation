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

#include <nav_grid_iterators/sub_grid.h>

namespace nav_grid_iterators
{
SubGrid::SubGrid(const nav_grid::NavGridInfo* info, const nav_grid::Index& index,
                 unsigned int min_x, unsigned int min_y,
                 unsigned int width, unsigned int height)
  : BaseIterator(info, index), min_x_(min_x), min_y_(min_y), width_(width), height_(height)
{
  // If the start coordinate is entirely off the grid or the size is 0
  // we invalidate the entire iterator and give up immediately
  if (min_x_ >= info->width || min_y_ >= info->height || width_ == 0 || height_ == 0)
  {
    index_ = nav_grid::Index(0, 0);
    width_ = 0;
    height_ = 0;
    min_x_ = 0;
    min_y_ = 0;
    return;
  }

  // If the end coordinate is off the grid, we shorten the dimensions to
  // cover the on-grid potion
  if (min_x_ + width_ > info->width)
  {
    width_ = info->width - min_x_;
  }
  if (min_y_ + height_ > info->height)
  {
    height_ = info->height - min_y_;
  }
}

SubGrid SubGrid::begin() const
{
  return SubGrid(info_, min_x_, min_y_, width_, height_);
}

SubGrid SubGrid::end() const
{
  return SubGrid(info_, nav_grid::Index(min_x_, min_y_ + height_), min_x_, min_y_, width_, height_);
}

void SubGrid::increment()
{
  ++index_.x;
  if (index_.x >= min_x_ + width_)
  {
    index_.x = min_x_;
    ++index_.y;
  }
}

bool SubGrid::fieldsEqual(const SubGrid& other)
{
  return min_x_ == other.min_x_ && min_y_ == other.min_y_ && width_ == other.width_ && height_ == other.height_;
}

}  // namespace nav_grid_iterators
