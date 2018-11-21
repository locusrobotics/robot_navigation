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

#include <nav_grid_iterators/circle_outline.h>
#include <nav_grid/coordinate_conversion.h>

namespace nav_grid_iterators
{
CircleOutline::CircleOutline(const nav_grid::NavGridInfo* info, double center_x, double center_y, double radius)
  : CircleOutline(info, center_x, center_y, static_cast<unsigned int>(ceil(radius / info->resolution)))
{
}

CircleOutline::CircleOutline(const nav_grid::NavGridInfo* info, double center_x, double center_y, unsigned int radius)
  : BaseIterator(info), distance_(radius), init_(false)
{
  signed_width_ = static_cast<int>(info->width);
  signed_height_ = static_cast<int>(info->height);

  // Calculate and save the center coordinates
  worldToGrid(*info_, center_x, center_y, center_index_x_, center_index_y_);

  point_x_ = distance_;
  point_y_ = 0;

  if (!isValidIndex(center_index_x_ + point_x_, center_index_y_ + point_y_))
  {
    increment();
    init_ = !isValidIndex(center_index_x_ + point_x_, center_index_y_ + point_y_);
  }
  index_.x = center_index_x_ + point_x_;
  index_.y = center_index_y_ + point_y_;
  start_index_ = index_;
}

CircleOutline::CircleOutline(const nav_grid::NavGridInfo* info, const nav_grid::Index& index,
                             int center_index_x, int center_index_y, unsigned int distance,
                             bool init, const nav_grid::Index& start_index)
  : BaseIterator(info, index),
    center_index_x_(center_index_x), center_index_y_(center_index_y), distance_(distance), init_(init),
    start_index_(start_index)
{
  signed_width_ = static_cast<int>(info->width);
  signed_height_ = static_cast<int>(info->height);
  point_x_ = distance_;
  point_y_ = 0;
}

CircleOutline CircleOutline::begin() const
{
  return CircleOutline(info_, start_index_, center_index_x_, center_index_y_,
                       distance_, false, start_index_);
}

CircleOutline CircleOutline::end() const
{
  return CircleOutline(info_, start_index_, center_index_x_, center_index_y_,
                       distance_, true, start_index_);
}

void CircleOutline::increment()
{
  init_ = true;
  while (true)
  {
    int nx = -signum(point_y_);
    int ny = signum(point_x_);
    if (nx != 0 && isOnOutline(point_x_ + nx, point_y_))
    {
      point_x_ += nx;
    }
    else if (ny != 0 && isOnOutline(point_x_, point_y_ + ny))
    {
      point_y_ += ny;
    }
    else
    {
      point_x_ += nx;
      point_y_ += ny;
    }

    if (isValidIndex(center_index_x_ + point_x_, center_index_y_ + point_y_))
    {
      break;
    }
    if (point_x_ == static_cast<int>(distance_) && point_y_ == 0)
    {
      index_ = start_index_;
      return;
    }
  }
  index_.x = center_index_x_ + point_x_;
  index_.y = center_index_y_ + point_y_;
}

bool CircleOutline::fieldsEqual(const CircleOutline& other)
{
  return center_index_x_ == other.center_index_x_ && center_index_y_ == other.center_index_y_ &&
         distance_ == other.distance_ && init_ == other.init_;
}

bool CircleOutline::isValidIndex(int x, int y) const
{
  return x >= 0 && y >= 0 && x < signed_width_ && y < signed_height_;
}

bool CircleOutline::isOnOutline(int dx, int dy) const
{
  return static_cast<unsigned int>(hypot(dx, dy)) == distance_;
}

}  // namespace nav_grid_iterators
