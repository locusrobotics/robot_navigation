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

#include <nav_grid_iterators/polygon_fill.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_2d_utils/polygons.h>
#include <algorithm>

namespace nav_grid_iterators
{
PolygonFill::PolygonFill(const nav_grid::NavGridInfo* info, nav_2d_msgs::Polygon2D polygon)
  : BaseIterator(info), polygon_(polygon), start_index_(0, 0)
{
  if (polygon.points.size() == 0)
  {
    internal_iterator_.reset(new SubGrid(info_, 0, 0, 0, 0));
    start_index_ = **internal_iterator_;
    index_ = start_index_;
    return;
  }

  // Find the minimum and maximum coordinates of the vertices
  double min_x = polygon_.points[0].x;
  double max_x = min_x;
  double min_y = polygon_.points[0].y;
  double max_y = min_y;
  for (const auto& vertex : polygon_.points)
  {
    min_x = std::min(min_x, vertex.x);
    min_y = std::min(min_y, vertex.y);
    max_x = std::max(max_x, vertex.x);
    max_y = std::max(max_y, vertex.y);
  }

  // Save the minimum in grid coordinates
  worldToGridBounded(*info_, min_x, min_y, min_x_, min_y_);

  // Calculate the maximum in grid coordinates and then save the width/height
  unsigned int max_x_grid, max_y_grid;
  worldToGridBounded(*info_, max_x, max_y, max_x_grid, max_y_grid);
  width_ = max_x_grid - min_x_ + 1;
  height_ = max_y_grid - min_y_ + 1;

  // Initialize internal iterator
  internal_iterator_.reset(new SubGrid(info_, min_x_, min_y_, width_, height_));
  index_.x = min_x_;
  index_.y = min_y_;

  // Iterate to first valid index
  if (!isInside(index_.x, index_.y)) ++(*this);
  start_index_ = **internal_iterator_;
  index_ = start_index_;
}

PolygonFill::PolygonFill(const PolygonFill& other)
  : PolygonFill(other.info_, other.index_, other.polygon_, other.min_x_, other.min_y_, other.width_, other.height_,
                other.start_index_)
{
}

PolygonFill::PolygonFill(const nav_grid::NavGridInfo* info, const nav_grid::Index& index,
               nav_2d_msgs::Polygon2D polygon,
               unsigned int min_x, unsigned int min_y, unsigned int width, unsigned int height,
               const nav_grid::Index& start_index)
  : BaseIterator(info, index), polygon_(polygon),
    min_x_(min_x), min_y_(min_y), width_(width), height_(height), start_index_(start_index)
{
  internal_iterator_.reset(new SubGrid(info_, min_x_, min_y_, width_, height_));
}

PolygonFill& PolygonFill::operator=(const PolygonFill& other)
{
  info_ = other.info_;
  index_ = other.index_;
  polygon_ = other.polygon_;
  min_x_ = other.min_x_;
  min_y_ = other.min_y_;
  width_ = other.width_;
  height_ = other.height_;
  start_index_ = other.start_index_;
  internal_iterator_.reset(new SubGrid(info_, index_, min_x_, min_y_, width_, height_));
  return *this;
}

bool PolygonFill::isInside(unsigned int x, unsigned int y) const
{
  // Determine if the current index is inside the polygon using the number of crossings method
  double wx, wy;
  gridToWorld(*info_, x, y, wx, wy);
  return nav_2d_utils::isInside(polygon_, wx, wy);
}

PolygonFill PolygonFill::begin() const
{
  return PolygonFill(info_, start_index_, polygon_, min_x_, min_y_, width_, height_, start_index_);
}

PolygonFill PolygonFill::end() const
{
  return PolygonFill(info_, *internal_iterator_->end(), polygon_, min_x_, min_y_, width_, height_,
                start_index_);
}

void PolygonFill::increment()
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

bool PolygonFill::fieldsEqual(const PolygonFill& other)
{
  return nav_2d_utils::equals(polygon_, other.polygon_);
}

}  // namespace nav_grid_iterators
