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

#include <nav_grid_iterators/polygon_outline.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_2d_utils/polygons.h>

namespace nav_grid_iterators
{
PolygonOutline::PolygonOutline(const nav_grid::NavGridInfo* info, nav_2d_msgs::Polygon2D polygon, bool bresenham)
  : BaseIterator(info), polygon_(polygon), start_index_(0, 0), bresenham_(bresenham), side_index_(0)
{
  if (polygon.points.size() == 0)
  {
    internal_iterator_.reset(new Line(info_, 0.0, 0.0, 0.0, 0.0, false, bresenham_));
    index_ = **internal_iterator_;
    start_index_ = index_;
    return;
  }
  loadSide();
  index_ = **internal_iterator_;
  start_index_ = index_;
}

PolygonOutline::PolygonOutline(const PolygonOutline& other)
  : PolygonOutline(other.info_, other.index_, other.polygon_, other.bresenham_, other.side_index_)
{
}

PolygonOutline::PolygonOutline(const nav_grid::NavGridInfo* info, const nav_grid::Index& index,
                               nav_2d_msgs::Polygon2D polygon, bool bresenham, unsigned int side_index)
  : BaseIterator(info, index), polygon_(polygon), start_index_(index), bresenham_(bresenham), side_index_(side_index)
{
  loadSide();
}

PolygonOutline& PolygonOutline::operator=(const PolygonOutline& other)
{
  info_ = other.info_;
  index_ = other.index_;
  polygon_ = other.polygon_;
  bresenham_ = other.bresenham_;
  side_index_ = other.side_index_;
  loadSide();
  return *this;
}

void PolygonOutline::loadSide()
{
  while (side_index_ < polygon_.points.size())
  {
    // The next index loops around to the first index
    unsigned int next_index = side_index_ + 1;
    if (next_index == polygon_.points.size())
    {
      next_index = 0;
    }

    internal_iterator_.reset(new Line(info_,
                                      polygon_.points[side_index_].x, polygon_.points[side_index_].y,
                                      polygon_.points[next_index].x, polygon_.points[next_index].y,
                                      false, bresenham_));

    if (*internal_iterator_ != internal_iterator_->end())
      break;
    ++side_index_;
  }
}

PolygonOutline PolygonOutline::begin() const
{
  return PolygonOutline(info_, start_index_, polygon_, bresenham_, 0);
}

PolygonOutline PolygonOutline::end() const
{
  // Since the polygon outline loops around to the original index when it is complete
  // the end iterator is represented by having the current side to be the invalid
  return PolygonOutline(info_, start_index_, polygon_, bresenham_, polygon_.points.size());
}

void PolygonOutline::increment()
{
  ++(*internal_iterator_);
  if (*internal_iterator_ == internal_iterator_->end())
  {
    ++side_index_;
    if (side_index_ == polygon_.points.size())
    {
      index_ = start_index_;
      return;
    }
    loadSide();
  }
  index_ = **internal_iterator_;
}

bool PolygonOutline::fieldsEqual(const PolygonOutline& other)
{
  return side_index_ == other.side_index_ && nav_2d_utils::equals(polygon_, other.polygon_) &&
         bresenham_ == other.bresenham_;
}

}  // namespace nav_grid_iterators
