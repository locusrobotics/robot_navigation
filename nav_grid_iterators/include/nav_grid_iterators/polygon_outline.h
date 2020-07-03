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

#ifndef NAV_GRID_ITERATORS_POLYGON_OUTLINE_H
#define NAV_GRID_ITERATORS_POLYGON_OUTLINE_H

#include <nav_grid_iterators/base_iterator.h>
#include <nav_grid_iterators/line.h>
#include <nav_2d_msgs/Polygon2D.h>
#include <memory>

namespace nav_grid_iterators
{
/**
 * @class PolygonOutline
 * @brief Iterates over all of the valid indexes on the outline of a polygon
 */
class PolygonOutline : public BaseIterator<PolygonOutline>
{
public:
  /**
   * @brief Public Constructor.
   * @param info NavGridInfo for the grid to iterate over
   * @param polygon Polygon to iterate over
   */
  PolygonOutline(const nav_grid::NavGridInfo* info, nav_2d_msgs::Polygon2D polygon, bool bresenham = true);

  /**
   * @brief Copy Constructor
   * Required to ensure unique_ptr is set properly
   */
  PolygonOutline(const PolygonOutline& other);

  /**
   * @brief Assignment Operator
   * Required to ensure unique_ptr is set properly
   */
  PolygonOutline& operator=(const PolygonOutline& other);

  /**@name Standard BaseIterator Interface */
  /**@{*/
  PolygonOutline begin() const override;
  PolygonOutline end() const override;
  void increment() override;
  bool fieldsEqual(const PolygonOutline& other) override;
  /**@}*/

protected:
  /**
   * @brief Protected constructor that takes in an arbitrary index and other internal parameters
   * @param info NavGridInfo for the grid to iterate over
   * @param index Initial index
   * @param polygon Polygon to iterate over
   * @param side_index Which side we are currently iterating over
   */
  PolygonOutline(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, nav_2d_msgs::Polygon2D polygon,
                 bool bresenham, unsigned int side_index);

  /**
   * @brief Given a new side index, loads the internal iterator.
   * If there are no valid values in the internal iterator, increases the side index.
   */
  void loadSide();

  std::unique_ptr<Line> internal_iterator_;
  nav_2d_msgs::Polygon2D polygon_;
  nav_grid::Index start_index_;
  bool bresenham_;
  unsigned int side_index_;
};
}  // namespace nav_grid_iterators

#endif  // NAV_GRID_ITERATORS_POLYGON_OUTLINE_H
