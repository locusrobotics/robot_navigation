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

#ifndef NAV_GRID_ITERATORS_POLYGON_FILL_H
#define NAV_GRID_ITERATORS_POLYGON_FILL_H

#include <nav_grid_iterators/base_iterator.h>
#include <nav_grid_iterators/sub_grid.h>
#include <nav_2d_msgs/Polygon2D.h>
#include <memory>

namespace nav_grid_iterators
{
/**
 * @class PolygonFill
 * @brief Iterates over all of the valid indexes that lie within an arbitrary polygon in row major order
 */
class PolygonFill : public BaseIterator<PolygonFill>
{
public:
  /**
   * @brief Public Constructor.
   * @param info NavGridInfo for the grid to iterate over
   * @param polygon Polygon to iterate over
   */
  PolygonFill(const nav_grid::NavGridInfo* info, nav_2d_msgs::Polygon2D polygon);

  /**
   * @brief Copy Constructor
   * Required to ensure unique_ptr is set properly
   */
  PolygonFill(const PolygonFill& other);

  /**
   * @brief Assignment Operator
   * Required to ensure unique_ptr is set properly
   */
  PolygonFill& operator=(const PolygonFill& other);

  /**@name Standard BaseIterator Interface */
  /**@{*/
  PolygonFill begin() const override;
  PolygonFill end() const override;
  void increment() override;
  bool fieldsEqual(const PolygonFill& other) override;
  /**@}*/

protected:
  /**
   * @brief Protected constructor that takes in an arbitrary index and other internal parameters
   * @param info NavGridInfo for the grid to iterate over
   * @param index Initial index
   * @param polygon Polygon to iterate over
   * @param min_x Minimum valid index that is within the polygon (x coordinate)
   * @param min_y Minimum valid index that is within the polygon (y coordinate)
   * @param width Maximum number of valid indexes in a row of the polygon
   * @param height Maximum number of of valid indexes in a column of the polygon
   * @param start_index The first valid index in the minimum row
   */
  PolygonFill(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, nav_2d_msgs::Polygon2D polygon,
              unsigned int min_x, unsigned int min_y, unsigned int width, unsigned int height,
              const nav_grid::Index& start_index);


  /**
   * @brief Check if given index is inside the polygon.
   * @return true if inside, false otherwise.
   */
  bool isInside(unsigned int x, unsigned int y) const;

  nav_2d_msgs::Polygon2D polygon_;
  unsigned int min_x_, min_y_, width_, height_;
  nav_grid::Index start_index_;
  std::unique_ptr<SubGrid> internal_iterator_;
};
}  // namespace nav_grid_iterators

#endif  // NAV_GRID_ITERATORS_POLYGON_FILL_H
