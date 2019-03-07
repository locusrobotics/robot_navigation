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

#ifndef NAV_GRID_ITERATORS_LINE_H
#define NAV_GRID_ITERATORS_LINE_H

#include <nav_grid_iterators/base_iterator.h>
#include <nav_grid_iterators/line/abstract_line_iterator.h>
#include <memory>

namespace nav_grid_iterators
{
/**
 * @class Line
 * @brief Iterates over all of the valid indexes of a line
 */
class Line : public BaseIterator<Line>
{
public:
  /**
   * @brief Public Constructor.
   * @param info NavGridInfo for the grid to iterate over
   * @param x0 Start x coordinate
   * @param y0 Start y coordinate
   * @param x1 End x coordinate
   * @param y1 End y coordinate
   * @param include_last_index If true, will include the end coordinates.
   */
  Line(const nav_grid::NavGridInfo* info, double x0, double y0, double x1, double y1,
       bool include_last_index = true, bool bresenham = true);

  /**
   * @brief Copy Constructor
   * Required to ensure unique_ptr is set properly
   */
  Line(const Line& other);

  /**
   * @brief Assignment Operator
   * Required to ensure unique_ptr is set properly
   */
  Line& operator=(const Line& other);

  /**@name Standard BaseIterator Interface */
  /**@{*/
  Line begin() const override;
  Line end() const override;
  void increment() override;
  bool fieldsEqual(const Line& other) override;
  /**@}*/

protected:
  /**
   * @brief Protected constructor that takes in an arbitrary index and other internal parameters
   * @param info NavGridInfo for the grid to iterate over
   * @param index Initial index
   * @param x0 Start x coordinate
   * @param y0 Start y coordinate
   * @param x1 End x coordinate
   * @param y1 End y coordinate
   * @param include_last_index If true, will include the end coordinates.
   * @param start_index The first valid index
   * @param end_index The first invalid index
   */
  Line(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, double x0, double y0, double x1, double y1,
       bool include_last_index, bool bresenham, nav_grid::Index start_index, nav_grid::Index end_index);

  void constructIterator();

  /**
   * @brief Check if a SignedIndex is within the bounds of the NavGrid
   */
  bool inBounds(const nav_grid::SignedIndex& sindex);

  std::unique_ptr<AbstractLineIterator> internal_iterator_;
  double x0_, y0_, x1_, y1_;
  bool include_last_index_;
  bool bresenham_;
  int signed_width_, signed_height_;
  nav_grid::Index start_index_, end_index_;
};
}  // namespace nav_grid_iterators

#endif  // NAV_GRID_ITERATORS_LINE_H
