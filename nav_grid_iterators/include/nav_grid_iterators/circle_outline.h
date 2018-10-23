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

#ifndef NAV_GRID_ITERATORS_CIRCLE_OUTLINE_H
#define NAV_GRID_ITERATORS_CIRCLE_OUTLINE_H

#include <nav_grid_iterators/base_iterator.h>

namespace nav_grid_iterators
{

/**
 * @brief returns the sign of a number
 * @param val number
 * @return 1 if positive, 0 if 0, -1 if negative
 */
inline int signum(const int val)
{
  return (0 < val) - (val < 0);
}

/**
 * @class CircleOutline
 * @brief Iterates over the valid indexes that lie on the outline of a circle
 */
class CircleOutline : public BaseIterator<CircleOutline>
{
public:
  /**
   * @brief Public Constructor.
   * @param info NavGridInfo for the grid to iterate over
   * @param center_x Center of the circle (x coordinate)
   * @param center_y Center of the circle (y coordinate)
   * @param radius Size of the circle
   */
  CircleOutline(const nav_grid::NavGridInfo* info, double center_x, double center_y, double radius);


  /**
   * @brief Public Constructor with integer radius.
   * @param info NavGridInfo for the grid to iterate over
   * @param center_x Center of the circle (x coordinate)
   * @param center_y Center of the circle (y coordinate)
   * @param radius Size of the circle
   */
  CircleOutline(const nav_grid::NavGridInfo* info, double center_x, double center_y, unsigned int radius);

  /**@name Standard BaseIterator Interface */
  /**@{*/
  CircleOutline begin() const override;
  CircleOutline end() const override;
  void increment() override;
  bool fieldsEqual(const CircleOutline& other) override;
  /**@}*/

protected:
  /**
   * @brief Protected constructor that takes in an arbitrary index and other internal parameters
   * @param info NavGridInfo for the grid to iterate over
   * @param index Initial index
   * @param center_index_x Index of the center of the circle (x coordinate)
   * @param center_index_y Index of the center of the circle (y coordinate)
   * @param distance The number of cells in the radius of the circle
   * @param init Whether the first cell has been visited or not
   * @param start_index The first valid index in the minimum row
   */
  CircleOutline(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, int center_index_x, int center_index_y,
                unsigned int distance, bool init, const nav_grid::Index& start_index);


  /**
   * @brief Check if arbitrary coordinates are within the grid
   * @return true if inside grid, false otherwise.
   */
  bool isValidIndex(int x, int y) const;

  /**
   * @brief Check if a cell with the given distance from the center of the circle is on the outline of the circle
   * @return true if the distance to the cell when rounded to an integer is equal to the distance_
   */
  bool isOnOutline(int dx, int dy) const;

  int center_index_x_, center_index_y_;
  unsigned int distance_;
  bool init_;
  int signed_width_, signed_height_;
  int point_x_, point_y_;
  nav_grid::Index start_index_;
};
}  // namespace nav_grid_iterators


#endif  // NAV_GRID_ITERATORS_CIRCLE_OUTLINE_H
