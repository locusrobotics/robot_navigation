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

#ifndef NAV_GRID_ITERATORS_SPIRAL_H
#define NAV_GRID_ITERATORS_SPIRAL_H

#include <nav_grid_iterators/base_iterator.h>
#include <nav_grid_iterators/circle_outline.h>
#include <memory>

namespace nav_grid_iterators
{
/**
 * @class Spiral
 * @brief Iterates over all of the valid indexes that lie within a circle from the center out
 */
class Spiral : public BaseIterator<Spiral>
{
public:
  /**
   * @brief Public Constructor.
   * @param info NavGridInfo for the grid to iterate over
   * @param center_x Center of the circle (x coordinate)
   * @param center_y Center of the circle (y coordinate)
   * @param radius Size of the circle
   */
  Spiral(const nav_grid::NavGridInfo* info, double center_x, double center_y, double radius);

  /**
   * @brief Copy Constructor
   * Required to ensure unique_ptr is set properly
   */
  Spiral(const Spiral& other);

  /**
   * @brief Assignment Operator
   * Required to ensure unique_ptr is set properly
   */
  Spiral& operator=(const Spiral& other);

  /**@name Standard BaseIterator Interface */
  /**@{*/
  Spiral begin() const override;
  Spiral end() const override;
  void increment() override;
  bool fieldsEqual(const Spiral& other) override;
  /**@}*/

protected:
  /**
   * @brief Protected constructor that takes in an arbitrary index and other internal parameters
   * @param info NavGridInfo for the grid to iterate over
   * @param index Initial index
   * @param center_x Center of the circle (x coordinate)
   * @param center_y Center of the circle (y coordinate)
   * @param radius_sq Square of the size of the circle
   * @param distance Which ring of the spiral to start on
   * @param max_distance The maximum valid ring
   * @param start_index The first valid index in the spiral
   */
  Spiral(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, double center_x, double center_y,
         double radius_sq, unsigned int distance, unsigned int max_distance,
         const nav_grid::Index& start_index);

  /**
   * @brief Given a new distance value, loads the internal iterator.
   * If there are no valid values in the internal iterator, increases the distance.
   */
  void loadRing();

  /**
   * @brief Check if the center of the given index is within the circle
   * @return true if inside
   */
  bool isInside(unsigned int x, unsigned int y) const;

  double center_x_, center_y_, radius_sq_;
  unsigned int distance_, max_distance_;
  nav_grid::Index start_index_;
  std::unique_ptr<CircleOutline> internal_iterator_;
};
}  // namespace nav_grid_iterators


#endif  // NAV_GRID_ITERATORS_SPIRAL_H
