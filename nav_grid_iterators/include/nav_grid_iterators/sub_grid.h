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

#ifndef NAV_GRID_ITERATORS_SUB_GRID_H
#define NAV_GRID_ITERATORS_SUB_GRID_H

#include <nav_grid_iterators/base_iterator.h>
#include <nav_core2/bounds.h>

namespace nav_grid_iterators
{
/**
 * @class SubGrid
 * @brief Iterator for looping through every index within an aligned rectangular portion of the grid
 */
class SubGrid : public BaseIterator<SubGrid>
{
public:
  /**
   * @brief Public Constructor
   * @param info NavGridInfo for the grid to iterate over
   * @param min_x Minimum index (x coordinate)
   * @param min_y Minimum index (y coordinate)
   * @param width Number of indexes in the x direction
   * @param height Number of indexes in the y direction
   */
  SubGrid(const nav_grid::NavGridInfo* info, unsigned int min_x, unsigned int min_y,
          unsigned int width, unsigned int height)
    : SubGrid(info, nav_grid::Index(min_x, min_y), min_x, min_y, width, height) {}

  /**
   * @brief Public Constructor using UIntBounds object
   * @param info NavGridInfo for the grid to iterate over
   * @param bounds UIntBounds
   */
  SubGrid(const nav_grid::NavGridInfo* info, const nav_core2::UIntBounds& bounds)
    : SubGrid(info, bounds.getMinX(), bounds.getMinY(), bounds.getWidth(), bounds.getHeight()) {}

  /**
   * @brief Public constructor that takes in an arbitrary index
   * @param info NavGridInfo for the grid to iterate over
   * @param index Initial index
   * @param min_x Minimum index (x coordinate)
   * @param min_y Minimum index (y coordinate)
   * @param width Number of indexes in the x direction
   * @param height Number of indexes in the y direction
   */
  SubGrid(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, unsigned int min_x, unsigned int min_y,
          unsigned int width, unsigned int height);

  /**
   * @brief Public constructor using UIntBounds object that takes in an arbitrary index
   * @param info NavGridInfo for the grid to iterate over
   * @param index Initial index
   * @param bounds UIntBounds
   */
  SubGrid(const nav_grid::NavGridInfo* info, const nav_grid::Index& index, const nav_core2::UIntBounds& bounds)
    : SubGrid(info, index, bounds.getMinX(), bounds.getMinY(), bounds.getWidth(), bounds.getHeight()) {}

  /**@name Standard BaseIterator Interface */
  /**@{*/
  SubGrid begin() const override;
  SubGrid end() const override;
  void increment() override;
  bool fieldsEqual(const SubGrid& other) override;
  /**@}*/

protected:
  unsigned int min_x_, min_y_, width_, height_;
};
}  // namespace nav_grid_iterators

#endif  // NAV_GRID_ITERATORS_SUB_GRID_H
