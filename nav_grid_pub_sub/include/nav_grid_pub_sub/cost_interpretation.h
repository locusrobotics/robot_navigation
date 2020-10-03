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

#ifndef NAV_GRID_PUB_SUB_COST_INTERPRETATION_H
#define NAV_GRID_PUB_SUB_COST_INTERPRETATION_H

#include <nav_grid/nav_grid.h>
#include <nav_grid_iterators/whole_grid.h>
#include <vector>

namespace nav_grid_pub_sub
{
/**
 * @brief return cost_interpretation_table[original_value] (or original_value if not a valid index)
 *
 * Since original_value is used as the index into the table, it must be an integer-like type (i.e. not double)
 */
template<typename NumericType, typename IntegralType>
inline NumericType interpretCost(IntegralType original_value,
                                 const std::vector<NumericType>& cost_interpretation_table)
{
  unsigned int index = static_cast<unsigned int>(original_value);
  if (index < cost_interpretation_table.size())
  {
    return cost_interpretation_table[index];
  }
  else
  {
    return original_value;
  }
}

/**
 * @brief Apply a given interpretation to the provided nav grid
 */
template<typename IntegralType>
inline void applyInterpretation(nav_grid::NavGrid<IntegralType>& grid,
                                const std::vector<IntegralType>& cost_interpretation_table)
{
  for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(grid.getInfo()))
  {
    grid.setValue(index, interpretCost(grid(index), cost_interpretation_table));
  }
}

/**
 * @brief Scale the given value to fit within [0, 100] (unless its ignore_value, then its -1)
 *
 * Note denominator is used instead of max_value to avoid recalculating the difference each time this is called.
 *
 * @param value Value to interpret
 * @param min_value Minimum value that will correspond to 0
 * @param denominator The result of max_value - min_value, where max_value corresponds with 100
 * @param unknown_value If the value is equal to this value, return -1
 */
template<typename NumericType>
inline unsigned char interpretValue(const NumericType value, const NumericType min_value,
                                    const NumericType denominator, const NumericType unknown_value)
{
  if (value == unknown_value)
  {
    return -1;
  }
  else
  {
    double ratio = (value - min_value) / denominator;
    return static_cast<unsigned char>(ratio * 100.0);
  }
}

}  // namespace nav_grid_pub_sub

#endif  // NAV_GRID_PUB_SUB_COST_INTERPRETATION_H
