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

#ifndef NAV_GRID_ITERATORS_LINE_RAY_TRACE_H
#define NAV_GRID_ITERATORS_LINE_RAY_TRACE_H

#include <nav_grid_iterators/line/abstract_line_iterator.h>

namespace nav_grid_iterators
{
/**
 * @class RayTrace
 * @brief Line Iterator with Ray Tracing (subpixel accuracy)
 */
class RayTrace : public AbstractLineIterator
{
public:
  /**
   * @brief Public constructor
   * @param x0 Start x coordinate
   * @param y0 Start y coordinate
   * @param x1 End x coordinate
   * @param y1 End y coordinate
   * @param include_last_index If true, will include the end coordinates.
   */
  RayTrace(double x0, double y0, double x1, double y1, bool include_last_index = true);

  /**
   * @brief Test if two iterators are equivalent
   */
  bool operator==(const RayTrace& other)
  {
    return x0_ == other.x0_ && y0_ == other.y0_ && x1_ == other.x1_ && y1_ == other.y1_ && index_ == other.index_;
  }

  /**
   * @brief Test if two iterators are not equivalent - required for testing if iterator is at the end
   */
  bool operator!=(const RayTrace& other) { return !(*this == other); }

  /**
   * @brief Helper function for range-style iteration
   * @return Iterator representing beginning of the iteration
   */
  RayTrace begin() const;

  /**
   * @brief Helper function for range-style iteration
   * @return Iterator representing end of the iteration, with an invalid index
   */
  RayTrace end() const;

  nav_grid::SignedIndex getFinalIndex() const override;
  void increment() override;

  /**
   * @brief Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  RayTrace& operator++()
  {
    increment();
    return *this;
  }

  using self_type = RayTrace;
  using value_type = nav_grid::SignedIndex;
  using reference = nav_grid::SignedIndex&;
  using pointer = nav_grid::SignedIndex*;
  using iterator_category = std::input_iterator_tag;
  using difference_type = int;

protected:
  /**
   * @brief Protected constructor that takes in an arbitrary index and other internal parameters
   * @param index Initial index
   * @param x0 Start x coordinate
   * @param y0 Start y coordinate
   * @param x1 End x coordinate
   * @param y1 End y coordinate
   * @param include_last_index If true, will include the end coordinates.
   * @param dx
   * @param dy
   * @param initial_error
   * @param loop_inc_x
   * @param loop_inc_y
   */
  RayTrace(const nav_grid::SignedIndex& index,
       double x0, double y0, double x1, double y1, bool include_last_index,
       double dx, double dy, double initial_error, int loop_inc_x, int loop_inc_y);


  double x0_, y0_, x1_, y1_;
  bool include_last_index_;
  double dx_, dy_, error_, initial_error_;
  int loop_inc_x_, loop_inc_y_;
};

}  // namespace nav_grid_iterators

#endif  // NAV_GRID_ITERATORS_LINE_RAY_TRACE_H
