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

#ifndef NAV_GRID_ITERATORS_BASE_ITERATOR_H
#define NAV_GRID_ITERATORS_BASE_ITERATOR_H

#include <nav_grid/nav_grid_info.h>
#include <nav_grid/index.h>

namespace nav_grid_iterators
{
template<class Derived>
class BaseIterator
{
public:
  /**
   * @brief Public Constructor. Takes in a pointer to the info and starts at the minimum index
   * @param info NavGridInfo for the grid to iterate over
   */
  explicit BaseIterator(const nav_grid::NavGridInfo* info) : BaseIterator(info, nav_grid::Index(0, 0)) {}

  /**
   * @brief Public Constructor. Takes in a reference to the info and starts at the minimum index
   * @param info NavGridInfo for the grid to iterate over
   */
  explicit BaseIterator(const nav_grid::NavGridInfo& info) : BaseIterator(&info, nav_grid::Index(0, 0)) {}

  /**
   * @brief Destructor
   */
  virtual ~BaseIterator() = default;

  /**
   * @brief Helper function for range-style iteration
   * Equivalent to the above constructor
   * @return Iterator representing beginning of the iteration
   */
  virtual Derived begin() const = 0;

  /**
   * @brief Helper function for range-style iteration
   * @return Iterator representing end of the iteration, with an invalid index
   */
  virtual Derived end() const  = 0;

  /**
   * @brief Test if two iterators are equivalent
   *
   * Derived classes may want to implement the fieldsEqual function
   * for checking if additional fields beyond the index and info are equal.
   */
  bool operator==(const Derived& other) { return info_ == other.info_ && index_ == other.index_ && fieldsEqual(other); }

  /**
   * @brief Test if two iterators are not equivalent - required for testing if iterator is at the end
   */
  bool operator!=(const Derived& other) { return !(*this == other); }

  /**
   * @brief Additional check for whether fields of derived iterators are equal.
   *
   * Helps make overriding the == operator easy.
   */
  virtual bool fieldsEqual(const Derived& other) { return true; }

  /**
   * @brief Dereference the iterator
   * @return the index to which the iterator is pointing.
   */
  const nav_grid::Index& operator*() const { return index_; }

  /**
   * @brief Increase the iterator to the next element.
   * @return a reference to the updated iterator.
   */
  Derived& operator++()
  {
    increment();
    return *static_cast<Derived*>(this);
  }

  /**
   * @brief Increase the iterator to the next element.
   */
  virtual void increment() = 0;

  using self_type = Derived;
  using value_type = nav_grid::Index;
  using reference = nav_grid::Index&;
  using pointer = nav_grid::Index*;
  using iterator_category = std::input_iterator_tag;
  using difference_type = int;


protected:
  /**
   * @brief Protected constructor that takes in an arbitrary index
   * @param info NavGridInfo for the grid to iterate over
   * @param index Initial index
   */
  BaseIterator(const nav_grid::NavGridInfo* info, const nav_grid::Index& index) : info_(info), index_(index) {}

  const nav_grid::NavGridInfo* info_;
  nav_grid::Index index_;
};
}  // namespace nav_grid_iterators

#endif  // NAV_GRID_ITERATORS_BASE_ITERATOR_H
