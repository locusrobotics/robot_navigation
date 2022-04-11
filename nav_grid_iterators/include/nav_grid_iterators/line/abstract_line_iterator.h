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

#ifndef NAV_GRID_ITERATORS_LINE_ABSTRACT_LINE_ITERATOR_H
#define NAV_GRID_ITERATORS_LINE_ABSTRACT_LINE_ITERATOR_H

#include <nav_grid/index.h>

namespace nav_grid_iterators
{
/**
 * @class AbstractLineIterator
 * @brief Abstract class for iterating over lines.
 *
 * Not constrained by a bounding box from NavGridInfo, i.e. can include positive and negative indexes
 */
class AbstractLineIterator
{
public:
  /**
   * @brief Public Constructor
   */
  AbstractLineIterator() {}

  /**
   * @brief Public Destructor
   */
  virtual ~AbstractLineIterator() = default;

  /**
   * @brief Dereference the iterator
   * @return the index to which the iterator is pointing.
   */
  const nav_grid::SignedIndex& operator*() const { return index_; }

  virtual nav_grid::SignedIndex getFinalIndex() const = 0;

  bool isFinished()
  {
    return getFinalIndex() == index_;
  }

  /**
   * @brief Increase the iterator to the next element.
   */
  virtual void increment() = 0;

protected:
  /**
   * @brief Protected Constructor - takes arbitrary index
   */
  explicit AbstractLineIterator(nav_grid::SignedIndex index) : index_(index) {}
  nav_grid::SignedIndex index_;
};

}  // namespace nav_grid_iterators

#endif  // NAV_GRID_ITERATORS_LINE_ABSTRACT_LINE_ITERATOR_H
