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

#ifndef NAV_GRID_INDEX_H
#define NAV_GRID_INDEX_H

#include <string>

namespace nav_grid
{
/**
 * @class GenericIndex
 * @brief A simple pair of x/y coordinates
 */
template <typename NumericType>
struct GenericIndex
{
  NumericType x, y;
  explicit GenericIndex(const NumericType& x = 0, const NumericType& y = 0) : x(x), y(y) {}

  /**
   * @brief comparison operator that requires equal x and y
   */
  bool operator == (const GenericIndex& other) const
  {
    return x == other.x && y == other.y;
  }

  bool operator != (const GenericIndex& other) const
  {
    return !operator==(other);
  }

  /**
   * @brief less than operator so object can be used in sets
   */
  bool operator < (const GenericIndex& other) const
  {
    return x < other.x || (x == other.x && y < other.y);
  }

  // Derived Comparators
  bool operator > (const GenericIndex& other) const { return other < *this; }
  bool operator <= (const GenericIndex& other) const { return !(*this > other); }
  bool operator >= (const GenericIndex& other) const { return !(*this < other); }

  /**
   * @brief String representation of this object
   */
  std::string toString() const
  {
    return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
  }
};

template <typename NumericType>
inline std::ostream& operator<<(std::ostream& stream, const GenericIndex<NumericType>& index)
{
  stream << index.toString();
  return stream;
}

using SignedIndex = GenericIndex<int>;
using Index = GenericIndex<unsigned int>;

}  // namespace nav_grid

#endif  // NAV_GRID_INDEX_H
