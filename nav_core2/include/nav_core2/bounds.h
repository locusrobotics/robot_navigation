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

#ifndef NAV_CORE2_BOUNDS_H
#define NAV_CORE2_BOUNDS_H

#include <algorithm>
#include <limits>
#include <string>

namespace nav_core2
{
/**
 * @brief Templatized method for checking if a value falls inside a one-dimensional range
 * @param value     The value to check
 * @param min_value The minimum value of the range
 * @param max_value The maximum value of the range
 * @return True if the value is within the range
 */
template <typename NumericType>
inline bool inRange(const NumericType value, const NumericType min_value, const NumericType max_value)
{
  return min_value <= value && value <= max_value;
}

/**
 * @class GenericBounds
 * @brief Templatized class that represents a two dimensional bounds with ranges [min_x, max_x] [min_y, max_y] inclusive
 */
template <typename NumericType>
struct GenericBounds
{
public:
  /**
   * @brief Constructor for an empty bounds
   */
  GenericBounds()
  {
    reset();
  }

  /**
   * @brief Constructor for a non-empty initial bounds
   * @param x0 Initial min x
   * @param y0 Initial min y
   * @param x1 Initial max x
   * @param y1 Initial max y
   */
  GenericBounds(NumericType x0, NumericType y0, NumericType x1, NumericType y1)
    : min_x_(x0), min_y_(y0), max_x_(x1), max_y_(y1) {}

  /**
   * @brief Reset the bounds to be empty
   */
  void reset()
  {
    min_x_ = min_y_ = std::numeric_limits<NumericType>::max();
    max_x_ = max_y_ = std::numeric_limits<NumericType>::lowest();  // -max
  }

  /**
   * @brief Update the bounds to include the point (x, y)
   */
  void touch(NumericType x, NumericType y)
  {
    min_x_ = std::min(x, min_x_);
    min_y_ = std::min(y, min_y_);
    max_x_ = std::max(x, max_x_);
    max_y_ = std::max(y, max_y_);
  }

  /**
   * @brief Update the bounds to include points (x0, y0) and (x1, y1)
   * @param x0 smaller of two x values
   * @param y0 smaller of two y values
   * @param x1 larger of two x values
   * @param y1 larger of two y values
   */
  void update(NumericType x0, NumericType y0, NumericType x1, NumericType y1)
  {
    min_x_ = std::min(x0, min_x_);
    min_y_ = std::min(y0, min_y_);
    max_x_ = std::max(x1, max_x_);
    max_y_ = std::max(y1, max_y_);
  }

  /**
   * @brief Update the bounds to include the entirety of another bounds object
   * @param other Another bounds object
   */
  void merge(const GenericBounds<NumericType>& other)
  {
    update(other.min_x_, other.min_y_, other.max_x_, other.max_y_);
  }

  /**
   * @brief Returns true if the range is empty
   */
  bool isEmpty() const
  {
    return min_x_ > max_x_ && min_y_ > max_y_;
  }

  /**
   * @brief Returns true if the point is inside this range
   */
  bool contains(NumericType x, NumericType y) const
  {
    return inRange(x, min_x_, max_x_) && inRange(y, min_y_, max_y_);
  }

  /**
   * @brief returns true if the two bounds overlap each other
   */
  bool overlaps(const GenericBounds<NumericType>& other) const
  {
    return !isEmpty() && !other.isEmpty()
             && max_y_ >= other.min_y_ && min_y_ <= other.max_y_
             && max_x_ >= other.min_x_ && min_x_ <= other.max_x_;
  }

  /**
   * @brief comparison operator that requires all fields are equal
   */
  bool operator==(const GenericBounds<NumericType>& other) const
  {
    return min_x_ == other.min_x_ && min_y_ == other.min_y_ &&
           max_x_ == other.max_x_ && max_y_ == other.max_y_;
  }

  bool operator!=(const GenericBounds<NumericType>& other) const
  {
    return !operator==(other);
  }

  /**
   * @brief Returns a string representation of the bounds
   */
  std::string toString() const
  {
    if (!isEmpty())
    {
      return "(" + std::to_string(min_x_) + "," + std::to_string(min_y_) + "):(" +
             std::to_string(max_x_) + "," + std::to_string(max_y_) + ")";
    }
    else
    {
      return "(empty bounds)";
    }
  }

  NumericType getMinX() const { return min_x_; }
  NumericType getMinY() const { return min_y_; }
  NumericType getMaxX() const { return max_x_; }
  NumericType getMaxY() const { return max_y_; }

protected:
  NumericType min_x_, min_y_, max_x_, max_y_;
};

using Bounds = GenericBounds<double>;

inline unsigned int getDimension(unsigned int min_v, unsigned int max_v)
{
  return (min_v > max_v) ? 0 : max_v - min_v + 1;
}

class UIntBounds : public GenericBounds<unsigned int>
{
public:
  using GenericBounds<unsigned int>::GenericBounds;
  unsigned int getWidth() const { return getDimension(min_x_, max_x_); }
  unsigned int getHeight() const { return getDimension(min_y_, max_y_); }
};

}  // namespace nav_core2

#endif  // NAV_CORE2_BOUNDS_H
