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

#ifndef NAV_GRID_VECTOR_NAV_GRID_H
#define NAV_GRID_VECTOR_NAV_GRID_H

#include <nav_grid/nav_grid.h>
#include <nav_grid/coordinate_conversion.h>
#include <algorithm>
#include <vector>

namespace nav_grid
{
/**
 * @class VectorNavGrid
 * A straight-forward implementation of the NavGrid class where the data for cell (x, y) is stored in a std::vector
 * with index (y * info.width + x).
 */
template <typename T> class VectorNavGrid : public NavGrid<T>
{
public:
  using NavGrid<T>::NavGrid;

  /**
   * @brief Reset the contents of the grid to the default value
   */
  void reset() override
  {
    data_.assign(this->info_.width * this->info_.height, this->default_value_);
  }

  /**
   * @brief Change the info while attempting to keep the values associated with the grid coordinates
   *
   * If the width changes, we need to move each row to its new location
   *
   * If just the height changes, then we can resize the vector without having to move elements
   *
   * We just overwrite the rest of the grid info
   */
  void setInfo(const NavGridInfo& new_info) override
  {
    if (this->info_.width != new_info.width)
    {
      std::vector<T> new_vector(new_info.width * new_info.height, this->default_value_);
      unsigned int cols_to_move = std::min(this->info_.width, new_info.width);
      auto old_it = data_.begin();
      auto new_it = new_vector.begin();
      unsigned int max_row = std::min(this->info_.height, new_info.height);
      for (unsigned int row = 0; row < max_row; row++)
      {
        std::copy(old_it, old_it + cols_to_move, new_it);
        old_it += this->info_.width;
        new_it += new_info.width;
      }
      data_.swap(new_vector);
    }
    else if (this->info_.height != new_info.height)
    {
      data_.resize(new_info.width * new_info.height, this->default_value_);
    }

    this->info_ = new_info;
  }

  /**
   * @brief Update the info while keeping the data geometrically in tact
   *
   * If the resolution or frame_id changes, reset all the data.
   *
   * Otherwise, adjust the new_info so the grid stays aligned (The grid's new info will be within a
   * resolution-length of the original new_info). Then copy the common values into the new grid.
   *
   * @param[in] new_info New information to update the grid with
   */
  void updateInfo(const NavGridInfo& new_info) override
  {
    // If the info is the same, make no changes
    if (this->info_ == new_info)
    {
      return;
    }

    // If the resolution or frame changes, reset the whole grid
    if (this->info_.resolution != new_info.resolution || this->info_.frame_id != new_info.frame_id)
    {
      setInfo(new_info);
      return;
    }

    // project the new origin into the grid
    int cell_ox, cell_oy;
    worldToGrid(this->info_, new_info.origin_x, new_info.origin_y, cell_ox, cell_oy);

    // To save casting from unsigned int to int a bunch of times
    int old_size_x = static_cast<int>(this->info_.width);
    int old_size_y = static_cast<int>(this->info_.height);

    // we need to compute the overlap of the new and existing windows
    int lower_left_x = std::min(std::max(cell_ox, 0), old_size_x);
    int lower_left_y = std::min(std::max(cell_oy, 0), old_size_y);
    int upper_right_x = std::min(std::max(cell_ox + static_cast<int>(new_info.width), 0), old_size_x);
    int upper_right_y = std::min(std::max(cell_oy + static_cast<int>(new_info.height), 0), old_size_y);

    unsigned int cell_size_x = upper_right_x - lower_left_x;
    unsigned int cell_size_y = upper_right_y - lower_left_y;

    // we need a vector to store the new contents in the window temporarily
    std::vector<T> new_data(new_info.width * new_info.height, this->default_value_);

    // compute the starting cell location for copying data back in
    int start_x = lower_left_x - cell_ox;
    int start_y = lower_left_y - cell_oy;

    // now we want to copy the overlapping information into the new vector, but in its new location
    // we'll first need to compute the starting points for each vector
    auto src_index = data_.begin() + (lower_left_y * old_size_x + lower_left_x);
    auto dest_index = new_data.begin() + (start_y * new_info.width + start_x);

    // now, we'll copy the source vector into the destination vector
    for (unsigned int i = 0; i < cell_size_y; ++i)
    {
      std::copy(src_index, src_index + cell_size_x, dest_index);
      src_index += this->info_.width;
      dest_index += new_info.width;
    }

    data_.swap(new_data);

    // update the dimensions
    this->info_.width = new_info.width;
    this->info_.height = new_info.height;

    // update the origin. Recomputed instead of using new_info.origin
    // because we want to keep things grid-aligned
    this->info_.origin_x += cell_ox * this->info_.resolution;
    this->info_.origin_y += cell_oy * this->info_.resolution;
  }

  void setValue(const unsigned int x, const unsigned int y, const T& value) override
  {
    data_[getIndex(x, y)] = value;
  }

  T getValue(const unsigned int x, const unsigned int y) const override
  {
    return data_[getIndex(x, y)];
  }

  using NavGrid<T>::operator();
  using NavGrid<T>::getValue;
  using NavGrid<T>::setValue;

  /**
   * Overloading the [] operator so that the data can be accessed directly with vector_nav_grid[i]
   */
  T  operator[] (unsigned int i) const    {return data_[i];}
  T& operator[] (unsigned int i) {return data_[i];}

  /**
   * @brief Return the size of the vector. Equivalent to width * height.
   * @return size of the vector
   */
  unsigned int size() const { return data_.size(); }

  /**
   * @brief  Given two grid coordinates... compute the associated index
   * @param[in] mx The x coordinate
   * @param[in] my The y coordinate
   * @return    The associated index
   */
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return my * this->info_.width + mx;
  }

  /**
   * @brief  Given two world coordinates... compute the associated index
   * @param[in] mx The x coordinate
   * @param[in] my The y coordinate
   * @return    The associated index
   */
  inline unsigned int getIndex(double x, double y) const
  {
    unsigned int mx, my;
    worldToGridBounded(this->info_, x, y, mx, my);
    return getIndex(mx, my);
  }

  /**
   * @brief  Given an index... compute the associated grid coordinates
   * @param[in]  index The index
   * @param[out] mx Set to the associated x grid coordinate
   * @param[out] my Set to the associated y grid coordinate
   */
  inline void indexToCells(unsigned int index, unsigned int& mx, unsigned int& my) const
  {
    unsigned int w = this->info_.width;
    my = index / w;
    mx = index - my * w;
  }

protected:
  std::vector<T> data_;
};
}  // namespace nav_grid

#endif  // NAV_GRID_VECTOR_NAV_GRID_H
