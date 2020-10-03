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

#ifndef NAV_GRID_PUB_SUB_OCC_GRID_MESSAGE_UTILS_H
#define NAV_GRID_PUB_SUB_OCC_GRID_MESSAGE_UTILS_H

#include <nav_2d_utils/conversions.h>
#include <nav_grid_pub_sub/cost_interpretation.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_core2/bounds.h>
#include <nav_grid_iterators/whole_grid.h>
#include <nav_grid_iterators/sub_grid.h>
#include <algorithm>
#include <limits>
#include <vector>


namespace nav_grid_pub_sub
{

/**
 * @brief NavGrid<unsigned char> to OccupancyGrid using cost_interpretation_table
 */
inline nav_msgs::OccupancyGrid toOccupancyGrid(const nav_grid::NavGrid<unsigned char>& grid,
    const ros::Time& stamp = ros::Time(0),
    const std::vector<unsigned char> cost_interpretation_table = std::vector<unsigned char>())
{
  nav_msgs::OccupancyGrid ogrid;
  const nav_grid::NavGridInfo& info = grid.getInfo();
  ogrid.header.frame_id = info.frame_id;
  ogrid.header.stamp = stamp;
  ogrid.info = nav_2d_utils::infoToInfo(info);
  ogrid.data.resize(info.width * info.height);

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(info))
  {
    ogrid.data[data_index++] = interpretCost(grid(index), cost_interpretation_table);
  }
  return ogrid;
}

/**
 * @brief NavGrid<unsigned char> to OccupancyGridUpdate using cost_interpretation_table
 */
inline map_msgs::OccupancyGridUpdate toOccupancyGridUpdate(const nav_grid::NavGrid<unsigned char>& grid,
    const nav_core2::UIntBounds& bounds, const ros::Time& stamp = ros::Time(0),
    const std::vector<unsigned char> cost_interpretation_table = std::vector<unsigned char>())
{
  map_msgs::OccupancyGridUpdate update;
  const nav_grid::NavGridInfo& info = grid.getInfo();
  update.header.stamp = stamp;
  update.header.frame_id = info.frame_id;
  update.x = bounds.getMinX();
  update.y = bounds.getMinY();
  update.width = bounds.getWidth();
  update.height = bounds.getHeight();
  update.data.resize(update.width * update.height);

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::SubGrid(&info, bounds))
  {
    update.data[data_index++] = interpretCost(grid(index), cost_interpretation_table);
  }
  return update;
}



/**
 * @brief generic NavGrid to OccupancyGrid using scaling. Min and max explicitly provided.
 *
 * NumericType can be inferred, so explicitly specifying it is generally not needed.
 */
template<typename NumericType>
nav_msgs::OccupancyGrid toOccupancyGrid(const nav_grid::NavGrid<NumericType>& grid,
                                        const NumericType min_value, const NumericType max_value,
                                        const NumericType unknown_value,
                                        const ros::Time& stamp = ros::Time(0))
{
  nav_msgs::OccupancyGrid ogrid;
  const nav_grid::NavGridInfo& info = grid.getInfo();
  ogrid.header.frame_id = info.frame_id;
  ogrid.header.stamp = stamp;
  ogrid.info = nav_2d_utils::infoToInfo(info);
  ogrid.data.resize(info.width * info.height);

  NumericType denominator = max_value - min_value;
  if (denominator == 0)
  {
    denominator = 1;
  }

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(info))
  {
    ogrid.data[data_index++] = interpretValue(grid(index), min_value, denominator, unknown_value);
  }
  return ogrid;
}

/**
 * @brief Retrieve the minimum and maximum values from a grid, ignoring the unknown_value
 */
template<typename NumericType>
inline void getExtremeValues(const nav_grid::NavGrid<NumericType>& grid, const NumericType unknown_value,
                             NumericType& min_val, NumericType& max_val)
{
  max_val = std::numeric_limits<NumericType>::min();
  min_val = std::numeric_limits<NumericType>::max();

  for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(grid.getInfo()))
  {
    const NumericType& value = grid(index);
    if (value == unknown_value) continue;
    max_val = std::max(value, max_val);
    min_val = std::min(value, min_val);
  }
}

/**
 * @brief generic NavGrid to OccupancyGrid using scaling. Min and max not provided, so they are determined first.
 *
 * NumericType can be inferred, so explicitly specifying it is generally not needed.
 */
template<typename NumericType>
nav_msgs::OccupancyGrid toOccupancyGrid(const nav_grid::NavGrid<NumericType>& grid,
                                        const NumericType unknown_value = std::numeric_limits<NumericType>::max(),
                                        const ros::Time& stamp = ros::Time(0))
{
  NumericType min_val, max_val;
  getExtremeValues(grid, unknown_value, min_val, max_val);
  return toOccupancyGrid(grid, min_val, max_val, unknown_value, stamp);
}

/**
 * @brief generic NavGrid to OccupancyGridUpdate using scaling. Min and max explicitly provided.
 *
 * NumericType can be inferred, so explicitly specifying it is generally not needed.
 */
template<typename NumericType>
map_msgs::OccupancyGridUpdate toOccupancyGridUpdate(const nav_grid::NavGrid<NumericType>& grid,
                                        const nav_core2::UIntBounds& bounds,
                                        const NumericType min_value, const NumericType max_value,
                                        const NumericType unknown_value,
                                        const ros::Time& stamp = ros::Time(0))
{
  map_msgs::OccupancyGridUpdate update;
  const nav_grid::NavGridInfo& info = grid.getInfo();
  update.header.stamp = stamp;
  update.header.frame_id = info.frame_id;
  update.x = bounds.getMinX();
  update.y = bounds.getMinY();
  update.width = bounds.getWidth();
  update.height = bounds.getHeight();
  update.data.resize(update.width * update.height);
  NumericType denominator = max_value - min_value;
  if (denominator == 0)
  {
    denominator = 1;
  }

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::SubGrid(&info, bounds))
  {
    update.data[data_index++] = interpretValue(grid(index), min_value, denominator, unknown_value);
  }
  return update;
}

/**
 * @brief generic OccupancyGrid to NavGrid conversion using cost_interpretation_table
 */
template<typename NumericType>
void fromOccupancyGrid(const nav_msgs::OccupancyGrid& msg, nav_grid::NavGrid<NumericType>& grid,
                       const std::vector<NumericType>& cost_interpretation_table)
{
  nav_grid::NavGridInfo info = nav_2d_utils::infoToInfo(msg.info, msg.header.frame_id);
  const nav_grid::NavGridInfo current_info = grid.getInfo();
  if (info != current_info)
  {
    grid.setInfo(info);
  }

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(info))
  {
    // Because OccupancyGrid.msg defines its data as `int8[] data` we need to explicitly parameterize it as an
    // unsigned char, otherwise it will be interpretted incorrectly.
    NumericType value =
      nav_grid_pub_sub::interpretCost<NumericType, unsigned char>(msg.data[data_index++], cost_interpretation_table);
    grid.setValue(index, value);
  }
}

/**
 * @brief generic OccupancyGridUpdate to NavGrid conversion using cost_interpretation_table.
 * @return A bounds object to know how much was updated
 */
template<typename NumericType>
nav_core2::UIntBounds fromOccupancyGridUpdate(const map_msgs::OccupancyGridUpdate& update,
                                              nav_grid::NavGrid<NumericType>& grid,
                                              const std::vector<NumericType>& cost_interpretation_table)
{
  const nav_grid::NavGridInfo& info = grid.getInfo();
  nav_core2::UIntBounds bounds(update.x, update.y, update.x + update.width - 1, update.y + update.height - 1);

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::SubGrid(&info, bounds))
  {
    // Because OccupancyGridUpdate.msg defines its data as `int8[] data` we need to explicitly parameterize it as an
    // unsigned char, otherwise it will be interpretted incorrectly.
    NumericType value =
      nav_grid_pub_sub::interpretCost<NumericType, unsigned char>(update.data[data_index++], cost_interpretation_table);
    grid.setValue(index, value);
  }
  return bounds;
}

}  // namespace nav_grid_pub_sub

#endif  // NAV_GRID_PUB_SUB_OCC_GRID_MESSAGE_UTILS_H
