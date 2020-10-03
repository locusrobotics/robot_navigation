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

#ifndef NAV_GRID_PUB_SUB_NAV_GRID_MESSAGE_UTILS_H
#define NAV_GRID_PUB_SUB_NAV_GRID_MESSAGE_UTILS_H

#include <nav_2d_utils/conversions.h>
#include <nav_2d_msgs/NavGridOfChars.h>
#include <nav_2d_msgs/NavGridOfCharsUpdate.h>
#include <nav_2d_msgs/NavGridOfDoubles.h>
#include <nav_2d_msgs/NavGridOfDoublesUpdate.h>
#include <nav_grid_iterators/whole_grid.h>
#include <nav_grid_iterators/sub_grid.h>
#include <nav_core2/bounds.h>

namespace nav_grid_pub_sub
{
/**
 * @brief Utilities for converting NavGrid objects to NavGridOfX messages and updates
 */

/**
 * @brief Generic conversion from grid of arbitrary type to message of arbitrary type
 *
 * Note that the ROSMsgType must be provided since return types are not used in template type deduction.
 */
template<typename ROSMsgType, typename NumericType>
ROSMsgType toMsg(const nav_grid::NavGrid<NumericType>& grid, const ros::Time& stamp)
{
  const nav_grid::NavGridInfo& info = grid.getInfo();

  ROSMsgType msg;
  msg.stamp = stamp;
  msg.info = nav_2d_utils::toMsg(info);
  msg.data.resize(info.width * info.height);

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(info))
  {
    msg.data[data_index++] = grid(index);
  }
  return msg;
}

/**
 * @brief Generic conversion from a portion of a grid of arbitrary type to an update message of arbitrary type
 *
 * Note that the ROSMsgType must be provided since return types are not used in template type deduction.
 */
template<typename ROSMsgType, typename NumericType>
ROSMsgType toUpdate(const nav_grid::NavGrid<NumericType>& grid, const nav_core2::UIntBounds& bounds,
                    const ros::Time& stamp)
{
  ROSMsgType update;
  update.stamp = stamp;
  update.bounds = nav_2d_utils::toMsg(bounds);
  update.data.resize(bounds.getWidth() * bounds.getHeight());

  unsigned int data_index = 0;
  const nav_grid::NavGridInfo& info = grid.getInfo();
  for (const nav_grid::Index& index : nav_grid_iterators::SubGrid(&info, bounds))
  {
    update.data[data_index++] = grid(index);
  }
  return update;
}

/**
 * @brief Generic conversion from message of arbitrary type to grid of arbitrary type
 */
template<typename ROSMsgType, typename NumericType>
void fromMsg(const ROSMsgType& msg, nav_grid::NavGrid<NumericType>& grid)
{
  nav_grid::NavGridInfo info = nav_2d_utils::fromMsg(msg.info);
  const nav_grid::NavGridInfo current_info = grid.getInfo();
  if (info != current_info)
  {
    grid.setInfo(info);
  }

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(info))
  {
    grid.setValue(index, msg.data[data_index++]);
  }
}

/**
 * @brief Generic conversion from an update message to a portion of a grid of arbitrary type
 */
template<typename ROSMsgType, typename NumericType>
nav_core2::UIntBounds fromUpdate(const ROSMsgType& update, nav_grid::NavGrid<NumericType>& grid)
{
  const nav_grid::NavGridInfo& info = grid.getInfo();
  nav_core2::UIntBounds bounds = nav_2d_utils::fromMsg(update.bounds);

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::SubGrid(&info, bounds))
  {
    grid.setValue(index, update.data[data_index++]);
  }
  return bounds;
}

/**
 * @brief NavGrid<unsigned char> to NavGridOfChars
 */
inline nav_2d_msgs::NavGridOfChars toMsg(const nav_grid::NavGrid<unsigned char>& grid,
                                         const ros::Time& stamp = ros::Time(0))
{
  return toMsg<nav_2d_msgs::NavGridOfChars>(grid, stamp);
}

/**
 * @brief NavGrid<double> to NavGridOfDoubles
 */
inline nav_2d_msgs::NavGridOfDoubles toMsg(const nav_grid::NavGrid<double>& grid, const ros::Time& stamp = ros::Time(0))
{
  return toMsg<nav_2d_msgs::NavGridOfDoubles>(grid, stamp);
}

/**
 * @brief NavGrid<float> to NavGridOfDoubles
 */
inline nav_2d_msgs::NavGridOfDoubles toMsg(const nav_grid::NavGrid<float>& grid, const ros::Time& stamp = ros::Time(0))
{
  return toMsg<nav_2d_msgs::NavGridOfDoubles>(grid, stamp);
}

/**
 * @brief NavGrid<unsigned char> to NavGridOfCharsUpdate
 */
inline nav_2d_msgs::NavGridOfCharsUpdate toUpdate(const nav_grid::NavGrid<unsigned char>& grid,
        const nav_core2::UIntBounds& bounds, const ros::Time& stamp = ros::Time(0))
{
  return toUpdate<nav_2d_msgs::NavGridOfCharsUpdate>(grid, bounds, stamp);
}

/**
 * @brief NavGrid<double> to NavGridOfDoublesUpdate
 */
inline nav_2d_msgs::NavGridOfDoublesUpdate toUpdate(const nav_grid::NavGrid<double>& grid,
        const nav_core2::UIntBounds& bounds, const ros::Time& stamp = ros::Time(0))
{
  return toUpdate<nav_2d_msgs::NavGridOfDoublesUpdate>(grid, bounds, stamp);
}

/**
 * @brief NavGrid<float> to NavGridOfDoublesUpdate
 */
inline nav_2d_msgs::NavGridOfDoublesUpdate toUpdate(const nav_grid::NavGrid<float>& grid,
        const nav_core2::UIntBounds& bounds, const ros::Time& stamp = ros::Time(0))
{
  return toUpdate<nav_2d_msgs::NavGridOfDoublesUpdate>(grid, bounds, stamp);
}


}  // namespace nav_grid_pub_sub

#endif  // NAV_GRID_PUB_SUB_NAV_GRID_MESSAGE_UTILS_H
