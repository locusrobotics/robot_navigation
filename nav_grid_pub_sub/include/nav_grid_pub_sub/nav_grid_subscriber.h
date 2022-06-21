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

#ifndef NAV_GRID_PUB_SUB_NAV_GRID_SUBSCRIBER_H
#define NAV_GRID_PUB_SUB_NAV_GRID_SUBSCRIBER_H

#include <ros/ros.h>
#include <nav_grid/nav_grid.h>
#include <nav_core2/bounds.h>
#include <nav_2d_msgs/NavGridOfChars.h>
#include <nav_2d_msgs/NavGridOfCharsUpdate.h>
#include <nav_2d_msgs/NavGridOfDoubles.h>
#include <nav_2d_msgs/NavGridOfDoublesUpdate.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_2d_utils/bounds.h>
#include <nav_grid_pub_sub/nav_grid_message_utils.h>
#include <nav_grid_pub_sub/occ_grid_message_utils.h>
#include <string>
#include <vector>

namespace nav_grid_pub_sub
{
template<typename NumericType, typename NavGridOfX, typename NavGridOfXUpdate>
class GenericNavGridSubscriber
{
public:
  using NewDataCallback = std::function<void(const nav_core2::UIntBounds&)>;

  explicit GenericNavGridSubscriber(nav_grid::NavGrid<NumericType>& data) : data_(data) {}
  void init(ros::NodeHandle& nh, NewDataCallback callback, const std::string& topic = "map",
            bool nav_grid = true, bool subscribe_to_updates = true)
  {
    nh_ = nh;
    callback_ = callback;
    topic_ = topic;
    nav_grid_ = nav_grid;
    subscribe_to_updates_ = subscribe_to_updates;
    activate();
  }

  void activate()
  {
    std::string resolved_topic = nh_.resolveName(topic_);
    map_received_ = false;
    if (nav_grid_)
    {
      sub_ = nh_.subscribe(resolved_topic, 1, &GenericNavGridSubscriber::incomingNav, this);
      if (subscribe_to_updates_)
      {
        update_sub_ = nh_.subscribe(resolved_topic + "_updates", 10,
                                    &GenericNavGridSubscriber::incomingNavUpdate, this);
      }
    }
    else
    {
      sub_ = nh_.subscribe(resolved_topic, 1, &GenericNavGridSubscriber::incomingOcc, this);
      if (subscribe_to_updates_)
      {
        update_sub_ = nh_.subscribe(resolved_topic + "_updates", 10,
                                    &GenericNavGridSubscriber::incomingOccUpdate, this);
      }
    }
  }

  void deactivate()
  {
    sub_.shutdown();
    update_sub_.shutdown();
  }

  bool hasData() const { return map_received_; }

  void setCostInterpretation(const std::vector<NumericType>& cost_interpretation_table)
  {
    cost_interpretation_table_ = cost_interpretation_table;
  }
protected:
  void incomingNav(const NavGridOfX& new_map)
  {
    fromMsg(new_map, data_);
    map_received_ = true;
    callback_(nav_2d_utils::getFullUIntBounds(data_.getInfo()));
  }

  void incomingNavUpdate(const NavGridOfXUpdate& update)
  {
    if (!map_received_)
    {
      return;
    }
    nav_core2::UIntBounds bounds = fromUpdate(update, data_);
    callback_(bounds);
  }

  void incomingOcc(const nav_msgs::OccupancyGridConstPtr& new_map)
  {
    fromOccupancyGrid(*new_map, data_, cost_interpretation_table_);
    map_received_ = true;
    callback_(nav_2d_utils::getFullUIntBounds(data_.getInfo()));
  }

  void incomingOccUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
  {
    if (!map_received_)
    {
      return;
    }
    nav_core2::UIntBounds bounds = fromOccupancyGridUpdate(*update, data_, cost_interpretation_table_);
    callback_(bounds);
  }

  nav_grid::NavGrid<NumericType>& data_;
  NewDataCallback callback_;

  std::vector<NumericType> cost_interpretation_table_;

  ros::Subscriber sub_, update_sub_;
  bool map_received_;

  ros::NodeHandle nh_;
  std::string topic_;
  bool nav_grid_, subscribe_to_updates_;
};

using NavGridSubscriber =
  GenericNavGridSubscriber<unsigned char, nav_2d_msgs::NavGridOfChars, nav_2d_msgs::NavGridOfCharsUpdate>;

using NavGridOfDoublesSubscriber =
  GenericNavGridSubscriber<double, nav_2d_msgs::NavGridOfDoubles, nav_2d_msgs::NavGridOfDoublesUpdate>;

}  // namespace nav_grid_pub_sub

#endif  // NAV_GRID_PUB_SUB_NAV_GRID_SUBSCRIBER_H
