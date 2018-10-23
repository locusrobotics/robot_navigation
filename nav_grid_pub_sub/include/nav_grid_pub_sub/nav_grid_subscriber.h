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
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <string>
#include <vector>

namespace nav_grid_pub_sub
{
class NavGridSubscriber
{
public:
  using NewDataCallback = std::function<void(const nav_core2::UIntBounds&)>;

  explicit NavGridSubscriber(nav_grid::NavGrid<unsigned char>& data) : data_(data) {}
  void init(ros::NodeHandle& nh, NewDataCallback callback, const std::string& topic = "map",
            bool nav_grid = true, bool subscribe_to_updates = true);
  void activate();
  void deactivate();
  bool hasData() const { return map_received_; }

  void setCostInterpretation(const std::vector<unsigned char>& cost_interpretation_table)
  {
    cost_interpretation_table_ = cost_interpretation_table;
  }
protected:
  void incomingNav(const nav_2d_msgs::NavGridOfCharsConstPtr& new_map);
  void incomingNavUpdate(const nav_2d_msgs::NavGridOfCharsUpdateConstPtr& update);

  void incomingOcc(const nav_msgs::OccupancyGridConstPtr& new_map);
  void incomingOccUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update);

  nav_grid::NavGrid<unsigned char>& data_;
  NewDataCallback callback_;

  std::vector<unsigned char> cost_interpretation_table_;

  ros::Subscriber sub_, update_sub_;
  bool map_received_;

  ros::NodeHandle nh_;
  std::string topic_;
  bool nav_grid_, subscribe_to_updates_;
};
}  // namespace nav_grid_pub_sub

#endif  // NAV_GRID_PUB_SUB_NAV_GRID_SUBSCRIBER_H
