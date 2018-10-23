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

#include <nav_grid_pub_sub/nav_grid_subscriber.h>
#include <nav_2d_utils/conversions.h>
#include <nav_grid_pub_sub/cost_interpretation.h>
#include <nav_grid_iterators/whole_grid.h>
#include <nav_grid_iterators/sub_grid.h>
#include <string>
#include <vector>

namespace nav_grid_pub_sub
{
void NavGridSubscriber::init(ros::NodeHandle& nh, NewDataCallback callback, const std::string& topic,
                             bool nav_grid, bool subscribe_to_updates)
{
  nh_ = nh;
  callback_ = callback;
  topic_ = topic;
  nav_grid_ = nav_grid;
  subscribe_to_updates_ = subscribe_to_updates;
  activate();
}

void NavGridSubscriber::activate()
{
  std::string resolved_topic = nh_.resolveName(topic_);
  map_received_ = false;
  if (nav_grid_)
  {
    sub_ = nh_.subscribe(resolved_topic, 1, &NavGridSubscriber::incomingNav, this);
    if (subscribe_to_updates_)
    {
      update_sub_ = nh_.subscribe(resolved_topic + "_updates", 10, &NavGridSubscriber::incomingNavUpdate, this);
    }
  }
  else
  {
    sub_ = nh_.subscribe(resolved_topic, 1, &NavGridSubscriber::incomingOcc, this);
    if (subscribe_to_updates_)
    {
      update_sub_ = nh_.subscribe(resolved_topic + "_updates", 10, &NavGridSubscriber::incomingOccUpdate, this);
    }
  }
}

void NavGridSubscriber::deactivate()
{
  sub_.shutdown();
  update_sub_.shutdown();
}

void NavGridSubscriber::incomingNav(const nav_2d_msgs::NavGridOfCharsConstPtr& new_map)
{
  nav_grid::NavGridInfo info = nav_2d_utils::fromMsg(new_map->info);
  const nav_grid::NavGridInfo current_info = data_.getInfo();
  if (info != current_info)
  {
    data_.setInfo(info);
  }

  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(info))
  {
    data_.setValue(index, new_map->data[data_index++]);
  }
  map_received_ = true;
  callback_(nav_core2::UIntBounds(0, 0, info.width - 1, info.height - 1));
}

void NavGridSubscriber::incomingNavUpdate(const nav_2d_msgs::NavGridOfCharsUpdateConstPtr& update)
{
  const nav_grid::NavGridInfo& info = data_.getInfo();
  nav_core2::UIntBounds bounds = nav_2d_utils::fromMsg(update->bounds);
  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::SubGrid(&info, bounds))
  {
    data_.setValue(index, update->data[data_index++]);
  }
  callback_(bounds);
}

void NavGridSubscriber::incomingOcc(const nav_msgs::OccupancyGridConstPtr& new_map)
{
  nav_grid::NavGridInfo info = nav_2d_utils::infoToInfo(new_map->info);
  const nav_grid::NavGridInfo current_info = data_.getInfo();
  if (info != current_info)
  {
    data_.setInfo(info);
    data_.reset();
  }
  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::WholeGrid(info))
  {
    data_.setValue(index, interpretCost(new_map->data[data_index++], cost_interpretation_table_));
  }

  map_received_ = true;
  callback_(nav_core2::UIntBounds(0, 0, info.width - 1, info.height - 1));
}

void NavGridSubscriber::incomingOccUpdate(const map_msgs::OccupancyGridUpdateConstPtr& update)
{
  const nav_grid::NavGridInfo& info = data_.getInfo();
  nav_core2::UIntBounds bounds(update->x, update->y, update->x + update->width - 1, update->y + update->height - 1);
  unsigned int data_index = 0;
  for (const nav_grid::Index& index : nav_grid_iterators::SubGrid(&info, bounds))
  {
    data_.setValue(index, interpretCost(update->data[data_index++], cost_interpretation_table_));
  }
  callback_(bounds);
}
}  // namespace nav_grid_pub_sub
