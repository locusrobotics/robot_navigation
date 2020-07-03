/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
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

#ifndef NAV_CORE_ADAPTER_GLOBAL_PLANNER_ADAPTER_H
#define NAV_CORE_ADAPTER_GLOBAL_PLANNER_ADAPTER_H

#include <nav_core/base_global_planner.h>
#include <nav_core2/global_planner.h>
#include <nav_core_adapter/costmap_adapter.h>
#include <pluginlib/class_loader.h>
#include <memory>
#include <string>
#include <vector>

namespace nav_core_adapter
{

/**
 * @class GlobalPlannerAdapter
 * @brief used for employing a `nav_core2` global planner interface as a `nav_core` plugin, like in `move_base`.
 */
class GlobalPlannerAdapter: public nav_core::BaseGlobalPlanner
{
public:
  GlobalPlannerAdapter();

  // Standard ROS Global Planner Interface
  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) override;

protected:
  // Plugin handling
  pluginlib::ClassLoader<nav_core2::GlobalPlanner> planner_loader_;
  boost::shared_ptr<nav_core2::GlobalPlanner> planner_;
  ros::Publisher path_pub_;

  TFListenerPtr tf_;

  std::shared_ptr<CostmapAdapter> costmap_adapter_;
  costmap_2d::Costmap2DROS* costmap_ros_;
};

}  // namespace nav_core_adapter

#endif  // NAV_CORE_ADAPTER_GLOBAL_PLANNER_ADAPTER_H
