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

#include <nav_core_adapter/global_planner_adapter2.h>
#include <nav_core_adapter/costmap_adapter.h>
#include <nav_2d_utils/conversions.h>
#include <nav_2d_utils/tf_help.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>
#include <memory>
#include <string>
#include <vector>

namespace nav_core_adapter
{

GlobalPlannerAdapter2::GlobalPlannerAdapter2() :
  planner_loader_("nav_core", "nav_core::BaseGlobalPlanner")
{
}

/**
 * @brief Load the nav_core global planner and initialize it
 */
void GlobalPlannerAdapter2::initialize(const ros::NodeHandle& parent, const std::string& name,
                                       TFListenerPtr tf, nav_core2::Costmap::Ptr costmap)
{
  costmap_ = costmap;
  std::shared_ptr<CostmapAdapter> ptr = std::static_pointer_cast<CostmapAdapter>(costmap);

  if (!ptr)
  {
    ROS_FATAL_NAMED("GlobalPlannerAdapter2",
                    "GlobalPlannerAdapter2 can only be used with the CostmapAdapter, not other Costmaps!");
    exit(EXIT_FAILURE);
  }
  costmap_ros_ = ptr->getCostmap2DROS();

  ros::NodeHandle planner_nh(parent, name);
  std::string planner_name;
  planner_nh.param("planner_name", planner_name, std::string("global_planner/GlobalPlanner"));
  ROS_INFO_NAMED("GlobalPlannerAdapter2", "Loading plugin %s", planner_name.c_str());
  planner_ = planner_loader_.createInstance(planner_name);
  planner_->initialize(planner_loader_.getName(planner_name), costmap_ros_);
}

nav_2d_msgs::Path2D GlobalPlannerAdapter2::makePlan(const nav_2d_msgs::Pose2DStamped& start,
                                                    const nav_2d_msgs::Pose2DStamped& goal)
{
  geometry_msgs::PoseStamped start3d = nav_2d_utils::pose2DToPoseStamped(start),
                             goal3d = nav_2d_utils::pose2DToPoseStamped(goal);
  std::vector<geometry_msgs::PoseStamped> plan;
  bool ret = planner_->makePlan(start3d, goal3d, plan);
  if (!ret)
  {
    throw nav_core2::PlannerException("Generic Global Planner Error");
  }
  return nav_2d_utils::posesToPath2D(plan);
}
}  // namespace nav_core_adapter


//  register this planner as a GlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(nav_core_adapter::GlobalPlannerAdapter2, nav_core2::GlobalPlanner)
