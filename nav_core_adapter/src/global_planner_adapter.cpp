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

#include <nav_core_adapter/global_planner_adapter.h>
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

GlobalPlannerAdapter::GlobalPlannerAdapter() :
  planner_loader_("nav_core2", "nav_core2::GlobalPlanner")
{
}

/**
 * @brief Load the nav_core2 global planner and initialize it
 */
void GlobalPlannerAdapter::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  costmap_ros_ = costmap_ros;
  costmap_adapter_ = std::make_shared<CostmapAdapter>();
  costmap_adapter_->initialize(costmap_ros);

  ros::NodeHandle private_nh("~");
  ros::NodeHandle adapter_nh("~/" + name);
  std::string planner_name;
  adapter_nh.param("planner_name", planner_name, std::string("dlux_global_planner::DluxGlobalPlanner"));
  ROS_INFO_NAMED("GlobalPlannerAdapter", "Loading plugin %s", planner_name.c_str());
  planner_ = planner_loader_.createInstance(planner_name);
  planner_->initialize(private_nh, planner_loader_.getName(planner_name), tf_, costmap_adapter_);
  path_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
}

bool GlobalPlannerAdapter::makePlan(const geometry_msgs::PoseStamped& start,
                                    const geometry_msgs::PoseStamped& goal,
                                    std::vector<geometry_msgs::PoseStamped>& plan)
{
  nav_2d_msgs::Pose2DStamped start2d = nav_2d_utils::poseStampedToPose2D(start),
                             goal2d = nav_2d_utils::poseStampedToPose2D(goal);
  try
  {
    nav_2d_msgs::Path2D path2d = planner_->makePlan(start2d, goal2d);
    nav_msgs::Path path = nav_2d_utils::pathToPath(path2d);
    plan = path.poses;
    path_pub_.publish(path);
    return true;
  }
  catch (nav_core2::PlannerException& e)
  {
    ROS_ERROR_NAMED("GlobalPlannerAdapter", "makePlan Exception: %s", e.what());
    return false;
  }
}
}  // namespace nav_core_adapter


//  register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(nav_core_adapter::GlobalPlannerAdapter, nav_core::BaseGlobalPlanner)
