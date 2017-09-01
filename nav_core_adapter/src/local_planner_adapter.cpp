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

#include <nav_core_adapter/local_planner_adapter.h>
#include <nav_2d_utils/conversions.h>
#include <nav_2d_utils/tf_help.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>
#include <string>
#include <vector>

namespace nav_core_adapter
{

LocalPlannerAdapter::LocalPlannerAdapter() :
  planner_loader_("nav_core2", "nav_core2::LocalPlanner")
{
}

/**
 * @brief Load the nav_core2 local planner and initialize it
 */
void LocalPlannerAdapter::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  tf_ = TFListenerPtr(tf);
  costmap_ros_ = CostmapROSPtr(costmap_ros);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~/" + name);
  std::string planner_name;
  private_nh.param("planner_name", planner_name, std::string("dwb_local_planner::DWBLocalPlanner"));
  ROS_INFO_NAMED("LocalPlannerAdapter", "Loading plugin %s", planner_name.c_str());
  planner_ = planner_loader_.createInstance(planner_name);
  planner_->initialize(planner_loader_.getName(planner_name), tf_, costmap_ros_);

  odom_sub_ = std::make_shared<nav_2d_utils::OdomSubscriber>(nh);
}

/**
 * @brief Collect the additional information needed by nav_core2 and call the child computeVelocityCommands
 */
bool LocalPlannerAdapter::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  // Get the Pose
  nav_2d_msgs::Pose2DStamped pose2d;
  if (!getRobotPose(pose2d))
  {
    return false;
  }

  // Get the Velocity
  nav_2d_msgs::Twist2D velocity = odom_sub_->getTwist();

  nav_2d_msgs::Twist2DStamped cmd_vel_2d;
  try
  {
    cmd_vel_2d = planner_->computeVelocityCommands(pose2d, velocity);
  }
  catch (const nav_core2::PlannerException& e)
  {
    ROS_ERROR_NAMED("LocalPlannerAdapter", "computeVelocityCommands exception: %s", e.what());
    return false;
  }
  cmd_vel = nav_2d_utils::twist2Dto3D(cmd_vel_2d.velocity);
  return true;
}

/**
 * @brief Collect the additional information needed by nav_core2 and call the child isGoalReached
 */
bool LocalPlannerAdapter::isGoalReached()
{
  // Get the Pose
  nav_2d_msgs::Pose2DStamped pose2d;
  if (!getRobotPose(pose2d))
    return false;

  nav_2d_msgs::Twist2D velocity = odom_sub_->getTwist();
  return planner_->isGoalReached(pose2d, velocity);
}

/**
 * @brief Convert from 2d to 3d and call child setPlan
 */
bool LocalPlannerAdapter::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  nav_2d_msgs::Path2D path = nav_2d_utils::posesToPath2D(orig_global_plan);
  try
  {
    planner_->setPlan(path);
    return true;
  }
  catch (const nav_core2::PlannerException& e)
  {
    ROS_ERROR_NAMED("LocalPlannerAdapter", "setPlan Exception: %s", e.what());
    return false;
  }
}

bool LocalPlannerAdapter::getRobotPose(nav_2d_msgs::Pose2DStamped& pose2d)
{
  tf::Stamped<tf::Pose> current_pose;
  if (!costmap_ros_->getRobotPose(current_pose))
  {
    ROS_ERROR_NAMED("LocalPlannerAdapter", "Could not get robot pose");
    return false;
  }
  pose2d = nav_2d_utils::stampedPoseToPose2D(current_pose);
  return true;
}

}  // namespace nav_core_adapter

//  register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(nav_core_adapter::LocalPlannerAdapter, nav_core::BaseLocalPlanner)
