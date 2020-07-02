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
#include <nav_core_adapter/costmap_adapter.h>
#include <nav_core_adapter/shared_pointers.h>
#include <nav_2d_utils/conversions.h>
#include <nav_2d_utils/tf_help.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>
#include <memory>
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
void LocalPlannerAdapter::initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros)
{
  tf_ = createSharedPointerWithNoDelete<tf2_ros::Buffer>(tf);
  costmap_ros_ = costmap_ros;
  costmap_adapter_ = std::make_shared<CostmapAdapter>();
  costmap_adapter_->initialize(costmap_ros);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::NodeHandle adapter_nh("~/" + name);
  std::string planner_name;
  adapter_nh.param("planner_name", planner_name, std::string("dwb_local_planner::DWBLocalPlanner"));
  ROS_INFO_NAMED("LocalPlannerAdapter", "Loading plugin %s", planner_name.c_str());
  planner_ = planner_loader_.createInstance(planner_name);
  planner_->initialize(private_nh, planner_loader_.getName(planner_name), tf_, costmap_adapter_);
  has_active_goal_ = false;

  odom_sub_ = std::make_shared<nav_2d_utils::OdomSubscriber>(nh);
}

/**
 * @brief Collect the additional information needed by nav_core2 and call the child computeVelocityCommands
 */
bool LocalPlannerAdapter::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  if (!has_active_goal_)
  {
    return false;
  }

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
  bool ret = planner_->isGoalReached(pose2d, velocity);
  if (ret)
  {
    has_active_goal_ = false;
  }
  return ret;
}

/**
 * @brief Convert from 2d to 3d and call child setPlan
 */
bool LocalPlannerAdapter::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  nav_2d_msgs::Path2D path = nav_2d_utils::posesToPath2D(orig_global_plan);
  try
  {
    if (path.poses.size() > 0)
    {
      nav_2d_msgs::Pose2DStamped goal_pose;
      goal_pose.header = path.header;
      goal_pose.pose = path.poses.back();

      if (!has_active_goal_ || hasGoalChanged(goal_pose))
      {
        last_goal_ = goal_pose;
        has_active_goal_ = true;
        planner_->setGoalPose(goal_pose);
      }
    }

    planner_->setPlan(path);
    return true;
  }
  catch (const nav_core2::PlannerException& e)
  {
    ROS_ERROR_NAMED("LocalPlannerAdapter", "setPlan Exception: %s", e.what());
    return false;
  }
}

bool LocalPlannerAdapter::hasGoalChanged(const nav_2d_msgs::Pose2DStamped& new_goal)
{
  if (last_goal_.header.frame_id != new_goal.header.frame_id)
  {
    return true;
  }

  return last_goal_.pose.x != new_goal.pose.x || last_goal_.pose.y != new_goal.pose.y ||
         last_goal_.pose.theta != new_goal.pose.theta;
}

bool LocalPlannerAdapter::getRobotPose(nav_2d_msgs::Pose2DStamped& pose2d)
{
  geometry_msgs::PoseStamped current_pose;
  if (!costmap_ros_->getRobotPose(current_pose))
  {
    ROS_ERROR_NAMED("LocalPlannerAdapter", "Could not get robot pose");
    return false;
  }
  pose2d = nav_2d_utils::poseStampedToPose2D(current_pose);
  return true;
}

}  // namespace nav_core_adapter

//  register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(nav_core_adapter::LocalPlannerAdapter, nav_core::BaseLocalPlanner)
