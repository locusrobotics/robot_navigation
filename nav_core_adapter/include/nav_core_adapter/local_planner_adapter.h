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

#ifndef NAV_CORE_ADAPTER_LOCAL_PLANNER_ADAPTER_H
#define NAV_CORE_ADAPTER_LOCAL_PLANNER_ADAPTER_H

#include <nav_core/base_local_planner.h>
#include <nav_core2/local_planner.h>
#include <nav_core_adapter/costmap_adapter.h>
#include <nav_2d_utils/odom_subscriber.h>
#include <pluginlib/class_loader.h>
#include <memory>
#include <string>
#include <vector>

namespace nav_core_adapter
{

/**
 * @class LocalPlannerAdapter
 * @brief used for employing a nav_core2 local planner (such as dwb) as a nav_core plugin, like in move_base.
 */
class LocalPlannerAdapter: public nav_core::BaseLocalPlanner
{
public:
  LocalPlannerAdapter();

  // Standard ROS Local Planner Interface
  void initialize(std::string name, tf2_ros::Buffer* tf, costmap_2d::Costmap2DROS* costmap_ros) override;
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;
  bool isGoalReached() override;
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) override;

protected:
  /**
   * @brief Get the robot pose from the costmap and store as Pose2DStamped
   */
  bool getRobotPose(nav_2d_msgs::Pose2DStamped& pose2d);

  /**
   * @brief See if the back of the global plan matches the most recent goal pose
   * @return True if the plan has changed
   */
  bool hasGoalChanged(const nav_2d_msgs::Pose2DStamped& new_goal);

  // The most recent goal pose
  nav_2d_msgs::Pose2DStamped last_goal_;
  bool has_active_goal_;

  /**
   * @brief helper class for subscribing to odometry
   */
  std::shared_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;

  // Plugin handling
  pluginlib::ClassLoader<nav_core2::LocalPlanner> planner_loader_;
  boost::shared_ptr<nav_core2::LocalPlanner> planner_;

  // Pointer Copies
  TFListenerPtr tf_;

  std::shared_ptr<CostmapAdapter> costmap_adapter_;
  costmap_2d::Costmap2DROS* costmap_ros_;
};

}  // namespace nav_core_adapter

#endif  // NAV_CORE_ADAPTER_LOCAL_PLANNER_ADAPTER_H
