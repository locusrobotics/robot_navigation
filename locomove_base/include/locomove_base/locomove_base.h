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

#ifndef LOCOMOVE_BASE_LOCOMOVE_BASE_H
#define LOCOMOVE_BASE_LOCOMOVE_BASE_H

#include <actionlib/server/simple_action_server.h>
#include <locomotor/locomotor.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <nav_core/recovery_behavior.h>
#include <string>
#include <vector>

namespace locomove_base
{
enum RecoveryTrigger
{
  PLANNING_R, CONTROLLING_R, OSCILLATION_R
};

class LocoMoveBase
{
public:
  explicit LocoMoveBase(const ros::NodeHandle& nh);
  void setGoal(nav_2d_msgs::Pose2DStamped goal);
  void resetState();
protected:
  void requestNavigationFailure(const locomotor_msgs::ResultCode& result);

  void planLoopCallback(const ros::TimerEvent& event);
  void requestGlobalCostmapUpdate();

  void onGlobalCostmapUpdate(const ros::Duration& planning_time);
  void onGlobalCostmapException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time);

  void onNewGlobalPlan(nav_2d_msgs::Path2D new_global_plan, const ros::Duration& planning_time);
  void onGlobalPlanningException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time);

  void controlLoopCallback(const ros::TimerEvent& event);

  void onLocalCostmapUpdate(const ros::Duration& planning_time);
  void onLocalCostmapException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time);

  void onNewLocalPlan(nav_2d_msgs::Twist2DStamped new_command, const ros::Duration& planning_time);
  void onLocalPlanningException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time);

  void onNavigationCompleted();
  void onNavigationFailure(const locomotor_msgs::ResultCode result);

  void publishZeroVelocity();

  /**
   * @brief  Load the recovery behaviors for the navigation stack from the parameter server
   * @param node The ros::NodeHandle to be used for loading parameters
   * @return True if the recovery behaviors were loaded successfully, false otherwise
   */
  bool loadRecoveryBehaviors(ros::NodeHandle node);

  /**
   * @brief  Loads the default recovery behaviors for the navigation stack
   */
  void loadDefaultRecoveryBehaviors();

  void recovery();

  ros::NodeHandle private_nh_;
  locomotor::Locomotor locomotor_;

  // Simple Goal Handling
  void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
  ros::Subscriber goal_sub_;

  // MoveBaseAction
  void executeCB();
  actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction> server_;

  // Recoveries
  RecoveryTrigger recovery_trigger_;
  pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;
  std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
  unsigned int recovery_index_;
  bool recovery_behavior_enabled_ { true };

  // Timer Stuff
  double planner_frequency_ { 10.0 }, controller_frequency_ { 20.0 };
  ros::Duration desired_plan_duration_, desired_control_duration_;
  ros::Timer plan_loop_timer_, control_loop_timer_;

  // The Two Executors
  locomotor::Executor local_planning_ex_, global_planning_ex_;

  // Patience
  bool has_global_plan_;
  double planner_patience_ { 5.0 }, controller_patience_ { 15.0 };
  ros::Time last_valid_plan_, last_valid_control_;
  int max_planning_retries_ {-1},  // disabled by default
      planning_retries_;

  // Oscillation variables
  double oscillation_timeout_ {0.0}, oscillation_distance_{0.5};
  ros::Time last_oscillation_reset_;
  geometry_msgs::Pose2D oscillation_pose_;

  // Costmap Pointer Copies
  costmap_2d::Costmap2DROS* getCostmapPointer(const nav_core2::Costmap::Ptr& costmap);
  costmap_2d::Costmap2DROS* planner_costmap_ros_;
  costmap_2d::Costmap2DROS* controller_costmap_ros_;

  // Debug Publishing
  ros::Publisher goal_pub_;
};
}  // namespace locomove_base

#endif  // LOCOMOVE_BASE_LOCOMOVE_BASE_H
