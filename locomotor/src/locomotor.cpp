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

#include <locomotor/locomotor.h>
#include <nav_2d_utils/tf_help.h>
#include <functional>
#include <string>

namespace locomotor
{
ros::Duration getTimeDiffFromNow(const ros::WallTime& start_time)
{
  ros::WallDuration duration = ros::WallTime::now() - start_time;
  return ros::Duration(duration.sec, duration.nsec);
}

Locomotor::Locomotor(const ros::NodeHandle& private_nh) :
  costmap_loader_("nav_core2", "nav_core2::Costmap"),
  global_planner_mux_("nav_core2", "nav_core2::GlobalPlanner",
                      "global_planner_namespaces", "dlux_global_planner::DluxGlobalPlanner",
                      "current_global_planner", "switch_global_planner"),
  local_planner_mux_("nav_core2", "nav_core2::LocalPlanner",
                     "local_planner_namespaces", "dwb_local_planner::DWBLocalPlanner",
                     "current_local_planner", "switch_local_planner"),
  private_nh_(private_nh), path_pub_(private_nh_), twist_pub_(private_nh_)
{
  tf_ = std::make_shared<tf::TransformListener>(ros::Duration(10));

  private_nh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

  // If true, when getting robot pose, use ros::Time(0) instead of ros::Time::now()
  private_nh_.param("use_latest_pose", use_latest_pose_, true);

  local_planner_mux_.setSwitchCallback(std::bind(&Locomotor::switchLocalPlannerCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  ros::NodeHandle global_nh;
  odom_sub_ = std::make_shared<nav_2d_utils::OdomSubscriber>(global_nh);
}

void Locomotor::initializeGlobalCostmap(Executor& ex)
{
  std::string costmap_class;
  private_nh_.param("global_costmap_class", costmap_class, std::string("nav_core_adapter::CostmapAdapter"));
  ROS_INFO_NAMED("Locomotor", "Loading Global Costmap %s", costmap_class.c_str());
  global_costmap_ = costmap_loader_.createUniqueInstance(costmap_class);
  ROS_INFO_NAMED("Locomotor", "Initializing Global Costmap");
  global_costmap_->initialize(ex.getNodeHandle(), "global_costmap", tf_);
}

void Locomotor::initializeLocalCostmap(Executor& ex)
{
  std::string costmap_class;
  private_nh_.param("local_costmap_class", costmap_class, std::string("nav_core_adapter::CostmapAdapter"));
  ROS_INFO_NAMED("Locomotor", "Loading Local Costmap %s", costmap_class.c_str());
  local_costmap_ = costmap_loader_.createUniqueInstance(costmap_class);
  ROS_INFO_NAMED("Locomotor", "Initializing Local Costmap");
  local_costmap_->initialize(ex.getNodeHandle(), "local_costmap", tf_);
}

void Locomotor::initializeGlobalPlanners(Executor& ex)
{
  for (auto planner_name : global_planner_mux_.getPluginNames())
  {
    ROS_INFO_NAMED("Locomotor", "Initializing global planner %s", planner_name.c_str());
    global_planner_mux_.getPlugin(planner_name).initialize(ex.getNodeHandle(), planner_name, tf_, global_costmap_);
  }
}

void Locomotor::initializeLocalPlanners(Executor& ex)
{
  for (auto planner_name : local_planner_mux_.getPluginNames())
  {
    ROS_INFO_NAMED("Locomotor", "Initializing local planner %s", planner_name.c_str());
    local_planner_mux_.getPlugin(planner_name).initialize(ex.getNodeHandle(), planner_name, tf_, local_costmap_);
  }
}

void Locomotor::setGoal(nav_2d_msgs::Pose2DStamped goal)
{
  local_planner_mux_.getCurrentPlugin().setGoalPose(goal);
  state_ = locomotor_msgs::NavigationState();
  state_.goal = goal;
}

void Locomotor::switchLocalPlannerCallback(const std::string&, const std::string& new_planner)
{
  auto& new_local_planner = local_planner_mux_.getPlugin(new_planner);
  new_local_planner.setGoalPose(state_.goal);
  new_local_planner.setPlan(state_.global_plan);
}

void Locomotor::requestGlobalCostmapUpdate(Executor& work_ex, Executor& result_ex,
                                           CostmapUpdateCallback cb, CostmapUpdateExceptionCallback fail_cb)
{
  work_ex.addCallback(
    std::bind(&Locomotor::doCostmapUpdate, this, std::ref(*global_costmap_), std::ref(result_ex), cb, fail_cb));
}

void Locomotor::requestLocalCostmapUpdate(Executor& work_ex, Executor& result_ex,
                                          CostmapUpdateCallback cb, CostmapUpdateExceptionCallback fail_cb)
{
  work_ex.addCallback(
    std::bind(&Locomotor::doCostmapUpdate, this, std::ref(*local_costmap_), std::ref(result_ex), cb, fail_cb));
}

void Locomotor::requestGlobalPlan(Executor& work_ex, Executor& result_ex,
                                  GlobalPlanCallback cb, PlannerExceptionCallback fail_cb)
{
  work_ex.addCallback(std::bind(&Locomotor::makeGlobalPlan, this, std::ref(result_ex), cb, fail_cb));
}

void Locomotor::requestLocalPlan(Executor& work_ex, Executor& result_ex,
                                 LocalPlanCallback cb, PlannerExceptionCallback fail_cb,
                                 NavigationCompleteCallback complete_cb)
{
  work_ex.addCallback(std::bind(&Locomotor::makeLocalPlan, this, std::ref(result_ex), cb, fail_cb, complete_cb));
}

void Locomotor::requestNavigationFailure(Executor& result_ex, const locomotor_msgs::ResultCode& result,
                                         NavigationFailureCallback cb)
{
  result_ex.addCallback(std::bind(cb, result));
}

void Locomotor::doCostmapUpdate(nav_core2::Costmap& costmap, Executor& result_ex,
                                CostmapUpdateCallback cb, CostmapUpdateExceptionCallback fail_cb)
{
  ros::WallTime start_t = ros::WallTime::now();
  try
  {
    {
      boost::unique_lock<boost::recursive_mutex> lock(*(costmap.getMutex()));
      costmap.update();
    }
    if (cb) result_ex.addCallback(std::bind(cb, getTimeDiffFromNow(start_t)));
  }
  catch (const nav_core2::CostmapException& e)
  {
    if (fail_cb)
      result_ex.addCallback(std::bind(fail_cb, std::current_exception(), getTimeDiffFromNow(start_t)));
  }
}

void Locomotor::makeGlobalPlan(Executor& result_ex, GlobalPlanCallback cb, PlannerExceptionCallback fail_cb)
{
  ros::WallTime start_t = ros::WallTime::now();
  try
  {
    state_.global_pose = getGlobalRobotPose();

    {
      boost::unique_lock<boost::recursive_mutex> lock(*(global_costmap_->getMutex()));
      state_.global_plan = global_planner_mux_.getCurrentPlugin().makePlan(state_.global_pose, state_.goal);
    }
    if (cb) result_ex.addCallback(std::bind(cb, state_.global_plan, getTimeDiffFromNow(start_t)));
  }
  // if we didn't get a plan and we are in the planning state (the robot isn't moving)
  catch (const nav_core2::PlannerException& e)
  {
    if (fail_cb)
      result_ex.addCallback(std::bind(fail_cb, std::current_exception(), getTimeDiffFromNow(start_t)));
  }
}

void Locomotor::makeLocalPlan(Executor& result_ex, LocalPlanCallback cb, PlannerExceptionCallback fail_cb,
                              NavigationCompleteCallback complete_cb)
{
  state_.global_pose = getGlobalRobotPose();
  state_.local_pose = getLocalRobotPose();
  state_.current_velocity = odom_sub_->getTwistStamped();
  auto& local_planner = local_planner_mux_.getCurrentPlugin();

  if (local_planner.isGoalReached(state_.local_pose, state_.current_velocity.velocity))
  {
    if (complete_cb) result_ex.addCallback(std::bind(complete_cb));
    return;
  }

  // Actual Control
  // Extra Scope for Mutex
  {
    boost::unique_lock<boost::recursive_mutex> lock(*(local_costmap_->getMutex()));
    ros::WallTime start_t = ros::WallTime::now();
    try
    {
      state_.command_velocity = local_planner.computeVelocityCommands(state_.local_pose,
                                                                      state_.current_velocity.velocity);
      lock.unlock();
      if (cb) result_ex.addCallback(std::bind(cb, state_.command_velocity, getTimeDiffFromNow(start_t)));
    }
    catch (const nav_core2::PlannerException& e)
    {
      lock.unlock();
      if (fail_cb)
        result_ex.addCallback(std::bind(fail_cb, std::current_exception(), getTimeDiffFromNow(start_t)));
    }
  }
}

nav_2d_msgs::Pose2DStamped Locomotor::getRobotPose(const std::string& target_frame) const
{
  nav_2d_msgs::Pose2DStamped robot_pose, transformed_pose;
  robot_pose.header.frame_id = robot_base_frame_;
  if (!use_latest_pose_)
  {
    robot_pose.header.stamp = ros::Time::now();
  }
  bool ret = nav_2d_utils::transformPose(tf_, target_frame, robot_pose, transformed_pose);
  if (!ret)
  {
    throw nav_core2::PlannerTFException("Could not get pose into costmap frame!");
  }
  return transformed_pose;
}

}  // namespace locomotor
