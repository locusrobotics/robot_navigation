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

#include <locomove_base/locomove_base.h>
#include <nav_core_adapter/costmap_adapter.h>
#include <nav_2d_utils/conversions.h>
#include <nav_2d_utils/path_ops.h>
#include <string>
#include <vector>

namespace locomove_base
{
/**
 * @brief Reimplementation of ClassLoader::getName without needing a ClassLoader
 *
 * See: https://github.com/ros/pluginlib/blob/0a3c0de442596b46026eb4c738130bce90421927/pluginlib/include/pluginlib/class_loader_imp.hpp#L480  // NOLINT(whitespace/line_length)
 */
std::string getNamespace(const std::string& s)
{
  std::vector<std::string> split;
  boost::split(split, s, boost::is_any_of("/:"));
  return split.back();
}

/**
 * @brief Copy parameter values to get backwards compatibility
 *
 * @param nh Node handle to read and write parameters to
 * @return the same node handle
 *
 * NB: The reason this returns a NodeHandle is to be able to call this code before the
 * locomotor object is initialized. The updated NodeHandle is used when initializing Locomotor.
 */
const ros::NodeHandle& loadBackwardsCompatibleParameters(const ros::NodeHandle& nh)
{
  // Double check robot_base_frame parameters for backwards compatibility
  if (!nh.hasParam("robot_base_frame"))
  {
    // If robot_base_frame was not set, use one of the values from the costmaps
    std::string planner_frame, controller_frame, value_to_use;
    nh.param("global_costmap/robot_base_frame", planner_frame, std::string(""));
    nh.param("local_costmap/robot_base_frame", controller_frame, std::string(""));
    if (planner_frame != controller_frame)
    {
      if (planner_frame.length() == 0)
      {
        value_to_use = controller_frame;
      }
      else if (controller_frame.length() == 0)
      {
        value_to_use = planner_frame;
      }
      else
      {
        ROS_WARN_NAMED("LocoMoveBase", "Two different robot_base_frames set for global and local planner. "
                       "This could be problematic. Using the global base frame.");
        value_to_use = planner_frame;
      }
    }
    else
    {
      value_to_use = planner_frame;
    }
    nh.setParam("robot_base_frame", value_to_use);
  }

  // Set the Global Planner Parameters
  std::string planner_class, planner_namespace;
  std::vector<std::string> plugin_namespaces;

  // Load the name of the nav_core1 global planner
  nh.param("base_global_planner", planner_class, std::string("navfn/NavfnROS"));
  planner_namespace = getNamespace(planner_class);
  if (planner_class == "nav_core_adapter::GlobalPlannerAdapter2")
  {
    // If the planner class is the adapter, then get and use the nav_core2 planner
    nh.param(planner_namespace + "/planner_name", planner_class, std::string("global_planner::GlobalPlanner"));
    planner_namespace = getNamespace(planner_class);
    nh.setParam(planner_namespace + "/plugin_class", planner_class);
    plugin_namespaces.push_back(planner_namespace);
    nh.setParam("global_planner_namespaces", plugin_namespaces);
  }
  else
  {
    // Otherwise, we need to inject the routing through the adapter
    std::string adapter_namespace = "global_planner_adapter";
    plugin_namespaces.push_back(adapter_namespace);
    nh.setParam("global_planner_namespaces", plugin_namespaces);
    nh.setParam(adapter_namespace + "/planner_name", planner_class);
    nh.setParam(adapter_namespace + "/plugin_class", "nav_core_adapter::GlobalPlannerAdapter2");
  }
  plugin_namespaces.clear();

  // Since the nav_core1 local planners are not compatible with nav_core2, we either need to load the
  // class that is being adapted, or we just load DWB instead
  nh.param("base_local_planner", planner_class, std::string("base_local_planner/TrajectoryPlannerROS"));
  planner_namespace = getNamespace(planner_class);
  if (planner_namespace == "LocalPlannerAdapter")
  {
    nh.param(planner_namespace + "/planner_name", planner_class, std::string("dwb_local_planner::DWBLocalPlanner"));
    planner_namespace = getNamespace(planner_class);
    nh.setParam(planner_namespace + "/plugin_class", planner_class);
    plugin_namespaces.push_back(planner_namespace);
    nh.setParam("local_planner_namespaces", plugin_namespaces);
  }
  else if (planner_namespace == "DWAPlannerROS")
  {
    ROS_WARN_NAMED("LocoMoveBase", "Using DWB as the local planner instead of DWA.");
    nh.setParam(planner_namespace + "/plugin_class", "dwb_local_planner::DWBLocalPlanner");
    plugin_namespaces.push_back(planner_namespace);
    nh.setParam("local_planner_namespaces", plugin_namespaces);
  }
  else
  {
    ROS_FATAL_NAMED("LocoMoveBase", "%s is unsupported in LocoMoveBase because it is not forwards compatible.",
                    planner_class.c_str());
  }

  return nh;
}

LocoMoveBase::LocoMoveBase(const ros::NodeHandle& nh) :
  private_nh_(nh), locomotor_(loadBackwardsCompatibleParameters(nh)),
  server_(ros::NodeHandle(), "move_base", false),
  recovery_loader_("nav_core", "nav_core::RecoveryBehavior"),
  local_planning_ex_(private_nh_, false), global_planning_ex_(private_nh_)
{
  private_nh_.param("planner_frequency", planner_frequency_, planner_frequency_);
  if (planner_frequency_ > 0.0)
  {
    desired_plan_duration_ = ros::Duration(1.0 / planner_frequency_);
    plan_loop_timer_ = private_nh_.createTimer(desired_plan_duration_, &LocoMoveBase::planLoopCallback,
                                               this, false, false);  // one_shot=false(default), auto_start=false
  }

  private_nh_.param("controller_frequency", controller_frequency_, controller_frequency_);
  desired_control_duration_ = ros::Duration(1.0 / controller_frequency_);
  control_loop_timer_ = private_nh_.createTimer(desired_control_duration_,
                                                &LocoMoveBase::controlLoopCallback,
                                                this, false, false);  // one_shot=false(default), auto_start=false
  locomotor_.initializeGlobalCostmap(global_planning_ex_);
  locomotor_.initializeGlobalPlanners(global_planning_ex_);
  locomotor_.initializeLocalCostmap(local_planning_ex_);
  locomotor_.initializeLocalPlanners(local_planning_ex_);

  planner_costmap_ros_ = getCostmapPointer(locomotor_.getGlobalCostmap());
  controller_costmap_ros_ = getCostmapPointer(locomotor_.getLocalCostmap());

  goal_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("current_goal", 1);

  private_nh_.param("recovery_behavior_enabled", recovery_behavior_enabled_, recovery_behavior_enabled_);

  // load any user specified recovery behaviors, and if that fails load the defaults
  if (!loadRecoveryBehaviors(private_nh_))
  {
    loadDefaultRecoveryBehaviors();
  }

  // Patience
  private_nh_.param("planner_patience", planner_patience_, planner_patience_);
  private_nh_.param("controller_patience", controller_patience_, controller_patience_);
  private_nh_.param("max_planning_retries", max_planning_retries_, max_planning_retries_);

  // Oscillation
  private_nh_.param("oscillation_timeout", oscillation_timeout_, oscillation_timeout_);
  private_nh_.param("oscillation_distance", oscillation_distance_, oscillation_distance_);

  // we'll provide a mechanism for some people to send goals as PoseStamped messages over a topic
  // they won't get any useful information back about its status, but this is useful for tools
  // like nav_view and rviz
  ros::NodeHandle simple_nh("move_base_simple");
  goal_sub_ = simple_nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, boost::bind(&LocoMoveBase::goalCB, this, _1));

  server_.registerGoalCallback(std::bind(&LocoMoveBase::executeCB, this));
  server_.start();

  resetState();
}

void LocoMoveBase::setGoal(nav_2d_msgs::Pose2DStamped goal)
{
  resetState();
  locomotor_.setGoal(goal);
  goal_pub_.publish(nav_2d_utils::pose2DToPoseStamped(goal));
  if (planner_frequency_ > 0.0)
  {
    plan_loop_timer_.start();
  }
  else
  {
    requestGlobalCostmapUpdate();
  }
}

costmap_2d::Costmap2DROS* LocoMoveBase::getCostmapPointer(const nav_core2::Costmap::Ptr& costmap)
{
  std::shared_ptr<nav_core_adapter::CostmapAdapter> ptr =
    std::dynamic_pointer_cast<nav_core_adapter::CostmapAdapter>(costmap);

  if (!ptr)
  {
    ROS_FATAL_NAMED("LocoMoveBase", "LocoMoveBase can only be used with the CostmapAdapter, not other Costmaps!");
    exit(-1);
  }
  return ptr->getCostmap2DROS();
}

void LocoMoveBase::resetState()
{
  // we'll start executing recovery behaviors at the beginning of our list
  recovery_index_ = 0;
  recovery_trigger_ = PLANNING_R;

  // we want to make sure that we reset the last time we had a valid plan and control
  last_valid_plan_ = ros::Time::now();
  last_valid_control_ = ros::Time::now();
  last_oscillation_reset_ = ros::Time::now();
  planning_retries_ = 0;
  has_global_plan_ = false;
}

void LocoMoveBase::publishZeroVelocity()
{
  locomotor_.publishTwist(nav_2d_msgs::Twist2DStamped());
}

void LocoMoveBase::requestNavigationFailure(const locomotor_msgs::ResultCode& result)
{
  locomotor_.requestNavigationFailure(local_planning_ex_, result,
    std::bind(&LocoMoveBase::onNavigationFailure, this, std::placeholders::_1));
}

void LocoMoveBase::planLoopCallback(const ros::TimerEvent& event)
{
  requestGlobalCostmapUpdate();
}

void LocoMoveBase::requestGlobalCostmapUpdate()
{
  locomotor_.requestGlobalCostmapUpdate(global_planning_ex_, global_planning_ex_,
    std::bind(&LocoMoveBase::onGlobalCostmapUpdate, this, std::placeholders::_1),
    std::bind(&LocoMoveBase::onGlobalCostmapException, this, std::placeholders::_1, std::placeholders::_2));
}

void LocoMoveBase::onGlobalCostmapUpdate(const ros::Duration& planning_time)
{
  // Run the global planning on the separate executor, but put the result on the main executor
  locomotor_.requestGlobalPlan(global_planning_ex_, local_planning_ex_,
    std::bind(&LocoMoveBase::onNewGlobalPlan, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&LocoMoveBase::onGlobalPlanningException, this, std::placeholders::_1, std::placeholders::_2));
}

void LocoMoveBase::onGlobalCostmapException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
{
  // If the planner_frequency is non-zero, the costmap will attempt to update again on its own (via the Timer).
  // If it is zero, then we manually request a new update
  if (planner_frequency_ == 0.0)
  {
    requestGlobalCostmapUpdate();
  }
}

void LocoMoveBase::onNewGlobalPlan(nav_2d_msgs::Path2D new_global_plan, const ros::Duration& planning_time)
{
  has_global_plan_ = true;
  last_valid_plan_ = ros::Time::now();
  planning_retries_ = 0;
  locomotor_.publishPath(new_global_plan);
  locomotor_.getCurrentLocalPlanner().setPlan(new_global_plan);
  if (planning_time > desired_plan_duration_)
  {
    ROS_WARN_NAMED("locomotor", "Global planning missed its desired rate of %.4fHz... "
                   "the loop actually took %.4f seconds (>%.4f).",
                   planner_frequency_, planning_time.toSec(), desired_plan_duration_.toSec());
  }
  if (recovery_trigger_ == PLANNING_R)
  {
    recovery_index_ = 0;
  }
  control_loop_timer_.start();
}

void LocoMoveBase::onGlobalPlanningException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
{
  if (has_global_plan_)
  {
    // If we have a global plan already, we can ignore this exception for the time being
    return;
  }

  ++planning_retries_;
  if (ros::Time::now() > last_valid_plan_ + ros::Duration(planner_patience_) ||
      planning_retries_ > max_planning_retries_)
  {
    publishZeroVelocity();
    recovery_trigger_ = PLANNING_R;
    recovery();
  }
}

void LocoMoveBase::controlLoopCallback(const ros::TimerEvent& event)
{
  locomotor_.requestLocalCostmapUpdate(local_planning_ex_, local_planning_ex_,
    std::bind(&LocoMoveBase::onLocalCostmapUpdate, this, std::placeholders::_1),
    std::bind(&LocoMoveBase::onLocalCostmapException, this, std::placeholders::_1, std::placeholders::_2));
}

void LocoMoveBase::onLocalCostmapUpdate(const ros::Duration& planning_time)
{
  // check for an oscillation condition
  if (oscillation_timeout_ > 0.0 && last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now())
  {
    publishZeroVelocity();
    recovery_trigger_ = OSCILLATION_R;
    recovery();
    return;
  }
  locomotor_.requestLocalPlan(local_planning_ex_, local_planning_ex_,
    std::bind(&LocoMoveBase::onNewLocalPlan, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&LocoMoveBase::onLocalPlanningException, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&LocoMoveBase::onNavigationCompleted, this));
}

void LocoMoveBase::onLocalCostmapException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
{
  ROS_WARN_NAMED("LocoMoveBase",
                 "Sensor data is out of date, we're not going to allow commanding of the base for safety");
  publishZeroVelocity();
}

void LocoMoveBase::onNewLocalPlan(nav_2d_msgs::Twist2DStamped new_command, const ros::Duration& planning_time)
{
  if (recovery_trigger_ == CONTROLLING_R)
  {
    recovery_index_ = 0;
  }
  last_valid_control_ = ros::Time::now();
  locomotor_.publishTwist(new_command);

  nav_2d_msgs::Pose2DStamped current_pose = locomotor_.getLocalRobotPose();

  // check to see if we've moved far enough to reset our oscillation timeout
  if (nav_2d_utils::poseDistance(current_pose.pose, oscillation_pose_) >= oscillation_distance_)
  {
    last_oscillation_reset_ = ros::Time::now();
    oscillation_pose_ = current_pose.pose;

    // if our last recovery was caused by oscillation, we want to reset the recovery index
    if (recovery_trigger_ == OSCILLATION_R)
    {
      recovery_index_ = 0;
    }
  }

  if (!server_.isActive())
  {
    return;
  }

  // publish feedback
  move_base_msgs::MoveBaseFeedback feedback;
  feedback.base_position = nav_2d_utils::pose2DToPoseStamped(current_pose);
  server_.publishFeedback(feedback);
}

void LocoMoveBase::onLocalPlanningException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
{
  // check if we've tried to find a valid control for longer than our time limit
  if (ros::Time::now() > last_valid_control_ + ros::Duration(controller_patience_))
  {
    recovery_trigger_ = CONTROLLING_R;
    recovery();
  }
  else
  {
    ROS_WARN_NAMED("Locomotor", "Local planning error. Creating new global plan.");
    control_loop_timer_.stop();
    requestGlobalCostmapUpdate();
    planning_retries_ = 0;
    publishZeroVelocity();
  }
}

void LocoMoveBase::onNavigationCompleted()
{
  ROS_INFO_NAMED("Locomotor", "Plan completed! Stopping.");
  if (server_.isActive())
  {
    server_.setSucceeded();
  }
  plan_loop_timer_.stop();
  control_loop_timer_.stop();
}

void LocoMoveBase::onNavigationFailure(const locomotor_msgs::ResultCode result)
{
  if (server_.isActive())
  {
    server_.setAborted();
  }
  plan_loop_timer_.stop();
  control_loop_timer_.stop();
}

void LocoMoveBase::recovery()
{
  ROS_DEBUG_NAMED("locomotor", "In clearing/recovery state");

  // we'll invoke whatever recovery behavior we're currently on if they're enabled
  if (recovery_behavior_enabled_ && recovery_index_ < recovery_behaviors_.size())
  {
    ROS_DEBUG_NAMED("locomotor_recovery", "Executing behavior %u of %zu",
                    recovery_index_, recovery_behaviors_.size());
    recovery_behaviors_[recovery_index_]->runBehavior();

    // we'll check if the recovery behavior actually worked
    // ROS_DEBUG_NAMED("locomotor_recovery","Going back to planning state");
    control_loop_timer_.stop();
    requestGlobalCostmapUpdate();

    // we at least want to give the robot some time to stop oscillating after executing the behavior
    last_valid_plan_ = ros::Time::now();
    last_oscillation_reset_ = ros::Time::now();
    planning_retries_ = 0;

    // update the index of the next recovery behavior that we'll try
    recovery_index_++;
  }
  else
  {
    ROS_DEBUG_NAMED("locomotor_recovery",
                    "All recovery behaviors have failed, locking the planner and disabling it.");

    if (recovery_trigger_ == CONTROLLING_R)
    {
      ROS_ERROR("Aborting because a valid control could not be found."
                "Even after executing all recovery behaviors");
      requestNavigationFailure(locomotor::makeResultCode(locomotor_msgs::ResultCode::LOCAL_PLANNER, CONTROLLING_R,
        "Failed to find a valid control. Even after executing recovery behaviors."));
    }
    else if (recovery_trigger_ == PLANNING_R)
    {
      ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
      requestNavigationFailure(locomotor::makeResultCode(locomotor_msgs::ResultCode::GLOBAL_PLANNER, PLANNING_R,
        "Failed to find a valid plan. Even after executing recovery behaviors."));
    }
    else if (recovery_trigger_ == OSCILLATION_R)
    {
      ROS_ERROR("Aborting because the robot appears to be oscillating over and over."
                "Even after executing all recovery behaviors");
      requestNavigationFailure(locomotor::makeResultCode(locomotor_msgs::ResultCode::LOCAL_PLANNER, OSCILLATION_R,
        "Robot is oscillating. Even after executing recovery behaviors."));
    }
  }
}

bool LocoMoveBase::loadRecoveryBehaviors(ros::NodeHandle node)
{
  XmlRpc::XmlRpcValue behavior_list;
  if (!node.getParam("recovery_behaviors", behavior_list))
  {
    // if no recovery_behaviors are specified, we'll just load the defaults
    return false;
  }

  if (behavior_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
  {
    ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. "
              "We'll use the default recovery behaviors instead.",
              behavior_list.getType());
    return false;
  }

  for (int i = 0; i < behavior_list.size(); ++i)
  {
    if (behavior_list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
      ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. "
                "We'll use the default recovery behaviors instead.",
                behavior_list[i].getType());
      return false;
    }

    if (!behavior_list[i].hasMember("name") || !behavior_list[i].hasMember("type"))
    {
      ROS_ERROR("Recovery behaviors must have a name and a type and this does not. "
                "Using the default recovery behaviors instead.");
      return false;
    }

    // check for recovery behaviors with the same name
    std::string name_i = behavior_list[i]["name"];
    for (int j = i + 1; j < behavior_list.size(); j++)
    {
      if (behavior_list[j].getType() != XmlRpc::XmlRpcValue::TypeStruct || !behavior_list[j].hasMember("name"))
      {
        continue;
      }

      std::string name_j = behavior_list[j]["name"];
      if (name_i == name_j)
      {
        ROS_ERROR("A recovery behavior with the name %s already exists, "
                  "this is not allowed. Using the default recovery behaviors instead.",
                  name_i.c_str());
        return false;
      }
    }
  }

  TFListenerPtr tf = locomotor_.getTFListener();
  // if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
  for (int i = 0; i < behavior_list.size(); ++i)
  {
    try
    {
      boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

      // shouldn't be possible, but it won't hurt to check
      if (behavior.get() == nullptr)
      {
        ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. "
                  "This should not happen");
        return false;
      }

      // initialize the recovery behavior with its name
      behavior->initialize(behavior_list[i]["name"], tf.get(), planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(behavior);
    }
    catch (pluginlib::PluginlibException& ex)
    {
      ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
      return false;
    }
  }
  // if we've made it here... we've constructed a recovery behavior list successfully
  return true;
}

// we'll load our default recovery behaviors here
void LocoMoveBase::loadDefaultRecoveryBehaviors()
{
  recovery_behaviors_.clear();
  // Transform shared pointers to raw pointers for backwards compatibility with the recovery behaviors
  TFListenerPtr tf = locomotor_.getTFListener();
  try
  {
    // we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
    ros::NodeHandle n("~");

    // first, we'll load a recovery behavior to clear the costmap
    boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(
      recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
    cons_clear->initialize("conservative_reset", tf.get(), planner_costmap_ros_, controller_costmap_ros_);
    recovery_behaviors_.push_back(cons_clear);

    // next, we'll load a recovery behavior to rotate in place
    bool clearing_rotation_allowed;
    n.param("clearing_rotation_allowed", clearing_rotation_allowed, true);

    boost::shared_ptr<nav_core::RecoveryBehavior> rotate(
      recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
    if (clearing_rotation_allowed)
    {
      rotate->initialize("rotate_recovery", tf.get(), planner_costmap_ros_, controller_costmap_ros_);
      recovery_behaviors_.push_back(rotate);
    }

    // next, we'll load a recovery behavior that will do an aggressive reset of the costmap
    boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(
      recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
    ags_clear->initialize("aggressive_reset", tf.get(), planner_costmap_ros_, controller_costmap_ros_);
    recovery_behaviors_.push_back(ags_clear);

    // we'll rotate in-place one more time
    if (clearing_rotation_allowed)
      recovery_behaviors_.push_back(rotate);
  }
  catch (pluginlib::PluginlibException& ex)
  {
    ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s",
              ex.what());
  }

  return;
}

void LocoMoveBase::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal)
{
  setGoal(nav_2d_utils::poseStampedToPose2D(*goal));
}

void LocoMoveBase::executeCB()
{
  auto move_base_goal = server_.acceptNewGoal();
  setGoal(nav_2d_utils::poseStampedToPose2D(move_base_goal->target_pose));
}

}  // namespace locomove_base

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_base");
  ros::NodeHandle nh("~");
  locomove_base::LocoMoveBase locomotor(nh);
  ros::spin();
  return 0;
}
