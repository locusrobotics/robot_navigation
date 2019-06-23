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

#ifndef LOCOMOTOR_LOCOMOTOR_H
#define LOCOMOTOR_LOCOMOTOR_H

#include <ros/ros.h>
#include <locomotor/executor.h>
#include <locomotor/publishers.h>
#include <locomotor_msgs/NavigationState.h>
#include <locomotor_msgs/ResultCode.h>
#include <nav_core2/exceptions.h>
#include <nav_core2/costmap.h>
#include <nav_core2/global_planner.h>
#include <nav_core2/local_planner.h>
#include <pluginlib/class_loader.h>
#include <nav_2d_utils/odom_subscriber.h>
#include <nav_2d_utils/plugin_mux.h>
#include <map>
#include <string>
#include <vector>

namespace locomotor
{
// Callback Type Definitions
using CostmapUpdateCallback = std::function<void (const ros::Duration&)>;
using CostmapUpdateExceptionCallback = std::function<void (nav_core2::NavCore2ExceptionPtr, const ros::Duration&)>;
using GlobalPlanCallback = std::function<void (const nav_2d_msgs::Path2D&, const ros::Duration&)>;
using LocalPlanCallback = std::function<void (const nav_2d_msgs::Twist2DStamped&, const ros::Duration&)>;
using PlannerExceptionCallback = std::function<void (nav_core2::NavCore2ExceptionPtr, const ros::Duration&)>;
using NavigationCompleteCallback = std::function<void ()>;
using NavigationFailureCallback = std::function<void (const locomotor_msgs::ResultCode)>;

inline locomotor_msgs::ResultCode makeResultCode(int component = -1, int result_code = -1,
                                                 const std::string& message = "")
{
  locomotor_msgs::ResultCode code;
  code.component = component;
  code.result_code = result_code;
  code.message = message;
  return code;
}

/**
 * @class Locomotor
 * @brief an extensible path planning coordination engine
 */
class Locomotor
{
public:
  /**
   * @brief Base Constructor.
   */
  explicit Locomotor(const ros::NodeHandle& private_nh);

  /**
   * @defgroup InitializeMethods
   * Specify which executor to run each component on.
   * @{
   */
  void initializeGlobalCostmap(Executor& ex);
  void initializeLocalCostmap(Executor& ex);
  void initializeGlobalPlanners(Executor& ex);
  void initializeLocalPlanners(Executor& ex);
  /** @} */  // end of InitializeMethods group

  /**
   * @brief Starts a new navigation to the given goal
   * @param goal The goal to navigate to
   */
  virtual void setGoal(nav_2d_msgs::Pose2DStamped goal);

  /**
   * @defgroup ActionRequests
   * @{
   */

  /**
   * @brief Request the global costmap get updated as a new callback
   * @param work_ex Executor to do the work on
   * @param result_ex Executor to put the result callback on
   * @param cb Callback for if the update succeeds
   * @param fail_cb Callback for if the update fails
   */
  void requestGlobalCostmapUpdate(Executor& work_ex, Executor& result_ex,
                                  CostmapUpdateCallback cb = nullptr,
                                  CostmapUpdateExceptionCallback fail_cb = nullptr);

  /**
   * @brief Request the local costmap get updated as a new callback
   * @param work_ex Executor to do the work on
   * @param result_ex Executor to put the result callback on
   * @param cb Callback for if the update succeeds
   * @param fail_cb Callback for if the update fails
   */
  void requestLocalCostmapUpdate(Executor& work_ex, Executor& result_ex,
                                 CostmapUpdateCallback cb = nullptr,
                                 CostmapUpdateExceptionCallback fail_cb = nullptr);

  /**
   * @brief Request the global planner get run as a new callback
   * @param work_ex Executor to do the work on
   * @param result_ex Executor to put the result callback on
   * @param cb Callback for if the planning succeeds
   * @param fail_cb Callback for if the planning fails
   */
  void requestGlobalPlan(Executor& work_ex, Executor& result_ex,
                         GlobalPlanCallback cb = nullptr,
                         PlannerExceptionCallback fail_cb = nullptr);

  /**
   * @brief Request the local planner get run as a new callback
   * @param work_ex Executor to do the work on
   * @param result_ex Executor to put the result callback on
   * @param cb Callback for if the planning succeeds
   * @param fail_cb Callback for if the planning fails
   * @param complete_cb Callback for if the navigation is now complete
   */
  void requestLocalPlan(Executor& work_ex, Executor& result_ex,
                        LocalPlanCallback cb = nullptr,
                        PlannerExceptionCallback fail_cb = nullptr,
                        NavigationCompleteCallback complete_cb = nullptr);

  /**
   * @brief Request that a onNavigationFailure event be added as a new callback
   * @param result_ex Executor to put the result callback on
   * @param result Information about the failure
   * @param cb Callback for handling failure
   */
  void requestNavigationFailure(Executor& result_ex, const locomotor_msgs::ResultCode& result,
                                NavigationFailureCallback cb = nullptr);
  /** @} */  // end of ActionRequests group

  const locomotor_msgs::NavigationState& getNavigationState() const { return state_; }

  // Global/Local Planner Access
  std::vector<std::string> getGlobalPlannerNames() const { return global_planner_mux_.getPluginNames(); }
  std::string getCurrentGlobalPlannerName() const { return global_planner_mux_.getCurrentPluginName(); }
  nav_core2::GlobalPlanner& getCurrentGlobalPlanner() { return global_planner_mux_.getCurrentPlugin(); }
  bool useGlobalPlanner(const std::string& name) { return global_planner_mux_.usePlugin(name); }

  std::vector<std::string> getLocalPlannerNames() const { return local_planner_mux_.getPluginNames(); }
  std::string getCurrentLocalPlannerName() const { return local_planner_mux_.getCurrentPluginName(); }
  nav_core2::LocalPlanner& getCurrentLocalPlanner() { return local_planner_mux_.getCurrentPlugin(); }
  bool useLocalPlanner(const std::string& name) { return local_planner_mux_.usePlugin(name); }

  // Costmap Access
  nav_core2::Costmap::Ptr getGlobalCostmap() const { return global_costmap_; }
  nav_core2::Costmap::Ptr getLocalCostmap() const { return local_costmap_; }

  // Additional Convenience Access
  TFListenerPtr getTFListener() const { return tf_; }
  nav_2d_msgs::Pose2DStamped getGlobalRobotPose() const { return getRobotPose(global_costmap_->getFrameId()); }
  nav_2d_msgs::Pose2DStamped getLocalRobotPose() const { return getRobotPose(local_costmap_->getFrameId()); }

  // Publisher Access
  void publishPath(const nav_2d_msgs::Path2D& global_plan) { path_pub_.publishPath(global_plan); }
  void publishTwist(const nav_2d_msgs::Twist2DStamped& command) { twist_pub_.publishTwist(command); }

protected:
  /**
   * @defgroup ActualActions
   * @{
   */
  void doCostmapUpdate(nav_core2::Costmap& costmap, Executor& result_ex,
                       CostmapUpdateCallback cb, CostmapUpdateExceptionCallback fail_cb);
  void makeGlobalPlan(Executor& result_ex, GlobalPlanCallback cb, PlannerExceptionCallback fail_cb);
  void makeLocalPlan(Executor& result_ex, LocalPlanCallback cb, PlannerExceptionCallback fail_cb,
                     NavigationCompleteCallback complete_cb);
  /** @} */  // end of ActualActions group

  /**
   * @brief Callback for when the local planner switches to ensure the new planner has up to date information
   * @param old_planner Name used on local_planner_mux of the old planner
   * @param new_planner Name used on local_planner_mux of the new planner
   */
  virtual void switchLocalPlannerCallback(const std::string& old_planner, const std::string& new_planner);

  // Costmap Loader
  pluginlib::ClassLoader<nav_core2::Costmap> costmap_loader_;

  // Global Planners and Costmap
  nav_2d_utils::PluginMux<nav_core2::GlobalPlanner> global_planner_mux_;
  nav_core2::Costmap::Ptr global_costmap_;

  // Local Planners and Costmap
  nav_2d_utils::PluginMux<nav_core2::LocalPlanner> local_planner_mux_;
  nav_core2::Costmap::Ptr local_costmap_;

  // Tools for getting the position and velocity of the robot
  nav_2d_msgs::Pose2DStamped getRobotPose(const std::string& target_frame) const;
  TFListenerPtr tf_;
  bool use_latest_pose_;
  std::shared_ptr<nav_2d_utils::OdomSubscriber> odom_sub_;

  // Core Variables
  ros::NodeHandle private_nh_;
  locomotor_msgs::NavigationState state_;
  std::string robot_base_frame_;

  // Publishers
  PathPublisher path_pub_;
  TwistPublisher twist_pub_;
};
}  // namespace locomotor

#endif  // LOCOMOTOR_LOCOMOTOR_H
