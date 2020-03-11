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
#include <locomotor/locomotor_action_server.h>
#include <nav_2d_utils/conversions.h>
#include <string>

namespace locomotor
{
using nav_core2::getResultCode;

/**
 * @class DoubleThreadLocomotor
 * @brief Connect the callbacks in Locomotor to do global and local planning on two separate timers
 *
 * Uses two Executors, local_planning_ex_ and global_planning_ex_
 *
 * When a new goal is recieved, it starts a timer to update the global costmap on the global_planning_ex.
 * When the global costmap update finishes, it requests a global plan on the global_planning_ex.
 * When the first global plan is generated, it starts a timer to update the local costmap on the local_planning_ex.
 * When the local costmap update finishes, it requests a local plan on the local_planning_ex.
 * When the goal is reached, it stops both the timers.
 */
class DoubleThreadLocomotor
{
public:
  explicit DoubleThreadLocomotor(const ros::NodeHandle& private_nh)
    : private_nh_(private_nh), locomotor_(private_nh_),
      local_planning_ex_(private_nh_, false), global_planning_ex_(private_nh_),
      as_(private_nh_, std::bind(&DoubleThreadLocomotor::setGoal, this, std::placeholders::_1))
  {
    private_nh_.param("planner_frequency", planner_frequency_, planner_frequency_);
    desired_plan_duration_ = ros::Duration(1.0 / planner_frequency_);
    plan_loop_timer_ = private_nh_.createTimer(desired_plan_duration_, &DoubleThreadLocomotor::planLoopCallback,
                                               this, false, false);  // one_shot=false(default), auto_start=false
    private_nh_.param("controller_frequency", controller_frequency_, controller_frequency_);
    desired_control_duration_ = ros::Duration(1.0 / controller_frequency_);
    control_loop_timer_ = private_nh_.createTimer(desired_control_duration_,
                                                  &DoubleThreadLocomotor::controlLoopCallback,
                                                  this, false, false);  // one_shot=false(default), auto_start=false
    locomotor_.initializeGlobalCostmap(global_planning_ex_);
    locomotor_.initializeGlobalPlanners(global_planning_ex_);
    locomotor_.initializeLocalCostmap(local_planning_ex_);
    locomotor_.initializeLocalPlanners(local_planning_ex_);
  }

  void setGoal(nav_2d_msgs::Pose2DStamped goal)
  {
    locomotor_.setGoal(goal);
    plan_loop_timer_.start();
  }

protected:
  void planLoopCallback(const ros::TimerEvent& event)
  {
    requestGlobalCostmapUpdate();
  }

  void requestGlobalCostmapUpdate()
  {
    locomotor_.requestGlobalCostmapUpdate(global_planning_ex_, global_planning_ex_,
      std::bind(&DoubleThreadLocomotor::onGlobalCostmapUpdate, this, std::placeholders::_1),
      std::bind(&DoubleThreadLocomotor::onGlobalCostmapException, this, std::placeholders::_1, std::placeholders::_2));
  }

  void requestNavigationFailure(const locomotor_msgs::ResultCode& result)
  {
    locomotor_.requestNavigationFailure(local_planning_ex_, result,
      std::bind(&DoubleThreadLocomotor::onNavigationFailure, this, std::placeholders::_1));
  }

  // Locomotor Callbacks
  void onGlobalCostmapUpdate(const ros::Duration& planning_time)
  {
    // Run the global planning on the separate executor, but put the result on the main executor
    locomotor_.requestGlobalPlan(global_planning_ex_, local_planning_ex_,
      std::bind(&DoubleThreadLocomotor::onNewGlobalPlan, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DoubleThreadLocomotor::onGlobalPlanningException, this, std::placeholders::_1, std::placeholders::_2));
  }

  void onGlobalCostmapException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
  {
    requestNavigationFailure(makeResultCode(locomotor_msgs::ResultCode::GLOBAL_COSTMAP, getResultCode(e_ptr),
                                            "Global Costmap failure."));
  }

  void onNewGlobalPlan(nav_2d_msgs::Path2D new_global_plan, const ros::Duration& planning_time)
  {
    locomotor_.publishPath(new_global_plan);
    locomotor_.getCurrentLocalPlanner().setPlan(new_global_plan);

    if (planning_time > desired_plan_duration_)
    {
      ROS_WARN_NAMED("locomotor", "Global planning missed its desired rate of %.4fHz... "
                     "the loop actually took %.4f seconds (>%.4f).",
                     planner_frequency_, planning_time.toSec(), desired_plan_duration_.toSec());
    }
    control_loop_timer_.start();
    as_.publishFeedback(locomotor_.getNavigationState());
  }

  void onGlobalPlanningException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
  {
    ROS_ERROR_NAMED("Locomotor", "Global planning error. Giving up.");
    requestNavigationFailure(makeResultCode(locomotor_msgs::ResultCode::GLOBAL_PLANNER, getResultCode(e_ptr),
                                            "Global Planning Failure."));
  }

  void controlLoopCallback(const ros::TimerEvent& event)
  {
    locomotor_.requestLocalCostmapUpdate(local_planning_ex_, local_planning_ex_,
      std::bind(&DoubleThreadLocomotor::onLocalCostmapUpdate, this, std::placeholders::_1),
      std::bind(&DoubleThreadLocomotor::onLocalCostmapException, this, std::placeholders::_1, std::placeholders::_2));
  }

  void onLocalCostmapUpdate(const ros::Duration& planning_time)
  {
    locomotor_.requestLocalPlan(local_planning_ex_, local_planning_ex_,
      std::bind(&DoubleThreadLocomotor::onNewLocalPlan, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DoubleThreadLocomotor::onLocalPlanningException, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DoubleThreadLocomotor::onNavigationCompleted, this));
  }

  void onLocalCostmapException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
  {
    requestNavigationFailure(makeResultCode(locomotor_msgs::ResultCode::LOCAL_COSTMAP, getResultCode(e_ptr),
                                            "Local Costmap failure."));
  }

  void onNewLocalPlan(nav_2d_msgs::Twist2DStamped new_command, const ros::Duration& planning_time)
  {
    locomotor_.publishTwist(new_command);
    if (planning_time > desired_control_duration_)
    {
      ROS_WARN_NAMED("locomotor", "Control loop missed its desired rate of %.4fHz... "
                     "the loop actually took %.4f seconds (>%.4f).",
                     controller_frequency_, planning_time.toSec(), desired_control_duration_.toSec());
    }
    as_.publishFeedback(locomotor_.getNavigationState());
  }

  void onLocalPlanningException(nav_core2::NavCore2ExceptionPtr e_ptr, const ros::Duration& planning_time)
  {
    ROS_WARN_NAMED("Locomotor", "Local planning error. Creating new global plan.");
    control_loop_timer_.stop();
    requestGlobalCostmapUpdate();
  }

  void onNavigationCompleted()
  {
    ROS_INFO_NAMED("Locomotor", "Plan completed! Stopping.");
    plan_loop_timer_.stop();
    control_loop_timer_.stop();
    as_.completeNavigation();
  }

  void onNavigationFailure(const locomotor_msgs::ResultCode result)
  {
    plan_loop_timer_.stop();
    control_loop_timer_.stop();
    as_.failNavigation(result);
  }

  ros::NodeHandle private_nh_;
  // Locomotor Object
  Locomotor locomotor_;

  // Timer Stuff
  double planner_frequency_ { 10.0 }, controller_frequency_ { 20.0 };
  ros::Duration desired_plan_duration_, desired_control_duration_;
  ros::Timer plan_loop_timer_, control_loop_timer_;

  // The Two Executors
  Executor local_planning_ex_, global_planning_ex_;

  // Action Server
  LocomotorActionServer as_;
};
};  // namespace locomotor

int main(int argc, char** argv)
{
  ros::init(argc, argv, "locomotor_node");
  ros::NodeHandle nh("~");
  locomotor::DoubleThreadLocomotor sm(nh);
  ros::spin();
  return 0;
}
