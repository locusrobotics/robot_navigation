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
#ifndef NAV_CORE2_LOCAL_PLANNER_H
#define NAV_CORE2_LOCAL_PLANNER_H

#include <nav_core2/common.h>
#include <nav_core2/costmap.h>
#include <nav_2d_msgs/Path2D.h>
#include <nav_2d_msgs/Pose2DStamped.h>
#include <nav_2d_msgs/Twist2D.h>
#include <nav_2d_msgs/Twist2DStamped.h>
#include <string>

namespace nav_core2
{

/**
 * @class LocalPlanner
 * @brief Provides an interface for local planners used in navigation.
 */
class LocalPlanner
{
public:
  /**
   * @brief Virtual Destructor
   */
  virtual ~LocalPlanner() {}

  /**
   * @brief Constructs the local planner
   *
   * ROS parameters/topics are expected to be in the parent/name namespace.
   * It is suggested that all NodeHandles in the planner use the parent NodeHandle's callback queue.
   *
   * @param parent NodeHandle to derive other NodeHandles from
   * @param name The name to give this instance of the local planner
   * @param tf A pointer to a transform listener
   * @param costmap A pointer to the costmap
   */
  virtual void initialize(const ros::NodeHandle& parent, const std::string& name,
                          TFListenerPtr tf, Costmap::Ptr costmap) = 0;

  /**
   * @brief Sets the global goal for this local planner.
   * @param goal_pose The Goal Pose
   */
  virtual void setGoalPose(const nav_2d_msgs::Pose2DStamped& goal_pose) = 0;

  /**
   * @brief Sets the global plan for this local planner.
   *
   * @param path The global plan
   */
  virtual void setPlan(const nav_2d_msgs::Path2D& path) = 0;

  /**
   * @brief Compute the best command given the current pose, velocity and goal
   *
   * Get the local plan, given an initial pose, velocity and goal pose.
   * It is presumed that the global plan is already set.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @return          The best computed velocity
   */
  virtual nav_2d_msgs::Twist2DStamped computeVelocityCommands(const nav_2d_msgs::Pose2DStamped& pose,
                                                              const nav_2d_msgs::Twist2D& velocity) = 0;

  /**
   * @brief Check to see whether the robot has reached its goal
   *
   * This tests whether the robot has fully reached the goal, given the current pose and velocity.
   *
   * @param query_pose The pose to check, in local costmap coordinates.
   * @param velocity   Velocity to check
   * @return           True if the goal conditions have been met
   */
  virtual bool isGoalReached(const nav_2d_msgs::Pose2DStamped& pose, const nav_2d_msgs::Twist2D& velocity) = 0;
};
}  // namespace nav_core2

#endif  // NAV_CORE2_LOCAL_PLANNER_H
