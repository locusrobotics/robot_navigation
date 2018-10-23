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

#ifndef DWB_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H
#define DWB_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H

#include <ros/ros.h>
#include <nav_2d_msgs/Twist2D.h>
#include <dwb_msgs/Trajectory2D.h>
#include <vector>

namespace dwb_local_planner
{

/**
 * @class TrajectoryGenerator
 * @brief Interface for iterating through possible velocities and creating trajectories
 *
 * This class defines the plugin interface for two separate but related components.
 *
 * First, this class provides an iterator interface for exploring all of the velocities
 * to search, given the current velocity.
 *
 * Second, the class gives an independent interface for creating a trajectory from a twist,
 * i.e. projecting it out in time and space.
 *
 * Both components rely heavily on the robot's kinematic model, and can share many parameters,
 * which is why they are grouped into a singular class.
 */
class TrajectoryGenerator
{
public:
  using Ptr = std::shared_ptr<dwb_local_planner::TrajectoryGenerator>;

  virtual ~TrajectoryGenerator() {}

  /**
   * @brief Initialize parameters as needed
   * @param nh NodeHandle to read parameters from
   */
  virtual void initialize(ros::NodeHandle& nh) = 0;

  /**
   * @brief Reset the state (if any) when the planner gets a new goal
   */
  virtual void reset() {}

  /**
   * @brief Start a new iteration based on the current velocity
   * @param current_velocity
   */
  virtual void startNewIteration(const nav_2d_msgs::Twist2D& current_velocity) = 0;

  /**
   * @brief Test to see whether there are more twists to test
   * @return True if more twists, false otherwise
   */
  virtual bool hasMoreTwists() = 0;

  /**
   * @brief Return the next twist and advance the iteration
   * @return The Twist!
   */
  virtual nav_2d_msgs::Twist2D nextTwist() = 0;

  /**
   * @brief Get all the twists for an iteration.
   *
   * Note: Resets the iterator if one is in process
   *
   * @param current_velocity
   * @return all the twists
   */
  virtual std::vector<nav_2d_msgs::Twist2D> getTwists(const nav_2d_msgs::Twist2D& current_velocity)
  {
    std::vector<nav_2d_msgs::Twist2D> twists;
    startNewIteration(current_velocity);
    while (hasMoreTwists())
    {
      twists.push_back(nextTwist());
    }
    return twists;
  }

  /**
   * @brief Given a cmd_vel in the robot's frame and initial conditions, generate a Trajectory2D
   * @param start_pose Current robot location
   * @param start_vel Current robot velocity
   * @param cmd_vel The desired command velocity
   */
  virtual dwb_msgs::Trajectory2D generateTrajectory(const geometry_msgs::Pose2D& start_pose,
                                                    const nav_2d_msgs::Twist2D& start_vel,
                                                    const nav_2d_msgs::Twist2D& cmd_vel) = 0;
};

}  // namespace dwb_local_planner

#endif  // DWB_LOCAL_PLANNER_TRAJECTORY_GENERATOR_H
