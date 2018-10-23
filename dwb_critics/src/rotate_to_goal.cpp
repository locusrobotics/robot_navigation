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
#include <dwb_critics/rotate_to_goal.h>
#include <dwb_local_planner/trajectory_utils.h>
#include <nav_2d_utils/parameters.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>
#include <angles/angles.h>
#include <string>
#include <vector>

const double EPSILON = 1E-5;

PLUGINLIB_EXPORT_CLASS(dwb_critics::RotateToGoalCritic, dwb_local_planner::TrajectoryCritic)

namespace dwb_critics
{

inline double hypot_sq(double dx, double dy)
{
  return dx * dx + dy * dy;
}

void RotateToGoalCritic::onInit()
{
  xy_goal_tolerance_ = nav_2d_utils::searchAndGetParam(critic_nh_, "xy_goal_tolerance", 0.25);
  xy_goal_tolerance_sq_ = xy_goal_tolerance_ * xy_goal_tolerance_;
  double stopped_xy_velocity = nav_2d_utils::searchAndGetParam(critic_nh_, "trans_stopped_velocity", 0.25);
  stopped_xy_velocity_sq_ = stopped_xy_velocity * stopped_xy_velocity;
  critic_nh_.param("slowing_factor", slowing_factor_, 5.0);
  critic_nh_.param("lookahead_time", lookahead_time_, -1.0);
  reset();
}

void RotateToGoalCritic::reset()
{
  in_window_ = false;
  rotating_ = false;
}

bool RotateToGoalCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                                 const geometry_msgs::Pose2D& goal,
                                 const nav_2d_msgs::Path2D& global_plan)
{
  double dxy_sq = hypot_sq(pose.x - goal.x, pose.y - goal.y);
  in_window_ = in_window_ || dxy_sq <= xy_goal_tolerance_sq_;
  current_xy_speed_sq_ = hypot_sq(vel.x, vel.y);
  rotating_ = rotating_ || (in_window_ && current_xy_speed_sq_ <= stopped_xy_velocity_sq_);
  goal_yaw_ = goal.theta;
  return true;
}

double RotateToGoalCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
  // If we're not sufficiently close to the goal, we don't care what the twist is
  if (!in_window_)
  {
    return 0.0;
  }
  else if (!rotating_)
  {
    double speed_sq = hypot_sq(traj.velocity.x, traj.velocity.y);
    if (speed_sq >= current_xy_speed_sq_)
    {
      throw nav_core2::IllegalTrajectoryException(name_, "Not slowing down near goal.");
    }
    return speed_sq * slowing_factor_ + scoreRotation(traj);
  }

  // If we're sufficiently close to the goal, any transforming velocity is invalid
  if (fabs(traj.velocity.x) > EPSILON || fabs(traj.velocity.y) > EPSILON)
  {
    throw nav_core2::IllegalTrajectoryException(name_, "Nonrotation command near goal.");
  }

  return scoreRotation(traj);
}

double RotateToGoalCritic::scoreRotation(const dwb_msgs::Trajectory2D& traj)
{
  if (traj.poses.empty())
  {
    throw nav_core2::IllegalTrajectoryException(name_, "Empty trajectory.");
  }

  double end_yaw;
  if (lookahead_time_ >= 0.0)
  {
    geometry_msgs::Pose2D eval_pose = dwb_local_planner::projectPose(traj, lookahead_time_);
    end_yaw = eval_pose.theta;
  }
  else
  {
    end_yaw = traj.poses.back().theta;
  }
  return fabs(angles::shortest_angular_distance(end_yaw, goal_yaw_));
}

} /* namespace dwb_critics */
