/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Locus Robotics
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

#include <locomotor/locomotor_action_server.h>
#include <nav_2d_utils/path_ops.h>
#include <string>

namespace locomotor
{

LocomotorActionServer::LocomotorActionServer(const ros::NodeHandle nh, NewGoalCallback cb, const std::string name)
  : navigate_action_server_(nh, name, false), goal_cb_(cb)
{
  navigate_action_server_.registerGoalCallback(std::bind(&LocomotorActionServer::preGoalCallback, this));
  navigate_action_server_.registerPreemptCallback(std::bind(&LocomotorActionServer::preemptCallback, this));
  navigate_action_server_.start();
}
void LocomotorActionServer::publishFeedback(const locomotor_msgs::NavigationState& nav_state)
{
  if (!navigate_action_server_.isActive()) return;

  // Update The Feedback
  const geometry_msgs::Pose2D& pose = nav_state.global_pose.pose;
  if (feedback_.state.global_pose.header.frame_id.length() > 0)
  {
    // If we already have a global pose saved, calculate the distance traveled
    const geometry_msgs::Pose2D& prev_pose = feedback_.state.global_pose.pose;
    feedback_.distance_traveled += nav_2d_utils::poseDistance(prev_pose, pose);
  }

  feedback_.state = nav_state;

  if (feedback_.state.global_plan.poses.size() > 0)
  {
    feedback_.estimated_distance_remaining = nav_2d_utils::getPlanLength(feedback_.state.global_plan, pose);
    double total_distance = feedback_.distance_traveled + feedback_.estimated_distance_remaining;
    if (total_distance != 0.0)
      feedback_.percent_complete = feedback_.distance_traveled / total_distance;
  }

  navigate_action_server_.publishFeedback(feedback_);
}

void LocomotorActionServer::completeNavigation()
{
  if (!navigate_action_server_.isActive()) return;
  navigate_action_server_.setSucceeded();
}
void LocomotorActionServer::failNavigation(const locomotor_msgs::ResultCode& result_code)
{
  if (!navigate_action_server_.isActive()) return;
  locomotor_msgs::NavigateToPoseResult result;
  result.result_code = result_code;
  navigate_action_server_.setAborted(result, result_code.message);
}

void LocomotorActionServer::preGoalCallback()
{
  feedback_.distance_traveled = 0.0;
  feedback_.percent_complete = 0.0;
  feedback_.estimated_distance_remaining = 0.0;
  feedback_.state = locomotor_msgs::NavigationState();

  auto full_goal = navigate_action_server_.acceptNewGoal();
  goal_cb_(full_goal->goal);
}

void LocomotorActionServer::preemptCallback()
{
  if (!navigate_action_server_.isActive()) return;
  locomotor_msgs::NavigateToPoseResult result;
  result.result_code.result_code = -1;
  result.result_code.message = "Preempted.";
  navigate_action_server_.setPreempted(result, result.result_code.message);
}

}  // namespace locomotor
