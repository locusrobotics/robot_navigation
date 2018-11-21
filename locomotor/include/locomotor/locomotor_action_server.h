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

#ifndef LOCOMOTOR_LOCOMOTOR_ACTION_SERVER_H
#define LOCOMOTOR_LOCOMOTOR_ACTION_SERVER_H

#include <actionlib/server/simple_action_server.h>
#include <locomotor_msgs/NavigateToPoseAction.h>
#include <locomotor_msgs/NavigationState.h>
#include <string>

namespace locomotor
{
using NewGoalCallback = std::function<void (const nav_2d_msgs::Pose2DStamped&)>;

class LocomotorActionServer
{
public:
  LocomotorActionServer(const ros::NodeHandle nh, NewGoalCallback cb, const std::string name = "navigate");
  void publishFeedback(const locomotor_msgs::NavigationState& nav_state);
  void completeNavigation();
  void failNavigation(const locomotor_msgs::ResultCode& result_code);
protected:
  void preGoalCallback();
  void preemptCallback();
  actionlib::SimpleActionServer<locomotor_msgs::NavigateToPoseAction> navigate_action_server_;
  locomotor_msgs::NavigateToPoseFeedback feedback_;
  NewGoalCallback goal_cb_;
};
}  // namespace locomotor

#endif  // LOCOMOTOR_LOCOMOTOR_ACTION_SERVER_H
