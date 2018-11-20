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

#include <locomotor/publishers.h>
#include <nav_2d_utils/path_ops.h>
#include <nav_2d_utils/conversions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>
#include <string>

namespace locomotor
{
PathPublisher::PathPublisher(ros::NodeHandle& nh)
{
  std::string topic, publish_type;
  nh.param("global_plan_topic", topic, std::string("global_plan"));
  nh.param("global_plan_type", publish_type, std::string("Path3D"));
  nh.param("global_plan_epsilon", global_plan_epsilon_, 0.1);

  if (publish_type == "Path2D")
  {
    path_type_ = PathType::PATH_2D;
    pub_ = nh.advertise<nav_2d_msgs::Path2D>(topic, 1);
  }
  else if (publish_type == "")
  {
    path_type_ = PathType::NO_PATH;
  }
  else
  {
    if (publish_type != "Path3D")
    {
      ROS_ERROR_NAMED("Locomotor", "Unknown global_plan_type \"%s\". Using Path3D instead.", publish_type.c_str());
    }
    path_type_ = PathType::PATH_3D;
    pub_ = nh.advertise<nav_msgs::Path>(topic, 1);
  }
}

void PathPublisher::publishPath(const nav_2d_msgs::Path2D& global_plan)
{
  if (pub_.getNumSubscribers() == 0) return;

  nav_2d_msgs::Path2D to_publish = global_plan;
  if (global_plan_epsilon_ >= 0.0)
  {
    to_publish = nav_2d_utils::compressPlan(global_plan, global_plan_epsilon_);
  }
  switch (path_type_)
  {
  case PathType::PATH_2D:
    pub_.publish(to_publish);
    break;
  case PathType::PATH_3D:
    pub_.publish(nav_2d_utils::pathToPath(to_publish));
    break;
  case PathType::NO_PATH:
    break;
  };
}

TwistPublisher::TwistPublisher(ros::NodeHandle& nh)
{
  std::string topic, publish_type;
  nh.param("twist_topic", topic, std::string("cmd_vel"));
  nh.param("twist_type", publish_type, std::string("Twist3D"));

  ros::NodeHandle global_nh;
  if (publish_type == "Twist2D")
  {
    twist_type_ = TwistType::TWIST_2D;
    pub_ = global_nh.advertise<nav_2d_msgs::Twist2D>(topic, 1);
  }
  else if (publish_type == "Twist2DStamped")
  {
    twist_type_ = TwistType::TWIST_2D_STAMPED;
    pub_ = global_nh.advertise<nav_2d_msgs::Twist2DStamped>(topic, 1);
  }
  else if (publish_type == "")
  {
    twist_type_ = TwistType::NO_TWIST;
  }
  else
  {
    if (publish_type != "Twist3D")
    {
      ROS_ERROR_NAMED("Locomotor", "Unknown twist_type \"%s\". Using Twist3D instead.", publish_type.c_str());
    }
    twist_type_ = TwistType::TWIST_3D;
    pub_ = global_nh.advertise<geometry_msgs::Twist>(topic, 1);
  }
}

void TwistPublisher::publishTwist(const nav_2d_msgs::Twist2DStamped& command)
{
  if (pub_.getNumSubscribers() == 0) return;
  switch (twist_type_)
  {
  case TwistType::TWIST_2D:
    pub_.publish(command.velocity);
    break;
  case TwistType::TWIST_2D_STAMPED:
    pub_.publish(command);
    break;
  case TwistType::TWIST_3D:
    pub_.publish(nav_2d_utils::twist2Dto3D(command.velocity));
    break;
  case TwistType::NO_TWIST:
    break;
  };
}

}  // namespace locomotor
