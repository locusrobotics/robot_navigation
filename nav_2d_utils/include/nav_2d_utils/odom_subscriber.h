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

#ifndef NAV_2D_UTILS_ODOM_SUBSCRIBER_H
#define NAV_2D_UTILS_ODOM_SUBSCRIBER_H

#include <ros/ros.h>
#include <nav_2d_utils/conversions.h>
#include <nav_msgs/Odometry.h>
#include <nav_2d_msgs/Twist2DStamped.h>
#include <boost/thread/mutex.hpp>
#include <string>

namespace nav_2d_utils
{

/**
 * @class OdomSubscriber
 * Wrapper for some common odometry operations. Subscribes to the topic with a mutex.
 */
class OdomSubscriber
{
public:
  /**
   * @brief Constructor that subscribes to an Odometry topic
   *
   * @param nh NodeHandle for creating subscriber
   * @param default_topic Name of the topic that will be loaded of the odom_topic param is not set.
   */
  explicit OdomSubscriber(ros::NodeHandle& nh, std::string default_topic = "odom")
  {
    std::string odom_topic;
    nh.param("odom_topic", odom_topic, default_topic);
    odom_sub_ = nh.subscribe<nav_msgs::Odometry>(odom_topic, 1, boost::bind(&OdomSubscriber::odomCallback, this, _1));
  }

  inline nav_2d_msgs::Twist2D getTwist() { return odom_vel_.velocity; }
  inline nav_2d_msgs::Twist2DStamped getTwistStamped() { return odom_vel_; }

protected:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    ROS_INFO_ONCE("odom received!");
    boost::mutex::scoped_lock lock(odom_mutex_);
    odom_vel_.header = msg->header;
    odom_vel_.velocity = twist3Dto2D(msg->twist.twist);
  }

  ros::Subscriber odom_sub_;
  nav_2d_msgs::Twist2DStamped odom_vel_;
  boost::mutex odom_mutex_;
};

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS_ODOM_SUBSCRIBER_H
