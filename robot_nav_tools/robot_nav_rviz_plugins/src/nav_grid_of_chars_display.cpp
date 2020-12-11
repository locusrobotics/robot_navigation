/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
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

#include <robot_nav_rviz_plugins/nav_grid_display.h>
#include <nav_2d_msgs/NavGridOfChars.h>
#include <nav_2d_msgs/NavGridOfCharsUpdate.h>
#include <nav_grid_pub_sub/nav_grid_subscriber.h>
#include <string>

namespace robot_nav_rviz_plugins
{
/**
 * @brief Displays a nav_grid of chars along the XY plane.
 *
 * Relatively straightforward since the char data is written directly into the `panel_data_` with no translation needed.
 */
class NavGridOfCharsDisplay: public NavGridDisplay
{
public:
  NavGridOfCharsDisplay()
    : NavGridDisplay(ros::message_traits::datatype<nav_2d_msgs::NavGridOfChars>())
    , sub_(panel_data_)  // set up the subscriber to update directly into `panel_data_`
  {
  }

  void onSubscribe(const std::string& topic) override
  {
    sub_.init(update_nh_, std::bind(&NavGridOfCharsDisplay::newDataCallback, this, std::placeholders::_1),
              topic, true, true);
  }

  void onUnsubscribe() override
  {
    sub_.deactivate();
  }

protected:
  void newDataCallback(const nav_core2::UIntBounds& bounds)
  {
    if (bounds.isEmpty())
    {
      return;
    }
    Q_EMIT mapUpdated(bounds);
  }

  nav_grid_pub_sub::NavGridSubscriber sub_;
};
}  // namespace robot_nav_rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_nav_rviz_plugins::NavGridOfCharsDisplay, rviz::Display)
