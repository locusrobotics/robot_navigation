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
#include <ros/ros.h>
#include <pluginlib/class_loader.h>
#include <dwb_local_planner/debug_dwb_local_planner.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan_node");
  ros::NodeHandle private_nh("~");

  dwb_local_planner::DebugDWBLocalPlanner planner;
  ROS_INFO("Plan Node");

  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tf2(*tf);

  pluginlib::ClassLoader<nav_core2::Costmap> costmap_loader("nav_core2", "nav_core2::Costmap");
  std::string costmap_class;
  private_nh.param("local_costmap_class", costmap_class, std::string("nav_core_adapter::CostmapAdapter"));
  nav_core2::Costmap::Ptr costmap = costmap_loader.createUniqueInstance(costmap_class);
  costmap->initialize(private_nh, "costmap", tf);

  planner.initialize(private_nh, "dwb_local_planner", tf, costmap);
  ros::spin();
}
