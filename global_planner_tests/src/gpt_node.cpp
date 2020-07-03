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
#include <global_planner_tests/global_planner_tests.h>
#include <global_planner_tests/easy_costmap.h>
#include <pluginlib/class_loader.h>
#include <memory>
#include <string>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "plan");
  ros::NodeHandle private_nh("~");
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();

  std::string map_filename;
  private_nh.param("map_filename", map_filename, std::string("package://global_planner_tests/maps/smile.png"));
  nav_core2::Costmap::Ptr global_costmap = std::make_shared<global_planner_tests::EasyCostmap>(map_filename);

  pluginlib::ClassLoader<nav_core2::GlobalPlanner> global_planner_loader("nav_core2", "nav_core2::GlobalPlanner");

  std::string planner_name;
  private_nh.param("global_planner", planner_name, std::string("dlux_global_planner::DluxGlobalPlanner"));
  boost::shared_ptr<nav_core2::GlobalPlanner> global_planner = global_planner_loader.createInstance(planner_name);
  global_planner->initialize(private_nh, global_planner_loader.getName(planner_name), tf, global_costmap);

  if (global_planner_tests::hasCompleteCoverage(*global_planner, *global_costmap, 10, true, true, false))
  {
    printf("PASSES\n");
  }
  else
  {
    printf("FAILS\n");
  }

  return (0);
}
