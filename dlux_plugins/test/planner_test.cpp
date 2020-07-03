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
#include <global_planner_tests/many_map_test_suite.h>
#include <dlux_global_planner/dlux_global_planner.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>

using global_planner_tests::many_map_test_suite;
using dlux_global_planner::DluxGlobalPlanner;

void dlux_test(std::string ns, std::string potential_calculator = "", std::string traceback = "")
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;

  ros::NodeHandle nh("~/" + ns);
  if (potential_calculator.length() > 0)
    nh.setParam("potential_calculator", potential_calculator);
  if (traceback.length() > 0)
    nh.setParam("traceback", traceback);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns)) << potential_calculator << " " + traceback;
}

TEST(GlobalPlanner, AStarVon)
{
  dlux_test("AStarVon", "dlux_plugins::AStar", "dlux_plugins::VonNeumannPath");
}

TEST(GlobalPlanner, AStarGrid)
{
  dlux_test("AStarGrid", "dlux_plugins::AStar", "dlux_plugins::GridPath");
}

TEST(GlobalPlanner, AStarGradient)
{
  dlux_test("AStarGradient", "dlux_plugins::AStar", "dlux_plugins::GradientPath");
}

TEST(GlobalPlanner, DijkstraVon)
{
  dlux_test("DijkstraVon", "dlux_plugins::Dijkstra", "dlux_plugins::VonNeumannPath");
}

TEST(GlobalPlanner, DijkstraGrid)
{
  dlux_test("DijkstraGrid", "dlux_plugins::Dijkstra", "dlux_plugins::GridPath");
}

TEST(GlobalPlanner, DijkstraGradient)
{
  dlux_test("DijkstraGradient", "dlux_plugins::Dijkstra", "dlux_plugins::GradientPath");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
