
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


TEST(GlobalPlanner, DijkstraVonNeumann)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "DijkstraVonNeumann";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::Dijkstra");
  nh.setParam("planner/traceback", "dlux_plugins::VonNeumannPath");
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, DijkstraGrid)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "DijkstraGrid";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::Dijkstra");
  nh.setParam("planner/traceback", "dlux_plugins::GridPath");
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, DijkstraGradientStep)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "DijkstraGradientStep";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::Dijkstra");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/grid_step_near_high", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, DijkstraGradient)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "DijkstraGradient";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::Dijkstra");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/grid_step_near_high", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarVonNeumannManK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarVonNeumannManK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::VonNeumannPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarVonNeumannK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarVonNeumannK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::VonNeumannPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarVonNeumannMan)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarVonNeumannMan";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::VonNeumannPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarVonNeumann)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarVonNeumann";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::VonNeumannPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarVonNeumannManThreshK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarVonNeumannManThreshK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::VonNeumannPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarVonNeumannThreshK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarVonNeumannThreshK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::VonNeumannPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarVonNeumannManThresh)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarVonNeumannManThresh";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::VonNeumannPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarVonNeumannThresh)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarVonNeumannThresh";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::VonNeumannPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGridManK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGridManK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GridPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGridK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGridK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GridPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGridMan)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGridMan";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GridPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGrid)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGrid";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GridPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGridManThreshK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGridManThreshK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GridPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGridThreshK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGridThreshK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GridPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGridManThresh)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGridManThresh";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GridPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGridThresh)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGridThresh";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GridPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientStepManK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientStepManK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", true);
  nh.setParam("planner/grid_step_near_high", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientManK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientManK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", true);
  nh.setParam("planner/grid_step_near_high", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientStepK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientStepK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", false);
  nh.setParam("planner/grid_step_near_high", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", false);
  nh.setParam("planner/grid_step_near_high", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientStepMan)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientStepMan";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", true);
  nh.setParam("planner/grid_step_near_high", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientMan)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientMan";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", true);
  nh.setParam("planner/grid_step_near_high", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientStep)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientStep";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", false);
  nh.setParam("planner/grid_step_near_high", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradient)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradient";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 0.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", false);
  nh.setParam("planner/grid_step_near_high", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientStepManThreshK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientStepManThreshK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", true);
  nh.setParam("planner/grid_step_near_high", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientManThreshK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientManThreshK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", true);
  nh.setParam("planner/grid_step_near_high", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientStepThreshK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientStepThreshK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", false);
  nh.setParam("planner/grid_step_near_high", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientThreshK)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientThreshK";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", true);
  nh.setParam("planner/manhattan_heuristic", false);
  nh.setParam("planner/grid_step_near_high", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientStepManThresh)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientStepManThresh";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", true);
  nh.setParam("planner/grid_step_near_high", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientManThresh)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientManThresh";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", true);
  nh.setParam("planner/grid_step_near_high", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientStepThresh)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientStepThresh";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", false);
  nh.setParam("planner/grid_step_near_high", true);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


TEST(GlobalPlanner, AStarGradientThresh)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  DluxGlobalPlanner planner;
  std::string ns = "AStarGradientThresh";

  ros::NodeHandle nh("~/" + ns);
  nh.setParam("planner/potential_calculator", "dlux_plugins::AStar");
  nh.setParam("planner/traceback", "dlux_plugins::GradientPath");
  nh.setParam("planner/minimum_requeue_change", 1.0);
  nh.setParam("planner/use_kernel", false);
  nh.setParam("planner/manhattan_heuristic", false);
  nh.setParam("planner/grid_step_near_high", false);
  EXPECT_TRUE(many_map_test_suite(planner, tf, ns));
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
