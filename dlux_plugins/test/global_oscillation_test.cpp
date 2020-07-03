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
#include <nav_core2/exceptions.h>
#include <global_planner_tests/easy_costmap.h>
#include <global_planner_tests/global_planner_tests.h>
#include <dlux_global_planner/dlux_global_planner.h>
#include <nav_grid/coordinate_conversion.h>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

const char map_path[] = "package://dlux_plugins/test/robert_frost.png";

int barrier_x = 5;
int barrier_y0 = 1;
int barrier_y1 = 4;

void setBarrier(nav_core2::Costmap& costmap, const unsigned char value)
{
  for (int y=barrier_y0; y <= barrier_y1; y++)
  {
    costmap.setCost(barrier_x, y, value);
  }
}

bool pathsEqual(const nav_2d_msgs::Path2D& path_a, const nav_2d_msgs::Path2D& path_b)
{
  if (path_a.header.frame_id != path_b.header.frame_id || path_a.poses.size() != path_b.poses.size())
  {
    return false;
  }
  for (unsigned int i=0; i < path_a.poses.size(); i++)
  {
    if (path_a.poses[i].x != path_b.poses[i].x || path_a.poses[i].y != path_b.poses[i].y
       || path_a.poses[i].theta != path_b.poses[i].theta)
    {
      return false;
    }
  }
  return true;
}

/**
 * @brief Check to see whether path caching is working as expected
 *
 * @param ns Namespace for patermeters
 * @param expect_different Whether we expected the resulting paths to be different
 * @param path_caching Value for path_caching parameter
 * @param improvement_threshold Value for improvement_threshold parameter
 */
void replanning_test(const std::string& ns, bool expect_different,
                     bool path_caching = false, double improvement_threshold = -1.0)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  std::shared_ptr<global_planner_tests::EasyCostmap> easy_costmap =
        std::make_shared<global_planner_tests::EasyCostmap>(std::string(map_path), 1.0);

  dlux_global_planner::DluxGlobalPlanner global_planner;
  ros::NodeHandle nh("~"), private_nh("~/" + ns);
  private_nh.setParam("path_caching", path_caching);
  private_nh.setParam("improvement_threshold", improvement_threshold);
  private_nh.setParam("potential_calculator", "dlux_plugins::Dijkstra");
  private_nh.setParam("traceback", "dlux_plugins::GridPath");
  private_nh.setParam("print_statistics", true);
  global_planner.initialize(nh, ns, tf, easy_costmap);

  nav_core2::Costmap& costmap = *easy_costmap;
  const nav_grid::NavGridInfo& info = costmap.getInfo();

  nav_2d_msgs::Pose2DStamped start;
  nav_2d_msgs::Pose2DStamped goal;

  start.header.frame_id = info.frame_id;
  gridToWorld(info, 2, 5, start.pose.x, start.pose.y);
  goal.header.frame_id = info.frame_id;
  gridToWorld(info, 17, 5, goal.pose.x, goal.pose.y);

  // Block the lower path
  setBarrier(costmap, 254);

  // Get a path (takes upper route)
  nav_2d_msgs::Path2D first_path = global_planner.makePlan(start, goal);

  // Clear the barrier and open up shorter route
  setBarrier(costmap, 0);

  // Plan again
  nav_2d_msgs::Path2D second_path = global_planner.makePlan(start, goal);

  if (expect_different)
  {
    EXPECT_FALSE(pathsEqual(first_path, second_path));
  }
  else
  {
    EXPECT_TRUE(pathsEqual(first_path, second_path));
  }
}

TEST(GlobalPlannerReplanning, no_cache)
{
  // With no path caching, we expect two different paths
  replanning_test("no_cache", true);
}

TEST(GlobalPlannerReplanning, any_cache)
{
  // Once we turn on path caching, we expect they will be the same
  replanning_test("any_cache", false, true);
}

TEST(GlobalPlannerReplanning, improve_cache)
{
  // If the new path is better, we expect they will be different
  replanning_test("improve_cache", true, true, 0.0);
}

TEST(GlobalPlannerReplanning, big_improve)
{
  // The new path needs to be 1000pts better, so we expect they will be the same
  replanning_test("big_improve", false, true, 1000.0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "global_oscillation_test");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
