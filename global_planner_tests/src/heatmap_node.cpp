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
#include <nav_grid/coordinate_conversion.h>
#include <pluginlib/class_loader.h>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using global_planner_tests::PoseList;
using global_planner_tests::groupCells;
using global_planner_tests::planExists;

/**
 * Heatmap Test
 * Runs a variation on checkValidPathCoverage() that prints a heatmap to the console with which cells in the map were
 * frequently unable to get paths planned to/from them.
 *
 * Useful for annoying corner cases where your planner fails. Definitely NOT speaking from experience.
 *
 * See README.md for more details.
 */
int main(int argc, char** argv)
{
  // *************** Initialize the Planner ****************************************************************************
  ros::init(argc, argv, "plan");
  ros::NodeHandle private_nh("~");
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();

  std::string map_filename;
  private_nh.param("map_filename", map_filename, std::string("package://global_planner_tests/maps/smile.png"));
  std::shared_ptr<global_planner_tests::EasyCostmap> global_costmap =
    std::make_shared<global_planner_tests::EasyCostmap>(map_filename);

  pluginlib::ClassLoader<nav_core2::GlobalPlanner> global_planner_loader("nav_core2", "nav_core2::GlobalPlanner");

  std::string planner_name;
  private_nh.param("global_planner", planner_name, std::string("dlux_global_planner::DluxGlobalPlanner"));
  boost::shared_ptr<nav_core2::GlobalPlanner> global_planner = global_planner_loader.createInstance(planner_name);
  global_planner->initialize(private_nh, global_planner_loader.getName(planner_name), tf, global_costmap);

  // *************** Get a list of every free cell**********************************************************************
  PoseList free_cells, occupied_cells;
  groupCells(*global_costmap, free_cells, occupied_cells);

  int saved_pct = -1;
  unsigned int n = free_cells.size();
  if (n == 0)
  {
    return 0;
  }

  // *************** Attempt planning between every pair of free cells**************************************************
  int passing_plans = 0, total_plans = 0;
  std::vector<unsigned int> failures(n, 0);

  for (unsigned int j = 0; j < n; j++)
  {
    for (unsigned int i = 0; i < n; i++)
    {
      // Print progress updates
      {
        int pct = (total_plans / static_cast<float>(n * n)) * 20;
        if (pct != saved_pct)
        {
          float passing = 0.0;
          if (total_plans > 0)
          {
            passing = 100.0 * passing_plans / total_plans;
          }
          ROS_INFO("%d%% complete. %d tests, %.0f%% passing", pct * 100 / 20, total_plans, passing);
          saved_pct = pct;
        }
      }
      total_plans = total_plans + 1;

      if (planExists(*global_planner, free_cells[i], free_cells[j]))
      {
        passing_plans = passing_plans + 1;
      }
      else
      {
        // Mark the start and end cells of the planning failure
        failures[i] += 1;
        failures[j] += 1;
      }
      if (!ros::ok()) return EXIT_FAILURE;
    }
  }

  // *************** Construct/Print an ASCII map of the failures*******************************************************
  const nav_grid::NavGridInfo& info = global_costmap->getInfo();
  std::vector<char> mmap(info.width * info.height, '.');
  unsigned int highest = *std::max_element(failures.begin(), failures.end());

  for (unsigned int i = 0; i < n; i++)
  {
    unsigned int x, y;
    worldToGridBounded(info, free_cells[i].pose.x, free_cells[i].pose.y, x, y);
    int ct = failures[i];
    if (ct == 0)
    {
      mmap[ global_costmap->getIndex(x, y) ] = ' ';
    }
    else
    {
      double f = static_cast<double>(ct) / highest;
      int m = static_cast<int>(f * 9);
      mmap[ global_costmap->getIndex(x, y) ] = '0' + m;
    }
  }
  for (int j = info.height - 1; j >= 0; j--)
  {
    for (unsigned int i = 0; i < info.width; i++)
    {
      printf("%c", mmap[ global_costmap->getIndex(i, j) ]);
    }
    printf("\n");
  }

  ROS_INFO("%d/%d valid plans found.", passing_plans, total_plans);
  return EXIT_SUCCESS;
}
