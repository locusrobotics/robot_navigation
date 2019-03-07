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
#include <nav_grid/coordinate_conversion.h>
#include <nav_core2/exceptions.h>
#include <string>

namespace global_planner_tests
{
void groupCells(const nav_core2::Costmap& costmap, PoseList& free_cells, PoseList& occupied_cells, bool include_edges)
{
  const nav_grid::NavGridInfo& info = costmap.getInfo();
  int start = include_edges ? 0 : 1;
  unsigned int x_max = include_edges ? info.width : info.width - 1;
  unsigned int y_max = include_edges ? info.height : info.height - 1;
  nav_2d_msgs::Pose2DStamped pose;
  pose.header.frame_id = info.frame_id;

  for (unsigned int i = start; i < x_max; i++)
  {
    for (unsigned int j = start; j < y_max; j++)
    {
      gridToWorld(info, i, j, pose.pose.x, pose.pose.y);
      unsigned char cost = costmap(i, j);
      if (cost < costmap.INSCRIBED_INFLATED_OBSTACLE)
      {
        free_cells.push_back(pose);
      }
      else if (cost != costmap.NO_INFORMATION)
      {
        occupied_cells.push_back(pose);
      }
    }
  }
}

PoseList createPosesOutsideCostmap(const nav_core2::Costmap& costmap)
{
  PoseList poses;
  double ox = costmap.getOriginX(), oy = costmap.getOriginY();
  double res = costmap.getResolution();
  double w = costmap.getWidth() * res, h = costmap.getHeight() * res;
  double offset = 3.0 * res;  // arbitrary distance outside costmap
  nav_2d_msgs::Pose2DStamped pose;
  pose.header.frame_id = costmap.getFrameId();
  pose.pose.x = ox - offset;
  pose.pose.y = oy - offset;
  poses.push_back(pose);
  pose.pose.y = oy + h + offset;
  poses.push_back(pose);
  pose.pose.x = ox + w + offset;
  poses.push_back(pose);
  pose.pose.y = oy - offset;
  poses.push_back(pose);
  return poses;
}

PoseList subsetPoseList(const PoseList& cells, unsigned int num_cells)
{
  PoseList subset;
  if (cells.size() == 0 || num_cells == 0)
  {
    return subset;
  }

  if (cells.size() <= num_cells)
  {
    unsigned int i = 0;
    while (subset.size() < num_cells)
    {
      subset.push_back(cells[i++]);
      if (i >= cells.size())
      {
        i = 0;
      }
    }
  }
  else
  {
    double inc = cells.size() / static_cast<double>(num_cells);
    double i = 0.0;
    while (subset.size() < num_cells)
    {
      subset.push_back(cells[static_cast<int>(i)]);
      i += inc;
    }
  }

  return subset;
}

bool planExists(nav_core2::GlobalPlanner& planner, nav_2d_msgs::Pose2DStamped start, nav_2d_msgs::Pose2DStamped goal)
{
  try
  {
    planner.makePlan(start, goal);
    return true;
  }
  catch (nav_core2::PlannerException& e)
  {
    return false;
  }
}

bool checkValidPathCoverage(nav_core2::GlobalPlanner& planner, const PoseList& free_cells,
                            bool verbose, bool quit_early)
{
  int passing_plans = 0, total_plans = 0;
  for (nav_2d_msgs::Pose2DStamped start_pose : free_cells)
  {
    for (nav_2d_msgs::Pose2DStamped goal_pose : free_cells)
    {
      total_plans++;

      if (planExists(planner, start_pose, goal_pose))
      {
        passing_plans++;
      }
      else if (quit_early)
      {
        ROS_INFO("Failed to find a path between %.2f %.2f and %.2f %.2f",
                 start_pose.pose.x, start_pose.pose.y, goal_pose.pose.x, goal_pose.pose.y);
        return false;
      }
      if (!ros::ok()) return false;
    }
  }

  if (verbose)
    ROS_INFO("%d/%d valid plans found.", passing_plans, total_plans);

  return passing_plans == total_plans;
}

bool checkOccupiedPathCoverage(nav_core2::GlobalPlanner& planner,
                               const PoseList& start_cells, const PoseList& goal_cells,
                               const std::string& test_name,
                               bool check_exception_type, bool verbose, bool quit_early, bool invalid_starts)
{
  int passing_plans = 0, total_plans = 0;

  for (nav_2d_msgs::Pose2DStamped start_pose : start_cells)
  {
    for (nav_2d_msgs::Pose2DStamped goal_pose : goal_cells)
    {
      total_plans++;
      bool valid = true;
      try
      {
        planner.makePlan(start_pose, goal_pose);
        // Plan should throw exception. If it doesn't, this test fails
        if (quit_early)
        {
          ROS_INFO("Found an unexpected valid %s path between %.2f %.2f and %.2f %.2f", test_name.c_str(),
                   start_pose.pose.x, start_pose.pose.y, goal_pose.pose.x, goal_pose.pose.y);
          return false;
        }
        valid = false;
      }
      catch (nav_core2::OccupiedStartException& e)
      {
        if (check_exception_type)
          valid = invalid_starts;
      }
      catch (nav_core2::OccupiedGoalException& e)
      {
        if (check_exception_type)
          valid = !invalid_starts;
      }
      catch (nav_core2::PlannerException& e)
      {
        valid = !check_exception_type;
      }
      if (valid)
      {
        passing_plans++;
      }
      else if (quit_early)
      {
        ROS_INFO("An unexpected exception was thrown attempting a %s path between %.2f %.2f and %.2f %.2f",
                 test_name.c_str(), start_pose.pose.x, start_pose.pose.y, goal_pose.pose.x, goal_pose.pose.y);
        return false;
      }
    }
  }

  if (verbose)
    ROS_INFO("%d/%d %s plans correctly aborted.", passing_plans, total_plans, test_name.c_str());
  return passing_plans == total_plans;
}

bool checkOutOfBoundsPathCoverage(nav_core2::GlobalPlanner& planner,
                                 const PoseList& start_cells, const PoseList& goal_cells,
                                 const std::string& test_name,
                                 bool check_exception_type, bool verbose, bool quit_early, bool invalid_starts)
{
  int passing_plans = 0, total_plans = 0;

  for (nav_2d_msgs::Pose2DStamped start_pose : start_cells)
  {
    for (nav_2d_msgs::Pose2DStamped goal_pose : goal_cells)
    {
      total_plans++;
      bool valid = true;
      try
      {
        planner.makePlan(start_pose, goal_pose);
        // Plan should throw exception. If it doesn't, this test fails
        if (quit_early)
        {
          ROS_INFO("Found an unexpected valid %s path between %.2f %.2f and %.2f %.2f", test_name.c_str(),
                   start_pose.pose.x, start_pose.pose.y, goal_pose.pose.x, goal_pose.pose.y);
          return false;
        }
        valid = false;
      }
      catch (nav_core2::StartBoundsException& e)
      {
        if (check_exception_type)
          valid = invalid_starts;
      }
      catch (nav_core2::GoalBoundsException& e)
      {
        if (check_exception_type)
          valid = !invalid_starts;
      }
      catch (nav_core2::PlannerException& e)
      {
        valid = !check_exception_type;
      }
      if (valid)
      {
        passing_plans++;
      }
      else if (quit_early)
      {
        ROS_INFO("An unexpected exception was thrown attempting a %s path between %.2f %.2f and %.2f %.2f",
                 test_name.c_str(), start_pose.pose.x, start_pose.pose.y, goal_pose.pose.x, goal_pose.pose.y);
        return false;
      }
    }
  }

  if (verbose)
    ROS_INFO("%d/%d %s plans correctly aborted.", passing_plans, total_plans, test_name.c_str());
  return passing_plans == total_plans;
}


bool hasCompleteCoverage(nav_core2::GlobalPlanner& planner, const nav_core2::Costmap& costmap, int max_failure_cases,
                         bool check_exception_type, bool verbose, bool quit_early)
{
  PoseList free_cells, occupied_cells;
  groupCells(costmap, free_cells, occupied_cells);

  bool ret = checkValidPathCoverage(planner, free_cells, verbose, quit_early);
  if (!ret && quit_early)
  {
    return ret;
  }
  if (max_failure_cases >= 0)
  {
    free_cells = subsetPoseList(free_cells, max_failure_cases);
    occupied_cells = subsetPoseList(occupied_cells, max_failure_cases);
  }

  ret = checkOccupiedPathCoverage(planner, free_cells, occupied_cells, "Free->Occupied",
                                  check_exception_type, verbose, quit_early, false) && ret;
  if (!ret && quit_early)
  {
    return ret;
  }

  ret = checkOccupiedPathCoverage(planner, occupied_cells, free_cells, "Occupied->Free",
                                  check_exception_type, verbose, quit_early, true) && ret;
  if (!ret && quit_early)
  {
    return ret;
  }

  ret = checkOccupiedPathCoverage(planner, occupied_cells, occupied_cells, "Occupied->Occupied",
                                  false, verbose, quit_early) && ret;
  if (!ret && quit_early)
  {
    return ret;
  }

  PoseList out_of_bounds = createPosesOutsideCostmap(costmap);
  ret = checkOutOfBoundsPathCoverage(planner, free_cells, out_of_bounds, "Free->OutOfBounds",
                                     check_exception_type, verbose, quit_early, false) && ret;
  if (!ret && quit_early)
  {
    return ret;
  }

  ret = checkOutOfBoundsPathCoverage(planner, out_of_bounds, free_cells, "OutOfBounds->Free",
                                     check_exception_type, verbose, quit_early, true) && ret;
  if (!ret && quit_early)
  {
    return ret;
  }

  ret = checkOutOfBoundsPathCoverage(planner, out_of_bounds, out_of_bounds, "CompletelyOutOfBounds",
                                     false, verbose, quit_early) && ret;
  if (!ret && quit_early)
  {
    return ret;
  }

  return ret;
}

bool hasNoPaths(nav_core2::GlobalPlanner& planner, const nav_core2::Costmap& costmap,
                bool check_exception_type, bool verbose, bool quit_early)
{
  PoseList free_cells, occupied_cells;
  groupCells(costmap, free_cells, occupied_cells);
  int passing_plans = 0, total_plans = 0;

  unsigned int n = free_cells.size();
  for (unsigned int i = 0; i < n; i++)
  {
    for (unsigned int j = 0; j < n; j++)
    {
      if (i == j) continue;
      total_plans++;
      bool valid;
      try
      {
        planner.makePlan(free_cells[i], free_cells[j]);
        // Plan should throw exception. If it doesn't, this test fails
        if (quit_early)
        {
          ROS_INFO("Found an unexpected valid path between %.2f %.2f and %.2f %.2f",
                   free_cells[i].pose.x, free_cells[i].pose.y, free_cells[j].pose.x, free_cells[j].pose.y);
          return false;
        }
        valid = false;
      }
      catch (nav_core2::NoGlobalPathException& e)
      {
        valid = true;
      }
      catch (nav_core2::PlannerException& e)
      {
        valid = !check_exception_type;
      }
      if (valid)
      {
        passing_plans++;
      }
      else if (quit_early)
      {
        ROS_INFO("An unexpected exception was thrown attempting to plan between %.2f %.2f and %.2f %.2f",
                 free_cells[i].pose.x, free_cells[i].pose.y, free_cells[j].pose.x, free_cells[j].pose.y);
        return false;
      }
    }
  }

  if (verbose)
    ROS_INFO("%d/%d correctly aborted for having no path.", passing_plans, total_plans);
  return passing_plans == total_plans;
}

}  // namespace global_planner_tests
