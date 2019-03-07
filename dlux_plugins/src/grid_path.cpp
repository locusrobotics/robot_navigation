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

#include <dlux_plugins/grid_path.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_core2/exceptions.h>
#include <limits>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dlux_plugins::GridPath, dlux_global_planner::Traceback)

namespace dlux_plugins
{

nav_2d_msgs::Path2D GridPath::getPath(const dlux_global_planner::PotentialGrid& potential_grid,
                                      const geometry_msgs::Pose2D& start, const geometry_msgs::Pose2D& goal,
                                      double& path_cost)
{
  const nav_grid::NavGridInfo& info = potential_grid.getInfo();
  nav_2d_msgs::Path2D path;
  path_cost = 0.0;

  unsigned int current_x = 0, current_y = 0;
  worldToGridBounded(info, start.x, start.y, current_x, current_y);

  // Add 0.5 to represent center of the cell
  geometry_msgs::Pose2D current;
  current.x = current_x + 0.5;
  current.y = current_y + 0.5;
  path.poses.push_back(current);

  unsigned int goal_index = potential_grid.getIndex(goal.x, goal.y);

  // Main Loop
  while (potential_grid.getIndex(current_x, current_y) != goal_index)
  {
    float min_val = std::numeric_limits<float>::max();
    unsigned int min_x = 0, min_y = 0;
    int distance_sq = 0;
    for (int xd = -1; xd <= 1; xd++)
    {
      if ((current_x == 0 && xd == -1) || (current_x == info.width - 1 && xd == 1)) continue;
      for (int yd = -1; yd <= 1; yd++)
      {
        if ((current_y == 0 && yd == -1) || (current_y == info.height - 1 && yd == 1)) continue;
        if (xd == 0 && yd == 0)
          continue;
        unsigned int x = current_x + xd, y = current_y + yd;
        int index = potential_grid.getIndex(x, y);
        if (potential_grid[index] < min_val)
        {
          min_val = potential_grid[index];
          min_x = x;
          min_y = y;
          distance_sq = abs(xd) + abs(yd);
        }
      }
    }
    if (distance_sq == 0)
      throw nav_core2::PlannerException("Reached dead end in traceback.");

    double distance;
    if (distance_sq == 1)
      distance = 0.5;
    else
      distance = M_SQRT1_2;  // sqrt(2)/2

    path_cost += distance * cost_interpreter_->getCost(current_x, current_y);

    // Move to the Min Neighbor
    current_x = min_x;
    current_y = min_y;

    // Add 0.5 to represent center of the cell
    current.x = current_x + 0.5;
    current.y = current_y + 0.5;
    path.poses.push_back(current);
    path_cost += distance * cost_interpreter_->getCost(current_x, current_y);
  }
  return mapPathToWorldPath(path, info);
}

}  // namespace dlux_plugins
