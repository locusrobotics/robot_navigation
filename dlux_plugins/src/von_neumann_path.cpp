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

#include <dlux_plugins/von_neumann_path.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_core2/exceptions.h>
#include <limits>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dlux_plugins::VonNeumannPath, dlux_global_planner::Traceback)

namespace dlux_plugins
{
nav_2d_msgs::Path2D VonNeumannPath::getPath(const dlux_global_planner::PotentialGrid& potential_grid,
                                            const geometry_msgs::Pose2D& start, const geometry_msgs::Pose2D& goal,
                                            double& path_cost)
{
  nav_2d_msgs::Path2D path;
  path_cost = 0.0;

  unsigned int current_x = 0, current_y = 0;
  worldToGridBounded(potential_grid.getInfo(), start.x, start.y, current_x, current_y);

  // Add 0.5 to represent center of the cell
  geometry_msgs::Pose2D current;
  current.x = current_x + 0.5;
  current.y = current_y + 0.5;
  path.poses.push_back(current);
  path_cost += cost_interpreter_->getCost(current_x, current_y);

  unsigned int goal_index = potential_grid.getIndex(goal.x, goal.y);

  // Main Loop
  while (potential_grid.getIndex(current_x, current_y) != goal_index)
  {
    float min_val = std::numeric_limits<float>::max();
    unsigned int min_x = 0, min_y = 0;
    unsigned int x, y;
    int index;

    // Check the four neighbors
    if (current_x > 0)
    {
      x = current_x - 1;
      y = current_y;
      index = potential_grid.getIndex(x, y);
      if (potential_grid[index] < min_val)
      {
        min_val = potential_grid[index];
        min_x = x;
        min_y = y;
      }
    }
    if (current_x < potential_grid.getWidth() - 1)
    {
      x = current_x + 1;
      y = current_y;
      index = potential_grid.getIndex(x, y);
      if (potential_grid[index] < min_val)
      {
        min_val = potential_grid[index];
        min_x = x;
        min_y = y;
      }
    }
    if (current_y > 0)
    {
      x = current_x;
      y = current_y - 1;
      index = potential_grid.getIndex(x, y);
      if (potential_grid[index] < min_val)
      {
        min_val = potential_grid[index];
        min_x = x;
        min_y = y;
      }
    }
    if (current_y < potential_grid.getHeight() - 1)
    {
      x = current_x;
      y = current_y + 1;
      index = potential_grid.getIndex(x, y);
      if (potential_grid[index] < min_val)
      {
        min_val = potential_grid[index];
        min_x = x;
        min_y = y;
      }
    }

    if (min_val == std::numeric_limits<float>::max())
      throw nav_core2::PlannerException("Reached dead end in traceback.");

    // Move to the Min Neighbor
    current_x = min_x;
    current_y = min_y;

    // Add 0.5 to represent center of the cell
    current.x = current_x + 0.5;
    current.y = current_y + 0.5;
    path.poses.push_back(current);
    path_cost += cost_interpreter_->getCost(current_x, current_y);
  }
  return mapPathToWorldPath(path, potential_grid.getInfo());
}

}  // namespace dlux_plugins
