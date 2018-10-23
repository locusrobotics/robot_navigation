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

#include <dlux_plugins/dijkstra.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_core2/exceptions.h>
#include <dlux_global_planner/kernel_function.h>
#include <pluginlib/class_list_macros.h>
#include <queue>

PLUGINLIB_EXPORT_CLASS(dlux_plugins::Dijkstra, dlux_global_planner::PotentialCalculator)

namespace dlux_plugins
{
unsigned int Dijkstra::updatePotentials(dlux_global_planner::PotentialGrid& potential_grid,
                                        const geometry_msgs::Pose2D& start, const geometry_msgs::Pose2D& goal)
{
  const nav_grid::NavGridInfo& info = potential_grid.getInfo();
  queue_ = std::queue<nav_grid::Index>();
  potential_grid.reset();

  nav_grid::Index goal_i;
  worldToGridBounded(info, goal.x, goal.y, goal_i.x, goal_i.y);
  queue_.push(goal_i);
  potential_grid.setValue(goal_i, 0.0);

  nav_grid::Index start_i;
  worldToGridBounded(info, start.x, start.y, start_i.x, start_i.y);
  unsigned int c = 0;

  while (!queue_.empty())
  {
    nav_grid::Index i = queue_.front();
    queue_.pop();
    c++;

    if (i == start_i) return c;

    if (i.x > 0)
      add(potential_grid, nav_grid::Index(i.x - 1, i.y));
    if (i.y > 0)
      add(potential_grid, nav_grid::Index(i.x, i.y - 1));

    if (i.x < info.width - 1)
      add(potential_grid, nav_grid::Index(i.x + 1, i.y));
    if (i.y < info.height - 1)
      add(potential_grid, nav_grid::Index(i.x, i.y + 1));
  }

  throw nav_core2::NoGlobalPathException();
}

void Dijkstra::add(dlux_global_planner::PotentialGrid& potential_grid, nav_grid::Index next_index)
{
  if (potential_grid(next_index.x, next_index.y) < dlux_global_planner::HIGH_POTENTIAL)
    return;

  float cost = cost_interpreter_->getCost(next_index.x, next_index.y);
  if (cost_interpreter_->isLethal(cost))
    return;
  potential_grid.setValue(next_index,
                          dlux_global_planner::calculateKernel(potential_grid, cost, next_index.x, next_index.y));
  queue_.push(next_index);
}

}  // namespace dlux_plugins
