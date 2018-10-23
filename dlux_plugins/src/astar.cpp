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

#include <dlux_plugins/astar.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_core2/exceptions.h>
#include <dlux_global_planner/kernel_function.h>
#include <math.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dlux_plugins::AStar, dlux_global_planner::PotentialCalculator)

namespace dlux_plugins
{
void AStar::initialize(ros::NodeHandle& private_nh, nav_core2::Costmap::Ptr costmap,
                       dlux_global_planner::CostInterpreter::Ptr cost_interpreter)
{
  cost_interpreter_ = cost_interpreter;
  private_nh.param("manhattan_heuristic", manhattan_heuristic_, false);
  private_nh.param("use_kernel", use_kernel_, true);
  private_nh.param("minimum_requeue_change", minimum_requeue_change_, 1.0);
}

unsigned int AStar::updatePotentials(dlux_global_planner::PotentialGrid& potential_grid,
                                     const geometry_msgs::Pose2D& start, const geometry_msgs::Pose2D& goal)
{
  const nav_grid::NavGridInfo& info = potential_grid.getInfo();
  queue_ = AStarQueue();
  potential_grid.reset();

  nav_grid::Index goal_i;
  worldToGridBounded(info, goal.x, goal.y, goal_i.x, goal_i.y);
  queue_.push(QueueEntry(goal_i, 0.0));
  potential_grid.setValue(goal_i, 0.0);

  // bounds check done in dlux_global_planner
  nav_grid::Index start_i;
  worldToGridBounded(info, start.x, start.y, start_i.x, start_i.y);

  if (potential_grid.getWidth() == 0 || potential_grid.getHeight() == 0)
  {
    return 0;
  }

  unsigned int width_bound = potential_grid.getWidth() - 1, height_bound = potential_grid.getHeight() - 1;
  unsigned int c = 0;

  while (queue_.size() > 0)
  {
    QueueEntry top = queue_.top();
    queue_.pop();
    c++;

    nav_grid::Index i = top.i;
    if (i == start_i) return c;

    double prev_potential = potential_grid(i);

    if (i.x < width_bound)
        add(potential_grid, prev_potential, nav_grid::Index(i.x + 1, i.y), start_i);
    if (i.x > 0)
        add(potential_grid, prev_potential, nav_grid::Index(i.x - 1, i.y), start_i);
    if (i.y < height_bound)
        add(potential_grid, prev_potential, nav_grid::Index(i.x, i.y + 1), start_i);
    if (i.y > 0)
        add(potential_grid, prev_potential, nav_grid::Index(i.x, i.y - 1), start_i);
  }

  throw nav_core2::NoGlobalPathException();
}

void AStar::add(dlux_global_planner::PotentialGrid& potential_grid, double prev_potential,
                const nav_grid::Index& index, const nav_grid::Index& start_index)
{
  float cost = cost_interpreter_->getCost(index.x, index.y);
  if (cost_interpreter_->isLethal(cost))
    return;

  float new_potential;
  if (use_kernel_)
  {
    new_potential = dlux_global_planner::calculateKernel(potential_grid, cost, index.x, index.y);
  }
  else
  {
    new_potential = prev_potential + cost;
  }

  if (new_potential >= potential_grid(index) || potential_grid(index) - new_potential < minimum_requeue_change_)
    return;

  potential_grid.setValue(index, new_potential);
  queue_.push(QueueEntry(index, new_potential + getHeuristicValue(index, start_index)));
}

inline unsigned int uintDiff(const unsigned int a, const unsigned int b)
{
  return (a > b) ? a - b : b - a;
}

float AStar::getHeuristicValue(const nav_grid::Index& index, const nav_grid::Index& start_index) const
{
  unsigned int dx = uintDiff(start_index.x, index.x);
  unsigned int dy = uintDiff(start_index.y, index.y);
  float distance;
  if (manhattan_heuristic_)
    distance = static_cast<float>(dx + dy);
  else
    distance = hypot(dx, dy);
  return distance * cost_interpreter_->getNeutralCost();
}

}  // namespace dlux_plugins
