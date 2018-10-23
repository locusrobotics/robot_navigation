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

#ifndef DLUX_PLUGINS_ASTAR_H
#define DLUX_PLUGINS_ASTAR_H

#include <dlux_global_planner/potential_calculator.h>
#include <queue>
#include <vector>

namespace dlux_plugins
{

/**
 * @class AStar
 * @brief Potential calculator that explores using a distance heuristic (A*) but not the kernel function
 */
class AStar : public dlux_global_planner::PotentialCalculator
{
public:
  // Main PotentialCalculator interface
  void initialize(ros::NodeHandle& private_nh, nav_core2::Costmap::Ptr costmap,
                  dlux_global_planner::CostInterpreter::Ptr cost_interpreter) override;
  unsigned int updatePotentials(dlux_global_planner::PotentialGrid& potential_grid,
                                const geometry_msgs::Pose2D& start, const geometry_msgs::Pose2D& goal) override;
protected:
  /**
   * @brief Calculate the potential for index if not calculated already
   *
   * @param potential_grid Potential grid
   * @param prev_potential Potential of the previous cell
   * @param index Coordinates of cell to calculate
   * @param start_index Coordinates of start cell (for heuristic calculation)
   */
  void add(dlux_global_planner::PotentialGrid& potential_grid, double prev_potential,
           const nav_grid::Index& index, const nav_grid::Index& start_index);

  /**
   * @brief Calculate the heuristic value for a particular cell
   *
   * @param index Coordinates of cell to calculate
   * @param start_index Coordinates of start cell
   */
  float getHeuristicValue(const nav_grid::Index& index, const nav_grid::Index& start_index) const;

  /**
   * @brief Helper Class for sorting indexes by their heuristic
   */
  struct QueueEntry
  {
  public:
    QueueEntry(nav_grid::Index index, float heuristic) : i(index), cost(heuristic) {}
    nav_grid::Index i;
    float cost;
  };

  /**
   * @brief Comparator for sorting the QueueEntrys
   */
  struct QueueEntryComparator
  {
    bool operator()(const QueueEntry& a, const QueueEntry& b) const
    {
      return a.cost > b.cost;
    }
  };

  // Indexes sorted by heuristic
  using AStarQueue = std::priority_queue<QueueEntry, std::vector<QueueEntry>, QueueEntryComparator>;
  AStarQueue queue_;
  bool manhattan_heuristic_;
  bool use_kernel_;
  double minimum_requeue_change_;
};
}  // namespace dlux_plugins

#endif  // DLUX_PLUGINS_ASTAR_H
