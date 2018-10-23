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

#ifndef DLUX_PLUGINS_DIJKSTRA_H
#define DLUX_PLUGINS_DIJKSTRA_H

#include <dlux_global_planner/potential_calculator.h>
#include <queue>

namespace dlux_plugins
{
/**
 * @class Dijkstra
 * @brief Potential calculator that explores the potential breadth first while using the kernel function
 */
class Dijkstra : public dlux_global_planner::PotentialCalculator
{
public:
  // Main PotentialCalculator interface
  unsigned int updatePotentials(dlux_global_planner::PotentialGrid& potential_grid,
                                const geometry_msgs::Pose2D& start, const geometry_msgs::Pose2D& goal) override;
protected:
  /**
   * @brief Calculate the potential for next_index if not calculated already
   *
   * @param potential_grid Potential grid
   * @param next_index Coordinates of cell to calculate
   */
  void add(dlux_global_planner::PotentialGrid& potential_grid, nav_grid::Index next_index);

  std::queue<nav_grid::Index> queue_;
};
}  // namespace dlux_plugins

#endif  // DLUX_PLUGINS_DIJKSTRA_H
