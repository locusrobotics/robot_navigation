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

#ifndef DLUX_PLUGINS_GRADIENT_PATH_H
#define DLUX_PLUGINS_GRADIENT_PATH_H

#include <dlux_global_planner/traceback.h>
#include <nav_grid/vector_nav_grid.h>

namespace dlux_plugins
{
/**
 * @class GradientPath
 * @brief Traceback function that creates a smooth gradient from the start to the goal
 */
class GradientPath: public dlux_global_planner::Traceback
{
public:
  // Main Traceback interface
  void initialize(ros::NodeHandle& private_nh, dlux_global_planner::CostInterpreter::Ptr cost_interpreter) override;
  nav_2d_msgs::Path2D getPath(const dlux_global_planner::PotentialGrid& potential_grid,
                              const geometry_msgs::Pose2D& start, const geometry_msgs::Pose2D& goal,
                              double& path_cost) override;
protected:
  bool shouldGridStep(const dlux_global_planner::PotentialGrid& potential_grid, const nav_grid::Index& index);
  nav_grid::Index gridStep(const dlux_global_planner::PotentialGrid& potential_grid, const nav_grid::Index& index);
  inline void calculateGradient(const dlux_global_planner::PotentialGrid& potential_grid, const nav_grid::Index& index);

  double step_size_;
  double lethal_cost_;
  double iteration_factor_;
  bool grid_step_near_high_;
  nav_grid::VectorNavGrid<double> gradx_, grady_;
};
}  // namespace dlux_plugins

#endif  // DLUX_PLUGINS_GRADIENT_PATH_H
