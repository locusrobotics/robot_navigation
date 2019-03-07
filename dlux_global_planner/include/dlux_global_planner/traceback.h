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

#ifndef DLUX_GLOBAL_PLANNER_TRACEBACK_H
#define DLUX_GLOBAL_PLANNER_TRACEBACK_H

#include <ros/ros.h>
#include <dlux_global_planner/potential.h>
#include <dlux_global_planner/cost_interpreter.h>
#include <nav_grid/nav_grid_info.h>
#include <nav_2d_msgs/Path2D.h>

namespace dlux_global_planner
{
/**
 * @class Traceback
 * @brief Plugin interface for using the potential to trace a path from the start position back to the goal position.
 *
 * See README for more details
 */
class Traceback
{
public:
  Traceback() : cost_interpreter_(nullptr) {}
  virtual ~Traceback() = default;

  /**
   * @brief Initialize function
   *
   * Can be overridden to grab parameters
   *
   * @param private_nh NodeHandle to read parameters from
   * @param cost_interpreter CostInterpreter pointer
   */
  virtual void initialize(ros::NodeHandle& private_nh, CostInterpreter::Ptr cost_interpreter)
    { cost_interpreter_ = cost_interpreter; }

  /**
   * @brief Create the path from start to goal
   *
   * @param potential_grid[in] potential grid
   * @param start[in] start pose
   * @param goal[in] goal pose
   * @param path_cost[out] A number representing the cost of taking this path. Open to intepretation.
   * @return The path, in the same frame as the PotentialGrid
   */
  virtual nav_2d_msgs::Path2D getPath(const PotentialGrid& potential_grid,
                                      const geometry_msgs::Pose2D& start, const geometry_msgs::Pose2D& goal,
                                      double& path_cost) = 0;
protected:
  /**
   * @brief Helper function to convert a path in Grid coordinates to World coordinates
   *
   * Does not use nav_grid::gridToWorld because that takes integer inputs, whereas the input path
   * can be located anywhere within the grid cell.
   */
  nav_2d_msgs::Path2D mapPathToWorldPath(const nav_2d_msgs::Path2D& original, const nav_grid::NavGridInfo& info)
  {
    nav_2d_msgs::Path2D world_path;
    world_path.header.frame_id = info.frame_id;
    world_path.poses.resize(original.poses.size());
    for (unsigned int i = 0; i < original.poses.size(); i++)
    {
      world_path.poses[i].x = info.origin_x + original.poses[i].x * info.resolution;
      world_path.poses[i].y = info.origin_y + original.poses[i].y * info.resolution;
    }
    return world_path;
  }
  CostInterpreter::Ptr cost_interpreter_;
};
}  // namespace dlux_global_planner

#endif  // DLUX_GLOBAL_PLANNER_TRACEBACK_H
