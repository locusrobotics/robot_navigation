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
#include <dwb_critics/path_dist.h>
#include <nav_grid/coordinate_conversion.h>
#include <pluginlib/class_list_macros.h>
#include <nav_2d_utils/path_ops.h>
#include <vector>

namespace dwb_critics
{
bool PathDistCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                             const geometry_msgs::Pose2D& goal,
                             const nav_2d_msgs::Path2D& global_plan)
{
  reset();
  const nav_core2::Costmap& costmap = *costmap_;
  const nav_grid::NavGridInfo& info = costmap.getInfo();
  bool started_path = false;

  nav_2d_msgs::Path2D adjusted_global_plan = nav_2d_utils::adjustPlanResolution(global_plan, info.resolution);

  if (adjusted_global_plan.poses.size() != global_plan.poses.size())
  {
    ROS_DEBUG_NAMED("PathDistCritic", "Adjusted global plan resolution, added %zu points",
                    adjusted_global_plan.poses.size() - global_plan.poses.size());
  }

  unsigned int i;
  // put global path points into local map until we reach the border of the local map
  for (i = 0; i < adjusted_global_plan.poses.size(); ++i)
  {
    double g_x = adjusted_global_plan.poses[i].x;
    double g_y = adjusted_global_plan.poses[i].y;
    unsigned int map_x, map_y;
    if (worldToGridBounded(info, g_x, g_y, map_x, map_y) && costmap(map_x, map_y) != costmap.NO_INFORMATION)
    {
      cell_values_.setValue(map_x, map_y, 0.0);
      queue_->enqueueCell(map_x, map_y);
      started_path = true;
    }
    else if (started_path)
    {
      break;
    }
  }
  if (!started_path)
  {
    ROS_ERROR_NAMED("PathDistCritic",
                    "None of the %d first of %zu (%zu) points of the global plan were in the local costmap and free",
                    i, adjusted_global_plan.poses.size(), global_plan.poses.size());
    return false;
  }

  propogateManhattanDistances();

  return true;
}

}  // namespace dwb_critics

PLUGINLIB_EXPORT_CLASS(dwb_critics::PathDistCritic, dwb_local_planner::TrajectoryCritic)
