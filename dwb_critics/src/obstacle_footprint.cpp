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

#include <dwb_critics/obstacle_footprint.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_grid_iterators/polygon_outline.h>
#include <nav_2d_utils/polygons.h>
#include <nav_2d_utils/footprint.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>
#include <algorithm>
#include <vector>

PLUGINLIB_EXPORT_CLASS(dwb_critics::ObstacleFootprintCritic, dwb_local_planner::TrajectoryCritic)

namespace dwb_critics
{

void ObstacleFootprintCritic::onInit()
{
  BaseObstacleCritic::onInit();
  footprint_spec_ = nav_2d_utils::footprintFromParams(critic_nh_);
}

bool ObstacleFootprintCritic::prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                                      const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan)
{
  if (footprint_spec_.points.size() == 0)
  {
    ROS_ERROR_NAMED("ObstacleFootprintCritic", "Footprint spec is empty, maybe missing call to setFootprint?");
    return false;
  }
  return true;
}

double ObstacleFootprintCritic::scorePose(const nav_core2::Costmap& costmap, const geometry_msgs::Pose2D& pose)
{
  unsigned int cell_x, cell_y;
  if (!worldToGridBounded(costmap.getInfo(), pose.x, pose.y, cell_x, cell_y))
    throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  return scorePose(costmap, pose, nav_2d_utils::movePolygonToPose(footprint_spec_, pose));
}

double ObstacleFootprintCritic::scorePose(const nav_core2::Costmap& costmap, const geometry_msgs::Pose2D& pose,
                                          const nav_2d_msgs::Polygon2D& footprint)
{
  unsigned char footprint_cost = 0;
  nav_grid::NavGridInfo info = costmap.getInfo();
  for (nav_grid::Index index : nav_grid_iterators::PolygonOutline(&info, footprint))
  {
    unsigned char cost = costmap(index.x, index.y);
    // if the cell is in an obstacle the path is invalid or unknown
    if (cost == costmap.LETHAL_OBSTACLE)
    {
      throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
    }
    else if (cost == costmap.NO_INFORMATION)
    {
      throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Hits Unknown Region.");
    }
    footprint_cost = std::max(cost, footprint_cost);
  }

  // if all line costs are legal... then we can return that the footprint is legal
  return footprint_cost;
}

}  // namespace dwb_critics
