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

#include <dwb_critics/base_obstacle.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dwb_critics::BaseObstacleCritic, dwb_local_planner::TrajectoryCritic)

namespace dwb_critics
{

void BaseObstacleCritic::onInit()
{
  critic_nh_.param("sum_scores", sum_scores_, false);
}

double BaseObstacleCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
  const nav_core2::Costmap& costmap = *costmap_;
  double score = 0.0;
  for (unsigned int i = 0; i < traj.poses.size(); ++i)
  {
    double pose_score = scorePose(costmap, traj.poses[i]);
    // Optimized/branchless version of if (sum_scores_) score += pose_score, else score = pose_score;
    score = static_cast<double>(sum_scores_) * score + pose_score;
  }
  return score;
}

double BaseObstacleCritic::scorePose(const nav_core2::Costmap& costmap, const geometry_msgs::Pose2D& pose)
{
  unsigned int cell_x, cell_y;
  if (!worldToGridBounded(costmap.getInfo(), pose.x, pose.y, cell_x, cell_y))
    throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  unsigned char cost = costmap(cell_x, cell_y);
  if (!isValidCost(cost))
    throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
  return cost;
}

bool BaseObstacleCritic::isValidCost(const unsigned char cost)
{
  return cost != costmap_->LETHAL_OBSTACLE &&
         cost != costmap_->INSCRIBED_INFLATED_OBSTACLE &&
         cost != costmap_->NO_INFORMATION;
}

void BaseObstacleCritic::addCriticVisualization(sensor_msgs::PointCloud& pc)
{
  sensor_msgs::ChannelFloat32 grid_scores;
  grid_scores.name = name_;

  const nav_core2::Costmap& costmap = *costmap_;
  unsigned int size_x = costmap.getWidth();
  unsigned int size_y = costmap.getHeight();
  grid_scores.values.resize(size_x * size_y);
  unsigned int i = 0;
  for (unsigned int cy = 0; cy < size_y; cy++)
  {
    for (unsigned int cx = 0; cx < size_x; cx++)
    {
      grid_scores.values[i] = costmap(cx, cy);
      i++;
    }
  }
  pc.channels.push_back(grid_scores);
}

}  // namespace dwb_critics
