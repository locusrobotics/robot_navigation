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

#include <dwb_critics/map_grid.h>
#include <nav_grid/coordinate_conversion.h>
#include <nav_core2/exceptions.h>
#include <algorithm>
#include <memory>
#include <string>

namespace dwb_critics
{

// Customization of the CostmapQueue validCellToQueue method
bool MapGridCritic::MapGridQueue::validCellToQueue(const costmap_queue::CellData& cell)
{
  unsigned char cost = costmap_(cell.x_, cell.y_);
  if (cost == costmap_.LETHAL_OBSTACLE ||
      cost == costmap_.INSCRIBED_INFLATED_OBSTACLE ||
      cost == costmap_.NO_INFORMATION)
  {
    parent_.setAsObstacle(cell.x_, cell.y_);
    return false;
  }

  return true;
}

void MapGridCritic::onInit()
{
  queue_ = std::make_shared<MapGridQueue>(*costmap_, *this);

  // Always set to true, but can be overriden by subclasses
  stop_on_failure_ = true;

  std::string aggro_str;
  critic_nh_.param("aggregation_type", aggro_str, std::string("last"));
  std::transform(aggro_str.begin(), aggro_str.end(), aggro_str.begin(), ::tolower);
  if (aggro_str == "last")
  {
    aggregationType_ = ScoreAggregationType::Last;
  }
  else if (aggro_str == "sum")
  {
    aggregationType_ = ScoreAggregationType::Sum;
  }
  else if (aggro_str == "product")
  {
    aggregationType_ = ScoreAggregationType::Product;
  }
  else
  {
    ROS_ERROR_NAMED("MapGridCritic", "aggregation_type parameter \"%s\" invalid. Using Last.", aggro_str.c_str());
    aggregationType_ = ScoreAggregationType::Last;
  }
}

void MapGridCritic::setAsObstacle(unsigned int x, unsigned int y)
{
  cell_values_.setValue(x, y, obstacle_score_);
}

void MapGridCritic::reset()
{
  queue_->reset();
  if (costmap_->getInfo() == cell_values_.getInfo())
  {
    cell_values_.reset();
  }
  else
  {
    obstacle_score_ = static_cast<double>(costmap_->getWidth() * costmap_->getHeight());
    unreachable_score_ = obstacle_score_ + 1.0;
    cell_values_.setDefaultValue(unreachable_score_);
    cell_values_.setInfo(costmap_->getInfo());
  }
}

void MapGridCritic::propogateManhattanDistances()
{
  while (!queue_->isEmpty())
  {
    costmap_queue::CellData cell = queue_->getNextCell();
    cell_values_.setValue(cell.x_, cell.y_,
                          std::abs(static_cast<int>(cell.src_x_) - static_cast<int>(cell.x_))
                          + std::abs(static_cast<int>(cell.src_y_) - static_cast<int>(cell.y_)));
  }
}

double MapGridCritic::scoreTrajectory(const dwb_msgs::Trajectory2D& traj)
{
  double score = 0.0;
  unsigned int start_index = 0;
  if (aggregationType_ == ScoreAggregationType::Product)
  {
    score = 1.0;
  }
  else if (aggregationType_ == ScoreAggregationType::Last && !stop_on_failure_)
  {
    start_index = traj.poses.size() - 1;
  }
  double grid_dist;

  for (unsigned int i = start_index; i < traj.poses.size(); ++i)
  {
    grid_dist = scorePose(traj.poses[i]);
    if (stop_on_failure_)
    {
      if (grid_dist == obstacle_score_)
      {
        throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Hits Obstacle.");
      }
      else if (grid_dist == unreachable_score_)
      {
        throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Hits Unreachable Area.");
      }
    }

    switch (aggregationType_)
    {
    case ScoreAggregationType::Last:
      score = grid_dist;
      break;
    case ScoreAggregationType::Sum:
      score += grid_dist;
      break;
    case ScoreAggregationType::Product:
      if (score > 0)
      {
        score *= grid_dist;
      }
      break;
    }
  }

  return score;
}

double MapGridCritic::scorePose(const geometry_msgs::Pose2D& pose)
{
  unsigned int cell_x, cell_y;
  // we won't allow trajectories that go off the map... shouldn't happen that often anyways
  if (!worldToGridBounded(costmap_->getInfo(), pose.x, pose.y, cell_x, cell_y))
  {
    throw nav_core2::IllegalTrajectoryException(name_, "Trajectory Goes Off Grid.");
  }
  return getScore(cell_x, cell_y);
}

void MapGridCritic::addCriticVisualization(sensor_msgs::PointCloud& pc)
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
      grid_scores.values[i] = getScore(cx, cy);
      i++;
    }
  }
  pc.channels.push_back(grid_scores);
}

}  // namespace dwb_critics
