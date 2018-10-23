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

#ifndef DWB_CRITICS_BASE_OBSTACLE_H
#define DWB_CRITICS_BASE_OBSTACLE_H

#include <dwb_local_planner/trajectory_critic.h>

namespace dwb_critics
{
/**
 * @class BaseObstacleCritic
 * @brief Uses costmap to assign negative costs if a circular robot would collide at any point of the trajectory.
 *
 * This class can only be used to figure out if a circular robot is in collision. If the cell corresponding
 * with any of the poses in the Trajectory is an obstacle, inscribed obstacle or unknown, it will return a
 * negative cost. Otherwise it will return either the final pose's cost, or the sum of all poses, depending
 * on the sum_scores parameter.
 *
 * Other classes (like ObstacleFootprintCritic) can do more advanced checking for collisions.
 */
class BaseObstacleCritic : public dwb_local_planner::TrajectoryCritic
{
public:
  void onInit() override;
  double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) override;
  void addCriticVisualization(sensor_msgs::PointCloud& pc) override;

  /**
   * @brief Return the obstacle score for a particular pose
   * @param costmap Dereferenced costmap
   * @param pose Pose to check
   */
  virtual double scorePose(const nav_core2::Costmap& costmap, const geometry_msgs::Pose2D& pose);

  /**
   * @brief Check to see whether a given cell cost is valid for driving through.
   * @param cost Cost of the cell
   * @return Return true if valid cell
   */
  virtual bool isValidCost(const unsigned char cost);

protected:
  bool sum_scores_;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS_BASE_OBSTACLE_H
