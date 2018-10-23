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

#ifndef DWB_CRITICS_OBSTACLE_FOOTPRINT_H
#define DWB_CRITICS_OBSTACLE_FOOTPRINT_H

#include <dwb_critics/base_obstacle.h>
#include <nav_2d_msgs/Polygon2D.h>
#include <vector>

namespace dwb_critics
{

/**
 * @class ObstacleFootprintCritic
 * @brief Uses costmap 2d to assign negative costs if robot footprint is in obstacle on any point of the trajectory.
 *
 * Internally, this technically only checks if the border of the footprint collides with anything for computational
 * efficiency. This is valid if the obstacles in the local costmap are inflated.
 *
 * A more robust class could check every cell within the robot's footprint without inflating the obstacles,
 * at some computational cost. That is left as an excercise to the reader.
 */
class ObstacleFootprintCritic : public BaseObstacleCritic
{
public:
  void onInit() override;
  bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
               const geometry_msgs::Pose2D& goal, const nav_2d_msgs::Path2D& global_plan) override;
  double scorePose(const nav_core2::Costmap& costmap, const geometry_msgs::Pose2D& pose) override;
  virtual double scorePose(const nav_core2::Costmap& costmap, const geometry_msgs::Pose2D& pose,
                           const nav_2d_msgs::Polygon2D& oriented_footprint);
  double getScale() const override { return costmap_->getResolution() * scale_; }
protected:
  nav_2d_msgs::Polygon2D footprint_spec_;
};
}  // namespace dwb_critics

#endif  // DWB_CRITICS_OBSTACLE_FOOTPRINT_H
