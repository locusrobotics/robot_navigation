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

#ifndef NAV_2D_UTILS_PATH_OPS_H
#define NAV_2D_UTILS_PATH_OPS_H

#include <nav_2d_msgs/Path2D.h>

namespace nav_2d_utils
{
/**
 * @brief Calculate the linear distance between two poses
 */
double poseDistance(const geometry_msgs::Pose2D& pose0, const geometry_msgs::Pose2D& pose1);

/**
 * @brief Calculate the length of the plan, starting from the given index
 */
double getPlanLength(const nav_2d_msgs::Path2D& plan, const unsigned int start_index = 0);

/**
 * @brief Calculate the length of the plan from the pose on the plan closest to the given pose
 */
double getPlanLength(const nav_2d_msgs::Path2D& plan, const geometry_msgs::Pose2D& query_pose);

/**
 * @brief Increase plan resolution to match that of the costmap by adding points linearly between points
 *
 * @param global_plan_in input plan
 * @param resolution desired distance between waypoints
 * @return Higher resolution plan
 */
nav_2d_msgs::Path2D adjustPlanResolution(const nav_2d_msgs::Path2D& global_plan_in, double resolution);

/**
 * @brief Decrease the length of the plan by eliminating colinear points
 *
 * Uses the Ramer Douglas Peucker algorithm. Ignores theta values
 *
 * @param input_path Path to compress
 * @param epsilon maximum allowable error. Increase for greater compression.
 * @return Path2D with possibly fewer poses
 */
nav_2d_msgs::Path2D compressPlan(const nav_2d_msgs::Path2D& input_path, double epsilon = 0.1);

/**
 * @brief Convenience function to add a pose to a path in one line.
 * @param path Path to add to
 * @param x x-coordinate
 * @param y y-coordinate
 * @param theta theta (if needed)
 */
void addPose(nav_2d_msgs::Path2D& path, double x, double y, double theta = 0.0);

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS_PATH_OPS_H
