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

#include <nav_2d_utils/path_ops.h>
#include <nav_2d_utils/geometry_help.h>
#include <cmath>
#include <vector>

namespace nav_2d_utils
{

double poseDistance(const geometry_msgs::Pose2D& pose0, const geometry_msgs::Pose2D& pose1)
{
  return hypot(pose0.x - pose1.x, pose0.y - pose1.y);
}

double getPlanLength(const nav_2d_msgs::Path2D& plan, const unsigned int start_index)
{
  double length = 0.0;
  for (unsigned int i = start_index + 1; i < plan.poses.size(); i++)
  {
    length += poseDistance(plan.poses[i - 1], plan.poses[i]);
  }
  return length;
}

double getPlanLength(const nav_2d_msgs::Path2D& plan, const geometry_msgs::Pose2D& query_pose)
{
  if (plan.poses.size() == 0) return 0.0;

  unsigned int closest_index = 0;
  double closest_distance = poseDistance(plan.poses[0], query_pose);
  for (unsigned int i = 1; i < plan.poses.size(); i++)
  {
    double distance = poseDistance(plan.poses[i], query_pose);
    if (closest_distance > distance)
    {
      closest_index = i;
      closest_distance = distance;
    }
  }
  return getPlanLength(plan, closest_index);
}

nav_2d_msgs::Path2D adjustPlanResolution(const nav_2d_msgs::Path2D& global_plan_in, double resolution)
{
  nav_2d_msgs::Path2D global_plan_out;
  global_plan_out.header = global_plan_in.header;
  if (global_plan_in.poses.size() == 0)
  {
    return global_plan_out;
  }

  geometry_msgs::Pose2D last = global_plan_in.poses[0];
  global_plan_out.poses.push_back(last);

  double sq_resolution = resolution * resolution;

  for (unsigned int i = 1; i < global_plan_in.poses.size(); ++i)
  {
    geometry_msgs::Pose2D loop = global_plan_in.poses[i];
    double sq_dist = (loop.x - last.x) * (loop.x - last.x) + (loop.y - last.y) * (loop.y - last.y);
    if (sq_dist > sq_resolution)
    {
      // add points in-between
      double diff = sqrt(sq_dist) - resolution;
      double steps_double = ceil(diff / resolution) + 1.0;
      int steps = static_cast<int>(steps_double);

      double delta_x = (loop.x - last.x) / steps_double;
      double delta_y = (loop.y - last.y) / steps_double;
      double delta_t = (loop.theta - last.theta) / steps_double;

      for (int j = 1; j < steps; ++j)
      {
        geometry_msgs::Pose2D pose;
        pose.x = last.x + j * delta_x;
        pose.y = last.y + j * delta_y;
        pose.theta = last.theta + j * delta_t;
        global_plan_out.poses.push_back(pose);
      }
    }
    global_plan_out.poses.push_back(global_plan_in.poses[i]);
    last.x = loop.x;
    last.y = loop.y;
  }
  return global_plan_out;
}

using PoseList = std::vector<geometry_msgs::Pose2D>;

/**
 * @brief Helper function for other version of compressPlan.
 *
 * Uses the Ramer Douglas Peucker algorithm. Ignores theta values
 *
 * @param input Full list of poses
 * @param start_index Index of first pose (inclusive)
 * @param end_index Index of last pose (inclusive)
 * @param epsilon maximum allowable error. Increase for greater compression.
 * @param list of poses possibly compressed for the poses[start_index, end_index]
 */
PoseList compressPlan(const PoseList& input, unsigned int start_index, unsigned int end_index, double epsilon)
{
  // Skip if only two points
  if (end_index - start_index <= 1)
  {
    PoseList::const_iterator first = input.begin() + start_index;
    PoseList::const_iterator last = input.begin() + end_index + 1;
    return PoseList(first, last);
  }

  // Find the point with the maximum distance to the line from start to end
  const geometry_msgs::Pose2D& start = input[start_index],
                                 end = input[end_index];
  double max_distance = 0.0;
  unsigned int max_index = 0;
  for (unsigned int i = start_index + 1; i < end_index; i++)
  {
    const geometry_msgs::Pose2D& pose = input[i];
    double d = distanceToLine(pose.x, pose.y, start.x, start.y, end.x, end.y);
    if (d > max_distance)
    {
      max_index = i;
      max_distance = d;
    }
  }

  // If max distance is less than epsilon, eliminate all the points between start and end
  if (max_distance <= epsilon)
  {
    PoseList outer;
    outer.push_back(start);
    outer.push_back(end);
    return outer;
  }

  // If max distance is greater than epsilon, recursively simplify
  PoseList first_part = compressPlan(input, start_index, max_index, epsilon);
  PoseList second_part = compressPlan(input, max_index, end_index, epsilon);
  first_part.insert(first_part.end(), second_part.begin() + 1, second_part.end());
  return first_part;
}

nav_2d_msgs::Path2D compressPlan(const nav_2d_msgs::Path2D& input_path, double epsilon)
{
  nav_2d_msgs::Path2D results;
  results.header = input_path.header;
  results.poses = compressPlan(input_path.poses, 0, input_path.poses.size() - 1, epsilon);
  return results;
}

void addPose(nav_2d_msgs::Path2D& path, double x, double y, double theta)
{
  geometry_msgs::Pose2D pose;
  pose.x = x;
  pose.y = y;
  pose.theta = theta;
  path.poses.push_back(pose);
}
}  // namespace nav_2d_utils
