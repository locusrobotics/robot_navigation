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

#ifndef DLUX_GLOBAL_PLANNER_DLUX_GLOBAL_PLANNER_H
#define DLUX_GLOBAL_PLANNER_DLUX_GLOBAL_PLANNER_H

#include <nav_core2/global_planner.h>
#include <nav_core2/costmap.h>
#include <dlux_global_planner/cost_interpreter.h>
#include <dlux_global_planner/potential_calculator.h>
#include <dlux_global_planner/traceback.h>
#include <nav_2d_msgs/Pose2DStamped.h>
#include <nav_grid_pub_sub/nav_grid_publisher.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <string>

namespace dlux_global_planner
{
/**
 * @class DluxGlobalPlanner
 * @brief Plugin-based global wavefront planner that conforms to the `nav_core2` GlobalPlanner interface
 */
class DluxGlobalPlanner: public nav_core2::GlobalPlanner
{
public:
  DluxGlobalPlanner();

  // Standard GlobalPlanner Interface
  void initialize(const ros::NodeHandle& parent, const std::string& name,
                  TFListenerPtr tf, nav_core2::Costmap::Ptr costmap) override;
  nav_2d_msgs::Path2D makePlan(const nav_2d_msgs::Pose2DStamped& start,
                               const nav_2d_msgs::Pose2DStamped& goal) override;

  /**
   * @brief Check the costmap for any obstacles on this path
   * @param path Path to check
   * @return True if there are no obstacles
   */
  virtual bool isPlanValid(const nav_2d_msgs::Path2D& path) const;

protected:
  // Path Caching Methods

  /**
   * @brief Check whether there is a valid cached path where the goal hasn't changed
   *
   * The precise goal and the grid coordinates of the goal are passed (redundantly) so the implementing method
   * can decide what precision to require for the goal to be the same. The default implementation uses the grid coords.
   *
   * Also sets the cached goal data member(s)
   *
   * @param local_goal Precise (floating point) goal
   * @param goal_x x coordinate of the goal in the grid
   * @param goal_y y coordinate of the goal in the grid
   * @return True if a path has been cached, the goal is the same as the cached paths, and the plan is valid
   */
  virtual bool hasValidCachedPath(const geometry_msgs::Pose2D& local_goal,
                                  unsigned int goal_x, unsigned int goal_y);

  /**
   * @brief Whether the planner should always return a valid cached path without running the planning algorithm
   * @return True if the valid cached path should be returned without running the planning algorithm
   */
  virtual bool shouldReturnCachedPathImmediately() const;

  /**
   * @brief Given a cached path is available and a new path, should the new path be the one returned?
   * @param new_path The new path
   * @param new_path_cost The cost of the new path, according to the traceback
   * @return True if the new path should be the one returned. If False, return the cached one
   */
  virtual bool shouldReturnNewPath(const nav_2d_msgs::Path2D& new_path, const double new_path_cost) const;

  // Plugins
  pluginlib::ClassLoader<PotentialCalculator> calc_loader_;
  boost::shared_ptr<PotentialCalculator> calculator_;
  pluginlib::ClassLoader<Traceback> traceback_loader_;
  boost::shared_ptr<Traceback> traceback_;

  // Key members
  nav_core2::Costmap::Ptr costmap_;
  TFListenerPtr tf_;
  PotentialGrid potential_grid_;
  CostInterpreter::Ptr cost_interpreter_;

  // Path Caching
  bool path_caching_;
  double improvement_threshold_;
  nav_2d_msgs::Path2D cached_path_;
  unsigned int cached_goal_x_, cached_goal_y_;
  double cached_path_cost_;

  // potential publishing
  nav_grid_pub_sub::ScaleGridPublisher<float> potential_pub_;

  // debug printing
  bool print_statistics_;
};
}  // namespace dlux_global_planner

#endif  // DLUX_GLOBAL_PLANNER_DLUX_GLOBAL_PLANNER_H
