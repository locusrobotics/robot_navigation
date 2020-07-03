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

#ifndef DWB_LOCAL_PLANNER_TRAJECTORY_CRITIC_H
#define DWB_LOCAL_PLANNER_TRAJECTORY_CRITIC_H

#include <ros/ros.h>
#include <nav_core2/common.h>
#include <nav_core2/costmap.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_2d_msgs/Twist2D.h>
#include <nav_2d_msgs/Path2D.h>
#include <dwb_msgs/Trajectory2D.h>
#include <sensor_msgs/PointCloud.h>
#include <string>
#include <vector>

namespace dwb_local_planner
{
/**
 * @class TrajectoryCritic
 * @brief Evaluates a Trajectory2D to produce a score
 *
 * This class defines the plugin interface for the TrajectoryCritic which
 * gives scores to trajectories, where lower numbers are better, but negative
 * scores are considered invalid.
 *
 * The general lifecycle is
 *  1) initialize is called once at the beginning which in turn calls onInit.
 *       Derived classes may override onInit to load parameters as needed.
 *  2) prepare is called once before each set of trajectories.
 *       It is presumed that there are multiple trajectories that we want to evaluate,
 *       and there may be some shared work that can be done beforehand to optimize
 *       the scoring of each individual trajectory.
 *  3) scoreTrajectory is called once per trajectory and returns the score.
 *  4) debrief is called after each set of trajectories with the chosen trajectory.
 *       This can be used for stateful critics that monitor the trajectory through time.
 *
 *  Optionally, there is also a debugging mechanism for certain types of critics in the
 *  addCriticVisualization method. If the score for a trajectory depends on its relationship to
 *  the costmap, addCriticVisualization can provide that information to the dwb_local_planner
 *  which will publish the grid scores as a PointCloud2.
 */
class TrajectoryCritic
{
public:
  using Ptr = std::shared_ptr<dwb_local_planner::TrajectoryCritic>;

  virtual ~TrajectoryCritic() {}

  /**
   * @brief Initialize the critic with appropriate pointers and parameters
   *
   * The name and costmap are stored as member variables.
   * A NodeHandle for the critic is created with the namespace of the planner NodeHandle
   *
   * @param planner_nh Planner Nodehandle
   * @param name The name of this critic
   * @param costmap_ros Pointer to the costmap
   */
  void initialize(const ros::NodeHandle& planner_nh, std::string name, nav_core2::Costmap::Ptr costmap)
  {
    name_ = name;
    costmap_ = costmap;
    planner_nh_ = planner_nh;
    critic_nh_ = ros::NodeHandle(planner_nh_, name_);
    critic_nh_.param("scale", scale_, 1.0);
    onInit();
  }

  virtual void onInit() {}

  /**
   * @brief Reset the state of the critic
   *
   * Reset is called when the planner receives a new global plan.
   * This can be used to discard information specific to one plan.
   */
  virtual void reset() {}

  /**
   * @brief Prior to evaluating any trajectories, look at contextual information constant across all trajectories
   *
   * Subclasses may overwrite. Return false in case there is any error.
   *
   * @param pose Current pose (costmap frame)
   * @param vel Current velocity
   * @param goal The final goal (costmap frame)
   * @param global_plan Transformed global plan in costmap frame, possibly cropped to nearby points
   */
  virtual bool prepare(const geometry_msgs::Pose2D& pose, const nav_2d_msgs::Twist2D& vel,
                       const geometry_msgs::Pose2D& goal,
                       const nav_2d_msgs::Path2D& global_plan)
  {
    return true;
  }

  /**
   * @brief Return a raw score for the given trajectory.
   *
   * scores < 0 are considered invalid/errors, such as collisions
   * This is the raw score in that the scale should not be applied to it.
   */
  virtual double scoreTrajectory(const dwb_msgs::Trajectory2D& traj) = 0;

  /**
   * @brief debrief informs the critic what the chosen cmd_vel was (if it cares)
   */
  virtual void debrief(const nav_2d_msgs::Twist2D& cmd_vel) {}

  /**
   * @brief Add information to the given pointcloud for debugging costmap-grid based scores
   *
   * addCriticVisualization is an optional debugging mechanism for providing rich information
   * about the cost for certain trajectories. Some critics will have scoring mechanisms
   * wherein there will be some score for each cell in the costmap. This could be as
   * straightforward as the cost in the costmap, or it could be the number of cells away
   * from the goal pose.
   *
   * Prior to calling this, dwb_local_planner will load the PointCloud's header and the points
   * in row-major order. The critic may then add a ChannelFloat to the channels member of the PC
   * with the same number of values as the points array. This information may then be converted
   * and published as a PointCloud2.
   *
   * @param pc PointCloud to add channels to
   */
  virtual void addCriticVisualization(sensor_msgs::PointCloud& pc) {}

  std::string getName()
  {
    return name_;
  }

  virtual double getScale() const { return scale_; }
  void setScale(const double scale) { scale_ = scale; }
protected:
  std::string name_;
  nav_core2::Costmap::Ptr costmap_;
  double scale_;
  ros::NodeHandle critic_nh_, planner_nh_;
};

}  // namespace dwb_local_planner

#endif  // DWB_LOCAL_PLANNER_TRAJECTORY_CRITIC_H
