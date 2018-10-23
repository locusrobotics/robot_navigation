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

#ifndef DWB_LOCAL_PLANNER_DWB_LOCAL_PLANNER_H
#define DWB_LOCAL_PLANNER_DWB_LOCAL_PLANNER_H

#include <dwb_local_planner/trajectory_generator.h>
#include <dwb_local_planner/goal_checker.h>
#include <dwb_local_planner/trajectory_critic.h>
#include <dwb_local_planner/publisher.h>
#include <nav_core2/local_planner.h>
#include <pluginlib/class_loader.h>
#include <string>
#include <vector>

namespace dwb_local_planner
{

/**
 * @class DWBLocalPlanner
 * @brief Plugin-based flexible local_planner
 */
class DWBLocalPlanner: public nav_core2::LocalPlanner
{
public:
  /**
   * @brief Constructor that brings up pluginlib loaders
   */
  DWBLocalPlanner();

  virtual ~DWBLocalPlanner() {}

  /**
   * @brief nav_core2 initialization
   * @param name Namespace for this planner
   * @param tf TFListener pointer
   * @param costmap_ros Costmap pointer
   */
  void initialize(const ros::NodeHandle& parent, const std::string& name,
                  TFListenerPtr tf, nav_core2::Costmap::Ptr costmap) override;

  /**
   * @brief nav_core2 setGoalPose - Sets the global goal pose
   * @param goal_pose The Goal Pose
   */
  void setGoalPose(const nav_2d_msgs::Pose2DStamped& goal_pose) override;

  /**
   * @brief nav_core2 setPlan - Sets the global plan
   * @param path The global plan
   */
  void setPlan(const nav_2d_msgs::Path2D& path) override;

  /**
   * @brief nav_core2 computeVelocityCommands - calculates the best command given the current pose and velocity
   *
   * It is presumed that the global plan is already set.
   *
   * This is mostly a wrapper for the protected computeVelocityCommands
   * function which has additional debugging info.
   *
   * @param pose Current robot pose
   * @param velocity Current robot velocity
   * @return The best command for the robot to drive
   */
  nav_2d_msgs::Twist2DStamped computeVelocityCommands(const nav_2d_msgs::Pose2DStamped& pose,
                                                      const nav_2d_msgs::Twist2D& velocity) override;

  /**
   * @brief nav_core2 isGoalReached - Check whether the robot has reached its goal, given the current pose & velocity.
   *
   * The pose that it checks against is the last pose in the current global plan.
   * The calculation is delegated to the goal_checker plugin.
   *
   * @param pose Current pose
   * @param velocity Current velocity
   * @return True if the robot should be considered as having reached the goal.
   */
  bool isGoalReached(const nav_2d_msgs::Pose2DStamped& pose, const nav_2d_msgs::Twist2D& velocity) override;

  /**
   * @brief Score a given command. Can be used for testing.
   *
   * Given a trajectory, calculate the score where lower scores are better.
   * If the given (positive) score exceeds the best_score, calculation may be cut short, as the
   * score can only go up from there.
   *
   * @param traj Trajectory to check
   * @param best_score If positive, the threshold for early termination
   * @return The full scoring of the input trajectory
   */
  virtual dwb_msgs::TrajectoryScore scoreTrajectory(const dwb_msgs::Trajectory2D& traj, double best_score = -1);

  /**
   * @brief Compute the best command given the current pose and velocity, with possible debug information
   *
   * Same as above computeVelocityCommands, but with debug results.
   * If the results pointer is not null, additional information about the twists
   * evaluated will be in results after the call.
   *
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param results   Output param, if not null, will be filled in with full evaluation results
   * @return          Best command
   */
  virtual nav_2d_msgs::Twist2DStamped computeVelocityCommands(const nav_2d_msgs::Pose2DStamped& pose,
                                                              const nav_2d_msgs::Twist2D& velocity,
                                                              std::shared_ptr<dwb_msgs::LocalPlanEvaluation>& results);


protected:
  /**
   * @brief Helper method for preparing for the core scoring algorithm
   *
   * Runs the prepare method on all the critics with freshly transformed data
   */
  virtual void prepare(const nav_2d_msgs::Pose2DStamped& pose, const nav_2d_msgs::Twist2D& velocity);

  /**
   * @brief Iterate through all the twists and find the best one
   */
  virtual dwb_msgs::TrajectoryScore coreScoringAlgorithm(const geometry_msgs::Pose2D& pose,
                                                         const nav_2d_msgs::Twist2D velocity,
                                                         std::shared_ptr<dwb_msgs::LocalPlanEvaluation>& results);

  /**
   * @brief Transforms global plan into same frame as pose, clips far away poses and possibly prunes passed poses
   *
   * Three key operations
   * 1) Transforms global plan into frame of the given pose
   * 2) Only returns poses that are near the robot, i.e. whether they are likely on the local costmap
   * 3) If prune_plan_ is true, it will remove all points that we've already passed from both the transformed plan
   *     and the saved global_plan_. Technically, it iterates to a pose on the path that is within prune_distance_
   *     of the robot and erases all poses before that.
   */
  virtual nav_2d_msgs::Path2D transformGlobalPlan(const nav_2d_msgs::Pose2DStamped& pose);

  /**
   * @brief Helper method to transform a given pose to the local costmap frame.
   */
  geometry_msgs::Pose2D transformPoseToLocal(const nav_2d_msgs::Pose2DStamped& pose);

  nav_2d_msgs::Path2D global_plan_;  ///< Saved Global Plan
  nav_2d_msgs::Pose2DStamped goal_pose_;  ///< Saved Goal Pose
  bool prune_plan_;
  double prune_distance_;
  bool debug_trajectory_details_;
  bool short_circuit_trajectory_evaluation_;

  // Plugin handling
  pluginlib::ClassLoader<TrajectoryGenerator> traj_gen_loader_;
  TrajectoryGenerator::Ptr traj_generator_;
  pluginlib::ClassLoader<GoalChecker> goal_checker_loader_;
  GoalChecker::Ptr goal_checker_;
  pluginlib::ClassLoader<TrajectoryCritic> critic_loader_;
  std::vector<TrajectoryCritic::Ptr> critics_;

  /**
   * @brief try to resolve a possibly shortened critic name with the default namespaces and the suffix "Critic"
   *
   * @param base_name The name of the critic as read in from the parameter server
   * @return Our attempted resolution of the name, with namespace prepended and/or the suffix Critic appended
   */
  std::string resolveCriticClassName(std::string base_name);

  /**
   * @brief Load the critic parameters from the namespace
   * @param name The namespace of this planner.
   */
  virtual void loadCritics(const std::string name);

  std::vector<std::string> default_critic_namespaces_;

  nav_core2::Costmap::Ptr costmap_;
  bool update_costmap_before_planning_;
  TFListenerPtr tf_;
  DWBPublisher pub_;

  ros::NodeHandle planner_nh_;
};

}  // namespace dwb_local_planner

#endif  // DWB_LOCAL_PLANNER_DWB_LOCAL_PLANNER_H
