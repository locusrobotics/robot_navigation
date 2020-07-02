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

#include <dwb_local_planner/debug_dwb_local_planner.h>
#include <nav_2d_utils/tf_help.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>
#include <memory>
#include <string>

namespace dwb_local_planner
{

void DebugDWBLocalPlanner::initialize(const ros::NodeHandle& parent, const std::string& name,
                                      TFListenerPtr tf, nav_core2::Costmap::Ptr costmap)
{
  DWBLocalPlanner::initialize(parent, name, tf, costmap);

  debug_service_ = planner_nh_.advertiseService("debug_local_plan",
                                                &DebugDWBLocalPlanner::debugLocalPlanService, this);
  twist_gen_service_ = planner_nh_.advertiseService("generate_twists",
                                                    &DebugDWBLocalPlanner::generateTwistsService, this);
  score_service_ = planner_nh_.advertiseService("score_trajectory",
                                                &DebugDWBLocalPlanner::scoreTrajectoryService, this);
  critic_service_ = planner_nh_.advertiseService("get_critic_score",
                                                 &DebugDWBLocalPlanner::getCriticScoreService, this);
  generate_traj_service_ = planner_nh_.advertiseService("generate_traj",
                                                        &DebugDWBLocalPlanner::generateTrajectoryService, this);
}

bool DebugDWBLocalPlanner::generateTwistsService(dwb_msgs::GenerateTwists::Request  &req,
                                                 dwb_msgs::GenerateTwists::Response &res)
{
  res.twists = traj_generator_->getTwists(req.current_vel);
  return true;
}

bool DebugDWBLocalPlanner::generateTrajectoryService(dwb_msgs::GenerateTrajectory::Request  &req,
                                                     dwb_msgs::GenerateTrajectory::Response &res)
{
  res.traj = traj_generator_->generateTrajectory(req.start_pose, req.start_vel, req.cmd_vel);
  return true;
}

bool DebugDWBLocalPlanner::scoreTrajectoryService(dwb_msgs::ScoreTrajectory::Request  &req,
                                                  dwb_msgs::ScoreTrajectory::Response &res)
{
  if (req.goal.header.frame_id != "")
    setGoalPose(req.goal);
  if (req.global_plan.poses.size() > 0)
    setPlan(req.global_plan);

  prepare(req.pose, req.velocity);
  res.score = scoreTrajectory(req.traj);
  return true;
}

TrajectoryCritic::Ptr DebugDWBLocalPlanner::getCritic(std::string name)
{
  for (TrajectoryCritic::Ptr critic : critics_)
  {
    if (critic->getName() == name)
      return critic;
  }
  return nullptr;
}

bool DebugDWBLocalPlanner::getCriticScoreService(dwb_msgs::GetCriticScore::Request  &req,
                                                 dwb_msgs::GetCriticScore::Response &res)
{
  TrajectoryCritic::Ptr critic = getCritic(req.critic_name);
  if (critic == nullptr)
  {
    ROS_WARN_NAMED("DebugDWBLocalPlanner", "Critic %s not found!", req.critic_name.c_str());
    return false;
  }
  if (req.goal.header.frame_id != "")
    setGoalPose(req.goal);
  if (req.global_plan.poses.size() > 0)
    setPlan(req.global_plan);

  // This prepares all the critics, even though we only need to prepare the one
  prepare(req.pose, req.velocity);

  res.score.raw_score = critic->scoreTrajectory(req.traj);
  res.score.scale = critic->getScale();
  res.score.name = req.critic_name;

  pub_.publishCostGrid(costmap_, critics_);

  return true;
}

bool DebugDWBLocalPlanner::debugLocalPlanService(dwb_msgs::DebugLocalPlan::Request &req,
                                                 dwb_msgs::DebugLocalPlan::Response &res)
{
  if (req.goal.header.frame_id != "")
    setGoalPose(req.goal);
  if (req.global_plan.poses.size() > 0)
    setPlan(req.global_plan);
  std::shared_ptr<dwb_msgs::LocalPlanEvaluation> results = std::make_shared<dwb_msgs::LocalPlanEvaluation>();
  try
  {
    computeVelocityCommands(req.pose, req.velocity, results);
    res.results = *results;
    return true;
  }
  catch (const nav_core2::PlannerException& e)
  {
    ROS_ERROR_NAMED("DebugDWBLocalPlanner", "Exception in computeVelocityCommands: %s", e.what());
    return false;
  }
}

}  // namespace dwb_local_planner


//  register this planner as a LocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(dwb_local_planner::DebugDWBLocalPlanner, nav_core2::LocalPlanner)
