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
#ifndef NAV_CORE2_EXCEPTIONS_H
#define NAV_CORE2_EXCEPTIONS_H

#include <nav_2d_msgs/Pose2DStamped.h>
#include <stdexcept>
#include <string>
#include <memory>

/**************************************************
 * The nav_core2 Planning Exception Hierarchy!!
 * (with arbitrary integer result codes)
 **************************************************
 *
 * 0 CostmapException
 * 1     CostmapSafetyException
 * 2         CostmapDataLagException
 * 3 PlannerException
 * 4     GlobalPlannerException
 * 5         InvalidStartPoseException
 * 6             StartBoundsException
 * 7             OccupiedStartException
 * 8         InvalidGoalPoseException
 * 9             GoalBoundsException
 * 10            OccupiedGoalException
 * 11        NoGlobalPathException
 * 12        GlobalPlannerTimeoutException
 * 13    LocalPlannerException
 * 14        IllegalTrajectoryException
 * 15        NoLegalTrajectoriesException
 * 16    PlannerTFException
 *
 **************************************************/

namespace nav_core2
{

inline std::string poseToString(const nav_2d_msgs::Pose2DStamped& pose)
{
  return "(" + std::to_string(pose.pose.x) + ", " + std::to_string(pose.pose.y) + ", " + std::to_string(pose.pose.theta)
         + " : " + pose.header.frame_id + ")";
}

/**
 * @class CostmapException
 * @brief Extensible exception class for all costmap-related problems
 */
class CostmapException: public std::runtime_error
{
public:
  explicit CostmapException(const std::string& description) : std::runtime_error(description) {}
  using Ptr = std::shared_ptr<CostmapException>;
  virtual int getResultCode() const { return 0; }
};

/**
 * @class CostmapSafetyException
 * @brief General container for exceptions thrown when the costmap thinks any movement would be unsafe
 */
class CostmapSafetyException: public CostmapException
{
public:
  explicit CostmapSafetyException(const std::string& description) : CostmapException(description) {}
  int getResultCode() const override { return 1; }
};

/**
 * @class CostmapDataLagException
 * @brief Indicates costmap is out of date because data in not up to date.
 *
 * Functions similarly to `!Costmap2DROS::isCurrent()`
 */
class CostmapDataLagException: public CostmapSafetyException
{
public:
  explicit CostmapDataLagException(const std::string& description) : CostmapSafetyException(description) {}
  int getResultCode() const override { return 2; }
};

/**
 * @class PlannerException
 * @brief Parent type of all exceptions defined within
 */
class PlannerException: public std::runtime_error
{
public:
  explicit PlannerException(const std::string& description) : std::runtime_error(description) {}
  using Ptr = std::shared_ptr<PlannerException>;
  virtual int getResultCode() const { return 3; }
};

/**
 * @class GlobalPlannerException
 * @brief General container for exceptions thrown from the Global Planner
 */
class GlobalPlannerException: public PlannerException
{
public:
  explicit GlobalPlannerException(const std::string& description) : PlannerException(description) {}
  virtual int getResultCode() const { return 4; }
};

/**
 * @class LocalPlannerException
 * @brief General container for exceptions thrown from the Local Planner
 */
class LocalPlannerException: public PlannerException
{
public:
  explicit LocalPlannerException(const std::string& description) : PlannerException(description) {}
  virtual int getResultCode() const { return 13; }
};

/**
 * @class PlannerTFException
 * @brief Thrown when either the global or local planner cannot complete its operation due to TF errors
 */
class PlannerTFException: public PlannerException
{
public:
  explicit PlannerTFException(const std::string& description) : PlannerException(description) {}
  virtual int getResultCode() const { return 16; }
};

/**
 * @class InvalidStartPoseException
 * @brief Exception thrown when there is a problem at the start location for the global planner
 */
class InvalidStartPoseException: public GlobalPlannerException
{
public:
  explicit InvalidStartPoseException(const std::string& description) : GlobalPlannerException(description) {}
  InvalidStartPoseException(const nav_2d_msgs::Pose2DStamped& pose, const std::string& problem) :
    GlobalPlannerException("The starting pose " + poseToString(pose) + " is " + problem) {}
  virtual int getResultCode() const { return 5; }
};

/**
 * @class StartBoundsException
 * @brief Exception thrown when the start location of the global planner is out of the expected bounds
 */
class StartBoundsException: public InvalidStartPoseException
{
public:
  explicit StartBoundsException(const std::string& description) : InvalidStartPoseException(description) {}
  explicit StartBoundsException(const nav_2d_msgs::Pose2DStamped& pose) :
    InvalidStartPoseException(pose, "out of bounds") {}
  virtual int getResultCode() const { return 6; }
};

/**
 * @class OccupiedStartException
 * @brief Exception thrown when the start location of the global planner is occupied in the costmap
 */
class OccupiedStartException: public InvalidStartPoseException
{
public:
  explicit OccupiedStartException(const std::string& description) : InvalidStartPoseException(description) {}
  explicit OccupiedStartException(const nav_2d_msgs::Pose2DStamped& pose) :
    InvalidStartPoseException(pose, "occupied") {}
  virtual int getResultCode() const { return 7; }
};

/**
 * @class InvalidGoalPoseException
 * @brief Exception thrown when there is a problem at the goal location for the global planner
 */
class InvalidGoalPoseException: public GlobalPlannerException
{
public:
  explicit InvalidGoalPoseException(const std::string& description) : GlobalPlannerException(description) {}
  InvalidGoalPoseException(const nav_2d_msgs::Pose2DStamped& pose, const std::string& problem) :
    GlobalPlannerException("The goal pose " + poseToString(pose) + " is " + problem) {}
  virtual int getResultCode() const { return 8; }
};

/**
 * @class GoalBoundsException
 * @brief Exception thrown when the goal location of the global planner is out of the expected bounds
 */
class GoalBoundsException: public InvalidGoalPoseException
{
public:
  explicit GoalBoundsException(const std::string& description) : InvalidGoalPoseException(description) {}
  explicit GoalBoundsException(const nav_2d_msgs::Pose2DStamped& pose) :
    InvalidGoalPoseException(pose, "out of bounds") {}
  virtual int getResultCode() const { return 9; }
};

/**
 * @class OccupiedGoalException
 * @brief Exception thrown when the goal location of the global planner is occupied in the costmap
 */
class OccupiedGoalException: public InvalidGoalPoseException
{
public:
  explicit OccupiedGoalException(const std::string& description) : InvalidGoalPoseException(description) {}
  explicit OccupiedGoalException(const nav_2d_msgs::Pose2DStamped& pose) :
    InvalidGoalPoseException(pose, "occupied") {}
  virtual int getResultCode() const { return 10; }
};

/**
 * @class NoGlobalPathException
 * @brief Exception thrown when the global planner cannot find a path from the start to the goal
 */
class NoGlobalPathException: public GlobalPlannerException
{
public:
  explicit NoGlobalPathException(const std::string& description) : GlobalPlannerException(description) {}
  NoGlobalPathException() : GlobalPlannerException("No global path found.") {}
  virtual int getResultCode() const { return 11; }
};

/**
 * @class GlobalPlannerTimeoutException
 * @brief Exception thrown when the global planner has spent too long looking for a path
 */
class GlobalPlannerTimeoutException: public GlobalPlannerException
{
public:
  explicit GlobalPlannerTimeoutException(const std::string& description) : GlobalPlannerException(description) {}
  virtual int getResultCode() const { return 12; }
};

/**
 * @class IllegalTrajectoryException
 * @brief Thrown when one of the critics encountered a fatal error
 */
class IllegalTrajectoryException: public LocalPlannerException
{
public:
  IllegalTrajectoryException(const std::string& critic_name, const std::string& description)
    : LocalPlannerException(description), critic_name_(critic_name) {}
  std::string getCriticName() const { return critic_name_; }
  virtual int getResultCode() const { return 14; }
protected:
  std::string critic_name_;
};

/**
 * @class NoLegalTrajectoriesException
 * @brief Thrown when all the trajectories explored are illegal
 */
class NoLegalTrajectoriesException: public LocalPlannerException
{
public:
  explicit NoLegalTrajectoriesException(const std::string& description) : LocalPlannerException(description) {}
  virtual int getResultCode() const { return 15; }
};

}  // namespace nav_core2

#endif  // NAV_CORE2_EXCEPTIONS_H
