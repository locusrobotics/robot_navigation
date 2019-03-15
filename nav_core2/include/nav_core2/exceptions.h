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

enum {
  COSTMAPEXCEPTION                 =   0,
  COSTMAPSAFETYEXCEPTION           =   1,
  COSTMAPDATALAGEXCEPTION          =   2,

  PLANNEREXCEPTION                 =   3,
    GLOBALPLANNEREXCEPTION         =   4,
      INVALIDSTARTPOSEEXCEPTION    =   5,
      STARTBOUNDSEXCEPTION         =   6,
      OCCUPIEDSTARTEXCEPTION       =   7,
      INVALIDGOALPOSEEXCEPTION     =   8,
      GOALBOUNDSEXCEPTION          =   9,
      OCCUPIEDGOALEXCEPTION        =   10,
    NOGLOBALPATHEXCEPTION          =   11,
    GLOBALPLANNERTIMEOUTEXCEPTION  =   12,
    LOCALPLANNEREXCEPTION          =   13,
      ILLEGALTRAJECTORYEXCEPTION   =   14,
      NOLEGALTRAJECTORIESEXCEPTION =   15,
    PLANNERTFEXCEPTION             =   16
};

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
  explicit CostmapException(const std::string& description, int result_code = COSTMAPEXCEPTION) : std::runtime_error(description), result_code_(result_code) {}
  using Ptr = std::shared_ptr<CostmapException>;
  virtual int getResultCode() const { return result_code_; }

protected:
    int result_code_;
};

/**
 * @class CostmapSafetyException
 * @brief General container for exceptions thrown when the costmap thinks any movement would be unsafe
 */
class CostmapSafetyException: public CostmapException
{
public:
  explicit CostmapSafetyException(const std::string& description, int result_code = COSTMAPSAFETYEXCEPTION) : CostmapException(description, result_code) {}
  int getResultCode() const override { return result_code_; }
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
  explicit CostmapDataLagException(const std::string& description, int result_code = COSTMAPDATALAGEXCEPTION) : CostmapSafetyException(description, result_code) {}
  int getResultCode() const override { return result_code_; }
};

/**
 * @class PlannerException
 * @brief Parent type of all exceptions defined within
 */
class PlannerException: public std::runtime_error
{
public:
  explicit PlannerException(const std::string& description, int result_code = PLANNEREXCEPTION) : std::runtime_error(description), result_code_(result_code) {}
  using Ptr = std::shared_ptr<PlannerException>;
  virtual int getResultCode() const { return result_code_; }

protected:
    int result_code_;

};

/**
 * @class GlobalPlannerException
 * @brief General container for exceptions thrown from the Global Planner
 */
class GlobalPlannerException: public PlannerException
{
public:
  explicit GlobalPlannerException(const std::string& description, int result_code = GLOBALPLANNEREXCEPTION) : PlannerException(description, result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class LocalPlannerException
 * @brief General container for exceptions thrown from the Local Planner
 */
class LocalPlannerException: public PlannerException
{
public:
  explicit LocalPlannerException(const std::string& description, int result_code = LOCALPLANNEREXCEPTION) : PlannerException(description, result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class PlannerTFException
 * @brief Thrown when either the global or local planner cannot complete its operation due to TF errors
 */
class PlannerTFException: public PlannerException
{
public:
  explicit PlannerTFException(const std::string& description, int result_code = PLANNERTFEXCEPTION) : PlannerException(description, result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class InvalidStartPoseException
 * @brief Exception thrown when there is a problem at the start location for the global planner
 */
class InvalidStartPoseException: public GlobalPlannerException
{
public:
  explicit InvalidStartPoseException(const std::string& description, int result_code = INVALIDSTARTPOSEEXCEPTION) : GlobalPlannerException(description, result_code) {}
  InvalidStartPoseException(const nav_2d_msgs::Pose2DStamped& pose, const std::string& problem, int result_code = INVALIDSTARTPOSEEXCEPTION) :
    GlobalPlannerException("The starting pose " + poseToString(pose) + " is " + problem, result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class StartBoundsException
 * @brief Exception thrown when the start location of the global planner is out of the expected bounds
 */
class StartBoundsException: public InvalidStartPoseException
{
public:
  explicit StartBoundsException(const std::string& description, int result_code = STARTBOUNDSEXCEPTION) : InvalidStartPoseException(description, result_code) {}
  explicit StartBoundsException(const nav_2d_msgs::Pose2DStamped& pose, int result_code = STARTBOUNDSEXCEPTION) :
    InvalidStartPoseException(pose, "out of bounds", result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class OccupiedStartException
 * @brief Exception thrown when the start location of the global planner is occupied in the costmap
 */
class OccupiedStartException: public InvalidStartPoseException
{
public:
  explicit OccupiedStartException(const std::string& description, int result_code = OCCUPIEDSTARTEXCEPTION) : InvalidStartPoseException(description, result_code) {}
  explicit OccupiedStartException(const nav_2d_msgs::Pose2DStamped& pose, int result_code = OCCUPIEDSTARTEXCEPTION) :
    InvalidStartPoseException(pose, "occupied", result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class InvalidGoalPoseException
 * @brief Exception thrown when there is a problem at the goal location for the global planner
 */
class InvalidGoalPoseException: public GlobalPlannerException
{
public:
  explicit InvalidGoalPoseException(const std::string& description, int result_code = INVALIDGOALPOSEEXCEPTION) : GlobalPlannerException(description, result_code) {}
  InvalidGoalPoseException(const nav_2d_msgs::Pose2DStamped& pose, const std::string& problem, int result_code = INVALIDGOALPOSEEXCEPTION) :
    GlobalPlannerException("The goal pose " + poseToString(pose) + " is " + problem, result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class GoalBoundsException
 * @brief Exception thrown when the goal location of the global planner is out of the expected bounds
 */
class GoalBoundsException: public InvalidGoalPoseException
{
public:
  explicit GoalBoundsException(const std::string& description, int result_code = GOALBOUNDSEXCEPTION) : InvalidGoalPoseException(description, result_code) {}
  explicit GoalBoundsException(const nav_2d_msgs::Pose2DStamped& pose, int result_code = GOALBOUNDSEXCEPTION) :
    InvalidGoalPoseException(pose, "out of bounds", result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class OccupiedGoalException
 * @brief Exception thrown when the goal location of the global planner is occupied in the costmap
 */
class OccupiedGoalException: public InvalidGoalPoseException
{
public:
  explicit OccupiedGoalException(const std::string& description, int result_code = OCCUPIEDGOALEXCEPTION) : InvalidGoalPoseException(description, result_code) {}
  explicit OccupiedGoalException(const nav_2d_msgs::Pose2DStamped& pose, int result_code = OCCUPIEDGOALEXCEPTION) :
    InvalidGoalPoseException(pose, "occupied", result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class NoGlobalPathException
 * @brief Exception thrown when the global planner cannot find a path from the start to the goal
 */
class NoGlobalPathException: public GlobalPlannerException
{
public:
  explicit NoGlobalPathException(const std::string& description, int result_code = NOGLOBALPATHEXCEPTION) : GlobalPlannerException(description, result_code) {}
  NoGlobalPathException(int result_code = NOGLOBALPATHEXCEPTION) : GlobalPlannerException("No global path found.", result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class GlobalPlannerTimeoutException
 * @brief Exception thrown when the global planner has spent too long looking for a path
 */
class GlobalPlannerTimeoutException: public GlobalPlannerException
{
public:
  explicit GlobalPlannerTimeoutException(const std::string& description, int result_code = GLOBALPLANNERTIMEOUTEXCEPTION) : GlobalPlannerException(description, result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

/**
 * @class IllegalTrajectoryException
 * @brief Thrown when one of the critics encountered a fatal error
 */
class IllegalTrajectoryException: public LocalPlannerException
{
public:
  IllegalTrajectoryException(const std::string& critic_name, const std::string& description, int result_code = ILLEGALTRAJECTORYEXCEPTION)
    : LocalPlannerException(description, result_code), critic_name_(critic_name) {}
  std::string getCriticName() const { return critic_name_; }
  virtual int getResultCode() const override { return result_code_; }
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
  explicit NoLegalTrajectoriesException(const std::string& description, int result_code = NOLEGALTRAJECTORIESEXCEPTION) : LocalPlannerException(description, result_code) {}
  virtual int getResultCode() const override { return result_code_; }
};

}  // namespace nav_core2

#endif  // NAV_CORE2_EXCEPTIONS_H
