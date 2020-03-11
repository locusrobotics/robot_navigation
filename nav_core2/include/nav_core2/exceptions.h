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
#include <exception>
#include <string>
#include <memory>

/**************************************************
 * The nav_core2 Planning Exception Hierarchy!!
 * (with arbitrary integer result codes)
 **************************************************
 *   NavCore2Exception
 * 0   CostmapException
 * 1     CostmapSafetyException
 * 2       CostmapDataLagException
 * 3   PlannerException
 * 4     GlobalPlannerException
 * 5       InvalidStartPoseException
 * 6         StartBoundsException
 * 7         OccupiedStartException
 * 8       InvalidGoalPoseException
 * 9         GoalBoundsException
 * 10        OccupiedGoalException
 * 11      NoGlobalPathException
 * 12      GlobalPlannerTimeoutException
 * 13    LocalPlannerException
 * 14      IllegalTrajectoryException
 * 15      NoLegalTrajectoriesException
 * 16    PlannerTFException
 *
 * -1 Unknown
 **************************************************/

namespace nav_core2
{

inline std::string poseToString(const nav_2d_msgs::Pose2DStamped& pose)
{
  return "(" + std::to_string(pose.pose.x) + ", " + std::to_string(pose.pose.y) + ", " + std::to_string(pose.pose.theta)
         + " : " + pose.header.frame_id + ")";
}

class NavCore2Exception: public std::runtime_error
{
public:
  explicit NavCore2Exception(const std::string& description, int result_code)
    : std::runtime_error(description), result_code_(result_code) {}
  int getResultCode() const { return result_code_; }
protected:
  int result_code_;
};

using NavCore2ExceptionPtr = std::exception_ptr;

/**
 * @brief Handy function for getting the result code
 */
inline int getResultCode(const NavCore2ExceptionPtr& e_ptr)
{
  if (e_ptr == nullptr)
  {
    return -1;
  }
  try
  {
    std::rethrow_exception(e_ptr);
  }
  catch (const NavCore2Exception& e)
  {
    return e.getResultCode();
  }
  catch (...)
  {
    // Will end up here if current_exception returned a non-NavCore2Exception
    return -1;
  }
}

/**
 * @class CostmapException
 * @brief Extensible exception class for all costmap-related problems
 */
class CostmapException: public NavCore2Exception
{
public:
  explicit CostmapException(const std::string& description, int result_code = 0)
    : NavCore2Exception(description, result_code) {}
};

/**
 * @class CostmapSafetyException
 * @brief General container for exceptions thrown when the costmap thinks any movement would be unsafe
 */
class CostmapSafetyException: public CostmapException
{
public:
  explicit CostmapSafetyException(const std::string& description, int result_code = 1)
    : CostmapException(description, result_code) {}
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
  explicit CostmapDataLagException(const std::string& description, int result_code = 2)
    : CostmapSafetyException(description, result_code) {}
};

/**
 * @class PlannerException
 * @brief Parent type of all exceptions defined within
 */
class PlannerException: public NavCore2Exception
{
public:
  explicit PlannerException(const std::string& description, int result_code = 3)
    : NavCore2Exception(description, result_code) {}
};

/**
 * @class GlobalPlannerException
 * @brief General container for exceptions thrown from the Global Planner
 */
class GlobalPlannerException: public PlannerException
{
public:
  explicit GlobalPlannerException(const std::string& description, int result_code = 4)
    : PlannerException(description, result_code) {}
};

/**
 * @class LocalPlannerException
 * @brief General container for exceptions thrown from the Local Planner
 */
class LocalPlannerException: public PlannerException
{
public:
  explicit LocalPlannerException(const std::string& description, int result_code = 13)
    : PlannerException(description, result_code) {}
};

/**
 * @class PlannerTFException
 * @brief Thrown when either the global or local planner cannot complete its operation due to TF errors
 */
class PlannerTFException: public PlannerException
{
public:
  explicit PlannerTFException(const std::string& description, int result_code = 16)
    : PlannerException(description, result_code) {}
};

/**
 * @class InvalidStartPoseException
 * @brief Exception thrown when there is a problem at the start location for the global planner
 */
class InvalidStartPoseException: public GlobalPlannerException
{
public:
  explicit InvalidStartPoseException(const std::string& description, int result_code = 5)
    : GlobalPlannerException(description, result_code) {}
  InvalidStartPoseException(const nav_2d_msgs::Pose2DStamped& pose, const std::string& problem, int result_code = 5) :
    InvalidStartPoseException("The starting pose " + poseToString(pose) + " is " + problem, result_code) {}
};

/**
 * @class StartBoundsException
 * @brief Exception thrown when the start location of the global planner is out of the expected bounds
 */
class StartBoundsException: public InvalidStartPoseException
{
public:
  explicit StartBoundsException(const std::string& description, int result_code = 6)
    : InvalidStartPoseException(description, result_code) {}
  explicit StartBoundsException(const nav_2d_msgs::Pose2DStamped& pose) :
    InvalidStartPoseException(pose, "out of bounds", 6) {}
};

/**
 * @class OccupiedStartException
 * @brief Exception thrown when the start location of the global planner is occupied in the costmap
 */
class OccupiedStartException: public InvalidStartPoseException
{
public:
  explicit OccupiedStartException(const std::string& description, int result_code = 7)
    : InvalidStartPoseException(description, result_code) {}
  explicit OccupiedStartException(const nav_2d_msgs::Pose2DStamped& pose) :
    InvalidStartPoseException(pose, "occupied", 7) {}
};

/**
 * @class InvalidGoalPoseException
 * @brief Exception thrown when there is a problem at the goal location for the global planner
 */
class InvalidGoalPoseException: public GlobalPlannerException
{
public:
  explicit InvalidGoalPoseException(const std::string& description, int result_code = 8)
    : GlobalPlannerException(description, result_code) {}
  InvalidGoalPoseException(const nav_2d_msgs::Pose2DStamped& pose, const std::string& problem, int result_code = 8) :
    GlobalPlannerException("The goal pose " + poseToString(pose) + " is " + problem, result_code) {}
};

/**
 * @class GoalBoundsException
 * @brief Exception thrown when the goal location of the global planner is out of the expected bounds
 */
class GoalBoundsException: public InvalidGoalPoseException
{
public:
  explicit GoalBoundsException(const std::string& description, int result_code = 9)
    : InvalidGoalPoseException(description, result_code) {}
  explicit GoalBoundsException(const nav_2d_msgs::Pose2DStamped& pose) :
    InvalidGoalPoseException(pose, "out of bounds", 9) {}
};

/**
 * @class OccupiedGoalException
 * @brief Exception thrown when the goal location of the global planner is occupied in the costmap
 */
class OccupiedGoalException: public InvalidGoalPoseException
{
public:
  explicit OccupiedGoalException(const std::string& description, int result_code = 10)
    : InvalidGoalPoseException(description, result_code) {}
  explicit OccupiedGoalException(const nav_2d_msgs::Pose2DStamped& pose) :
    InvalidGoalPoseException(pose, "occupied", 10) {}
};

/**
 * @class NoGlobalPathException
 * @brief Exception thrown when the global planner cannot find a path from the start to the goal
 */
class NoGlobalPathException: public GlobalPlannerException
{
public:
  explicit NoGlobalPathException(const std::string& description, int result_code = 11)
    : GlobalPlannerException(description, result_code) {}
  NoGlobalPathException() : GlobalPlannerException("No global path found.") {}
};

/**
 * @class GlobalPlannerTimeoutException
 * @brief Exception thrown when the global planner has spent too long looking for a path
 */
class GlobalPlannerTimeoutException: public GlobalPlannerException
{
public:
  explicit GlobalPlannerTimeoutException(const std::string& description, int result_code = 12)
    : GlobalPlannerException(description, result_code) {}
};

/**
 * @class IllegalTrajectoryException
 * @brief Thrown when one of the critics encountered a fatal error
 */
class IllegalTrajectoryException: public LocalPlannerException
{
public:
  IllegalTrajectoryException(const std::string& critic_name, const std::string& description, int result_code = 14)
    : LocalPlannerException(description, result_code), critic_name_(critic_name) {}
  std::string getCriticName() const { return critic_name_; }
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
  explicit NoLegalTrajectoriesException(const std::string& description, int result_code = 15)
    : LocalPlannerException(description, result_code) {}
};

}  // namespace nav_core2

#endif  // NAV_CORE2_EXCEPTIONS_H
