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

#ifndef DLUX_GLOBAL_PLANNER_POTENTIAL_CALCULATOR_H
#define DLUX_GLOBAL_PLANNER_POTENTIAL_CALCULATOR_H

#include <ros/ros.h>
#include <dlux_global_planner/potential.h>
#include <dlux_global_planner/cost_interpreter.h>
#include <geometry_msgs/Pose2D.h>

namespace dlux_global_planner
{
/**
 * @class PotentialCalculator
 * @brief Plugin interface for calculating a numerical score (potential) for some subset of the cells in the costmap.
 *
 * The potential should be zero at the goal and grow higher from there.
 * See README for more details
 */
class PotentialCalculator
{
public:
  PotentialCalculator() : cost_interpreter_(nullptr) {}
  virtual ~PotentialCalculator() = default;

  /**
   * @brief Initialize function
   *
   * Can be overridden to grab parameters or direct access to the costmap
   *
   * @param private_nh NodeHandle to read parameters from
   * @param costmap Pointer to costmap (possibly for tracking changes)
   * @param cost_interpreter CostInterpreter pointer
   */
  virtual void initialize(ros::NodeHandle& private_nh, nav_core2::Costmap::Ptr costmap,
                          CostInterpreter::Ptr cost_interpreter)
    { cost_interpreter_ = cost_interpreter; }

  /**
   * @brief Main potential calculation function
   *
   * @param potential_grid potentials are written into here
   * @param start Start pose, in the same frame as the potential
   * @param goal Goal pose, in the same frame as the potential
   * @return Number of cells expanded. Used for comparing expansion strategies.
   */
  virtual unsigned int updatePotentials(PotentialGrid& potential_grid,
                                        const geometry_msgs::Pose2D& start, const geometry_msgs::Pose2D& goal) = 0;
protected:
  CostInterpreter::Ptr cost_interpreter_;
};
}  // namespace dlux_global_planner

#endif  // DLUX_GLOBAL_PLANNER_POTENTIAL_CALCULATOR_H
