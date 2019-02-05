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

#ifndef DLUX_GLOBAL_PLANNER_COST_INTERPRETER_H
#define DLUX_GLOBAL_PLANNER_COST_INTERPRETER_H
#include <nav_core2/costmap.h>
#include <ros/node_handle.h>
#include <array>
#include <memory>

namespace dlux_global_planner
{
enum struct UnknownInterpretation { LETHAL, EXPENSIVE, FREE };
const unsigned char LETHAL_COST = nav_core2::Costmap::INSCRIBED_INFLATED_OBSTACLE;
const float LETHAL_COST_F = static_cast<float>(LETHAL_COST);

/**
 * There is a fundamental tradeoff in planning between the length of the path and the costs contained in the costmap.
 * The neutral cost is a penalty for moving from one cell to an adjacent cell. The raw values in the costmap
 * can be weighted as well. See the README for more conceptual explanations.
 *
 * Practically speaking, we take the cost for each cell (c) and generally interpret it as a float:
 *     c' = neutral_cost + scale * c
 *
 * The caveat is that c' should not be greater than the the lethal cost, so non-lethal costs c with c' >= lethal_cost
 * will be interpreted as c' = lethal_cost -1
 *
 * A plot of this can be seen here: https://www.desmos.com/calculator/2haig4ki8h
 *
 * The special case is when c==255, i.e. NO_INFORMATION. There are three intepretations possible here.
 *   * LETHAL - Unknown cells are treated as lethal obstacles and cannot be passed through.
 *   * EXPENSIVE - Unknown cells are valid, but assigned a high cost. (lethal_cost - 1)
 *   * FREE - Unknown cells are valid and given the same weight as free cells. (neutral_cost)
 */
class CostInterpreter
{
public:
  void initialize(ros::NodeHandle& nh, nav_core2::Costmap::Ptr costmap);
  void setConfiguration(const unsigned char neutral_cost, const float scale, const UnknownInterpretation mode);

  inline unsigned char getNeutralCost() const { return neutral_cost_; }

  inline float interpretCost(const unsigned char cost) const
  {
    return cached_costs_[cost];
  }

  inline float getCost(const unsigned int x, const unsigned int y) const
  {
    return interpretCost(costmap_->operator()(x, y));
  }

  inline bool isLethal(const float cost) const
  {
    return cost >= LETHAL_COST_F;
  }

  typedef std::shared_ptr<CostInterpreter> Ptr;

protected:
  std::array<float, 256> cached_costs_;
  unsigned char neutral_cost_;

  nav_core2::Costmap::Ptr costmap_;
};
}  // namespace dlux_global_planner

#endif  // DLUX_GLOBAL_PLANNER_COST_INTERPRETER_H
