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

#include <dlux_global_planner/cost_interpreter.h>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <string>

namespace dlux_global_planner
{
void CostInterpreter::initialize(ros::NodeHandle& nh, nav_core2::Costmap::Ptr costmap)
{
  costmap_ = costmap;
  int neutral_cost;
  nh.param("neutral_cost", neutral_cost, 50);
  if (neutral_cost < 0 || neutral_cost > std::numeric_limits<unsigned char>::max())
  {
    throw std::invalid_argument("neutral_cost (" + std::to_string(neutral_cost) + ") must be a valid unsigned char!");
  }

  float scale;
  // The default value matches navfn's default value
  // https://github.com/ros-planning/navigation/blob/8ea5462f5395acf9741253ff38302d29175dd870/global_planner/cfg/GlobalPlanner.cfg#L10
  nh.param("scale", scale, 3.0f);

  UnknownInterpretation mode = UnknownInterpretation::EXPENSIVE;
  if (nh.hasParam("unknown_interpretation"))
  {
    if (nh.hasParam("allow_unknown"))
    {
      ROS_ERROR("allow_unknown can't be specified at the same time as unknown_interpretation.");
      ROS_ERROR("Using the value of unknown_interpretation.");
    }
    std::string unknown_str;
    nh.getParam("unknown_interpretation", unknown_str);
    if (unknown_str == "lethal")
    {
      mode = UnknownInterpretation::LETHAL;
    }
    else if (unknown_str == "expensive")
    {
      mode = UnknownInterpretation::EXPENSIVE;
    }
    else if (unknown_str == "free")
    {
      mode = UnknownInterpretation::FREE;
    }
    else
    {
      ROS_ERROR("Unknown value for unknown_interpretation '%s'. Using expensive instead.", unknown_str.c_str());
      mode = UnknownInterpretation::EXPENSIVE;
    }
  }

  setConfiguration(static_cast<unsigned char>(neutral_cost), scale, mode);
}

void CostInterpreter::setConfiguration(const unsigned char neutral_cost, const float scale,
                                       const UnknownInterpretation mode)
{
  neutral_cost_ = neutral_cost;
  for (unsigned int i = 0; i < cached_costs_.size(); i++)
  {
    if (i == nav_core2::Costmap::NO_INFORMATION)
    {
      float c;
      switch (mode)
      {
      case UnknownInterpretation::LETHAL:
        c = LETHAL_COST_F;
        break;
      case UnknownInterpretation::EXPENSIVE:
        c = LETHAL_COST_F - 1.0;
        break;
      default:  // case FREE:
        c = neutral_cost_;
        break;
      }
      cached_costs_[i] = c;
    }
    else if (i <= LETHAL_COST - 1)
    {
      float c = i * scale + neutral_cost_;
      cached_costs_[i] = std::min(c, LETHAL_COST_F - 1.0f);
    }
    else
    {
      cached_costs_[i] = LETHAL_COST_F;
    }
  }
}


}  // namespace dlux_global_planner
