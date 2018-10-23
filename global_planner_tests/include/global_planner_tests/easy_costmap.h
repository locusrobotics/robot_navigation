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

#ifndef GLOBAL_PLANNER_TESTS_EASY_COSTMAP_H
#define GLOBAL_PLANNER_TESTS_EASY_COSTMAP_H

#include <nav_core2/basic_costmap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <string>

namespace global_planner_tests
{
/**
 * @class EasyCostmap
 * @brief An instantiation of the Costmap class that simply populates the grid from an image.
 */
class EasyCostmap : public nav_core2::BasicCostmap
{
public:
  /**
   * @brief Constructor, loads directly from an image filename
   * @param filename The filename for the image to load
   * @param resolution Rather than loading from a yaml file, you can specify the resolution here
   * @param origin_at_center If true, will put the origin of the map at the center, rather than the bottom left
   */
  explicit EasyCostmap(const std::string& filename, const double resolution = 0.1, const bool origin_at_center = false);

  /**
   * @brief Empty constructor. You need to call loadMapFromFile afterward
   */
  EasyCostmap() { reset(); }

  // NavGrid Interface
  void reset() override;

  // Main Image Loading Logic
  void loadMapFromFile(const std::string& filename, const double resolution = 0.1, const bool origin_at_center = false);

protected:
  nav_msgs::OccupancyGrid original_grid_;
};  // class EasyCostmap
}  // namespace global_planner_tests

#endif  // GLOBAL_PLANNER_TESTS_EASY_COSTMAP_H
