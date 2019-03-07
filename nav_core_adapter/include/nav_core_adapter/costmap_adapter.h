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

#ifndef NAV_CORE_ADAPTER_COSTMAP_ADAPTER_H
#define NAV_CORE_ADAPTER_COSTMAP_ADAPTER_H

#include <nav_core2/common.h>
#include <nav_core2/costmap.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <string>

namespace nav_core_adapter
{
nav_grid::NavGridInfo infoFromCostmap(costmap_2d::Costmap2DROS* costmap_ros);

class CostmapAdapter : public nav_core2::Costmap
{
public:
  /**
   * @brief Deconstructor for possibly freeing the costmap_ros_ object
   */
  virtual ~CostmapAdapter();

  /**
   * @brief Initialize from an existing Costmap2DROS object
   * @param costmap_ros A Costmap2DROS object
   * @param needs_destruction Whether to free the costmap_ros object when this class is destroyed
   */
  void initialize(costmap_2d::Costmap2DROS* costmap_ros, bool needs_destruction = false);

  // Standard Costmap Interface
  void initialize(const ros::NodeHandle& parent, const std::string& name, TFListenerPtr tf) override;
  nav_core2::Costmap::mutex_t* getMutex() override;

  // NavGrid Interface
  void reset() override;
  void update() override;
  void setValue(const unsigned int x, const unsigned int y, const unsigned char& value) override;
  unsigned char getValue(const unsigned int x, const unsigned int y) const override;
  void setInfo(const nav_grid::NavGridInfo& new_info) override;
  void updateInfo(const nav_grid::NavGridInfo& new_info) override;

  // Get Costmap Pointer for Backwards Compatibility
  costmap_2d::Costmap2DROS* getCostmap2DROS() const { return costmap_ros_; }

protected:
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;
  bool needs_destruction_;
};

}  // namespace nav_core_adapter

#endif  // NAV_CORE_ADAPTER_COSTMAP_ADAPTER_H
