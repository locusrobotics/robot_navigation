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

#include <nav_core_adapter/costmap_adapter.h>
#include <nav_core2/exceptions.h>
#include <pluginlib/class_list_macros.h>
#include <string>

PLUGINLIB_EXPORT_CLASS(nav_core_adapter::CostmapAdapter, nav_core2::Costmap)

namespace nav_core_adapter
{

nav_grid::NavGridInfo infoFromCostmap(costmap_2d::Costmap2DROS* costmap_ros)
{
  nav_grid::NavGridInfo info;
  costmap_2d::Costmap2D* costmap = costmap_ros->getCostmap();
  info.width = costmap->getSizeInCellsX();
  info.height = costmap->getSizeInCellsY();
  info.resolution = costmap->getResolution();
  info.frame_id = costmap_ros->getGlobalFrameID();
  info.origin_x = costmap->getOriginX();
  info.origin_y = costmap->getOriginY();
  return info;
}

CostmapAdapter::~CostmapAdapter()
{
  if (needs_destruction_)
  {
    delete costmap_ros_;
  }
}

void CostmapAdapter::initialize(costmap_2d::Costmap2DROS* costmap_ros, bool needs_destruction)
{
  costmap_ros_ = costmap_ros;
  needs_destruction_ = needs_destruction;
  info_ = infoFromCostmap(costmap_ros_);
  costmap_ = costmap_ros_->getCostmap();
}

void CostmapAdapter::initialize(const ros::NodeHandle& parent, const std::string& name, TFListenerPtr tf)
{
  initialize(new costmap_2d::Costmap2DROS(name, *tf), true);
}

nav_core2::Costmap::mutex_t* CostmapAdapter::getMutex()
{
  return costmap_->getMutex();
}

void CostmapAdapter::reset()
{
  costmap_ros_->resetLayers();
}

void CostmapAdapter::update()
{
  info_ = infoFromCostmap(costmap_ros_);
  if (!costmap_ros_->isCurrent())
    throw nav_core2::CostmapDataLagException("Costmap2DROS is out of date somehow.");
}

void CostmapAdapter::setValue(const unsigned int x, const unsigned int y, const unsigned char& value)
{
  costmap_->setCost(x, y, value);
}

unsigned char CostmapAdapter::getValue(const unsigned int x, const unsigned int y) const
{
  unsigned int index = costmap_->getIndex(x, y);
  return costmap_->getCharMap()[index];
}

void CostmapAdapter::setInfo(const nav_grid::NavGridInfo& new_info)
{
  throw nav_core2::CostmapException("setInfo not implemented on CostmapAdapter");
}

void CostmapAdapter::updateInfo(const nav_grid::NavGridInfo& new_info)
{
  costmap_->updateOrigin(new_info.origin_x, new_info.origin_y);
}

}  // namespace nav_core_adapter
