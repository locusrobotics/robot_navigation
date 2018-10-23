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

#include <global_planner_tests/easy_costmap.h>
#include <global_planner_tests/util.h>
#include <map_server/image_loader.h>
#include <string>
#include <vector>
#include <algorithm>

namespace global_planner_tests
{

EasyCostmap::EasyCostmap(const std::string& filename, const double resolution, const bool origin_at_center)
{
  loadMapFromFile(filename, resolution, origin_at_center);
}

void EasyCostmap::loadMapFromFile(const std::string& filename, const double resolution, const bool origin_at_center)
{
  double origin[3] = {0.0, 0.0, 0.0};
  nav_msgs::GetMap::Response map_resp;
  map_server::loadMapFromFile(&map_resp, resolve_filename(filename).c_str(), resolution, true, 0.0, 0.0, origin, RAW);

  if (origin_at_center)
  {
    map_resp.map.info.origin.position.x = map_resp.map.info.width * resolution / -2;
    map_resp.map.info.origin.position.y = map_resp.map.info.height * resolution / -2;
  }
  original_grid_ = map_resp.map;
  original_grid_.header.frame_id = "map";
  reset();
}

void EasyCostmap::reset()
{
  nav_grid::NavGridInfo new_info;
  new_info.width = original_grid_.info.width;
  new_info.height = original_grid_.info.height;
  new_info.resolution = original_grid_.info.resolution;
  new_info.frame_id = original_grid_.header.frame_id;
  new_info.origin_x = original_grid_.info.origin.position.x;
  new_info.origin_y = original_grid_.info.origin.position.y;

  if (info_ != new_info)
  {
    info_ = new_info;
    data_.resize(info_.width * info_.height);
  }
  for (unsigned int i=0; i < data_.size(); i++)
  {
    data_[i] = static_cast<unsigned char>(original_grid_.data[i]);
  }
}

}  // namespace global_planner_tests
