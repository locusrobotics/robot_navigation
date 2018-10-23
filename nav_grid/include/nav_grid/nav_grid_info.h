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

#ifndef NAV_GRID_NAV_GRID_INFO_H
#define NAV_GRID_NAV_GRID_INFO_H

#include <string>

namespace nav_grid
{
/**
 * @struct NavGridInfo
 * This class defines a way to discretize a finite section of the world into a grid.
 * It contains similar information to the ROS msg nav_msgs/MapMetaData (aka the info field of nav_msgs/OccupancyGrid)
 * except the map_load_time is removed, the geometry is simplified from a Pose to xy coordinates, and the frame_id
 * is added.
 */
struct NavGridInfo
{
public:
  /* All data is publically accessible */
  unsigned int width = 0;
  unsigned int height = 0;
  double resolution = 1.0;
  std::string frame_id = "map";
  double origin_x = 0.0;  ///< The origin defines the coordinates of minimum corner of cell (0,0) in the grid
  double origin_y = 0.0;

  /**
   * @brief comparison operator that requires all fields are equal
   */
  bool operator == (const NavGridInfo& info) const
  {
    return width == info.width && height == info.height && resolution == info.resolution &&
           origin_x == info.origin_x && origin_y == info.origin_y && frame_id == info.frame_id;
  }

  bool operator != (const NavGridInfo& info) const
  {
    return !operator==(info);
  }

  /**
   * @brief String representation of this object
   */
  std::string toString() const
  {
    return std::to_string(width) + "x" + std::to_string(height) + " (" + std::to_string(resolution) + "res) " +
      frame_id + " " + std::to_string(origin_x) + " " + std::to_string(origin_y);
  }
};

inline std::ostream& operator<<(std::ostream& stream, const NavGridInfo& info)
{
  stream << info.toString();
  return stream;
}

}  // namespace nav_grid

#endif  // NAV_GRID_NAV_GRID_INFO_H
