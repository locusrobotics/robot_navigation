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

#ifndef NAV_GRID_COORDINATE_CONVERSION_H
#define NAV_GRID_COORDINATE_CONVERSION_H

#include <nav_grid/nav_grid_info.h>
#include <math.h>

namespace nav_grid
{
/**
 * @brief  Convert from grid coordinates to world coordinates of the center of the cell
 *
 * The resulting coordinates are for the center of the grid cell.
 *
 * @param[in]  mx The x grid coordinate
 * @param[in]  my The y grid coordinate
 * @param[out] wx Set to the associated x world coordinate
 * @param[out] wy Set to the associated y world coordinate
 */
inline void gridToWorld(const NavGridInfo& info, int mx, int my, double& wx, double& wy)
{
  wx = info.origin_x + (mx + 0.5) * info.resolution;
  wy = info.origin_y + (my + 0.5) * info.resolution;
}

/**
 * @brief  Convert from world coordinates to the precise (double) grid coordinates
 *
 * The results are not rounded, so that the values can be used for locating a position within a cell
 *
 * @param[in]  wx The x world coordinate
 * @param[in]  wy The y world coordinate
 * @param[out] mx Set to the associated x grid coordinate
 * @param[out] my Set to the associated y grid coordinate
 */
inline void worldToGrid(const NavGridInfo& info, double wx, double wy, double& mx, double& my)
{
  mx = (wx - info.origin_x) / info.resolution;
  my = (wy - info.origin_y) / info.resolution;
}

/**
 * @brief  Convert from world coordinates to grid coordinates without checking for legal bounds
 * @param[in]  wx The x world coordinate
 * @param[in]  wy The y world coordinate
 * @param[out] mx Set to the associated x grid coordinate
 * @param[out] my Set to the associated y grid coordinate
 * @note       The returned grid coordinates <b>are not guaranteed to lie within the grid.</b>
 */
inline void worldToGrid(const NavGridInfo& info, double wx, double wy, int& mx, int& my)
{
  double dmx, dmy;
  worldToGrid(info, wx, wy, dmx, dmy);
  mx = static_cast<int>(floor(dmx));
  my = static_cast<int>(floor(dmy));
}

/**
 * @brief  Convert from world coordinates to grid coordinates
 *
 * Combined functionality from costmap_2d::worldToMap and costmap_2d::worldToMapEnforceBounds.
 * The output parameters are set to grid indexes within the grid, even if the function returns false,
 * meaning the coordinates are outside the grid.
 *
 * @param[in]  wx The x world coordinate
 * @param[in]  wy The y world coordinate
 * @param[out] mx Set to the associated (bounds-enforced) x grid coordinate
 * @param[out] my Set to the associated (bounds-enforced) y grid coordinate
 * @return     True if the input coordinates were within the grid
 */
inline bool worldToGridBounded(const NavGridInfo& info, double wx, double wy, unsigned int& mx, unsigned int& my)
{
  double dmx, dmy;
  worldToGrid(info, wx, wy, dmx, dmy);

  bool valid = true;

  if (dmx < 0.0)
  {
    mx = 0;
    valid = false;
  }
  else if (dmx >= info.width)
  {
    mx = info.width - 1;
    valid = false;
  }
  else
  {
    mx = static_cast<unsigned int>(dmx);
  }

  if (dmy < 0.0)
  {
    my = 0;
    valid = false;
  }
  else if (dmy >= info.height)
  {
    my = info.height - 1;
    valid = false;
  }
  else
  {
    my = static_cast<unsigned int>(dmy);
  }

  return valid;
}

/**
 * @brief Check to see if the world coordinates are within the grid.
 *
 * This should only be used if the caller does not need the associated grid coordinates. Otherwise it would
 * be more efficient to call worldToGridBounded.
 *
 * @param[in]  wx The x world coordinate
 * @param[in]  wy The y world coordinate
 * @return     True if the input coordinates were within the grid
 */
inline bool isWithinGrid(const NavGridInfo& info, double wx, double wy)
{
  wx -= info.origin_x;
  wy -= info.origin_y;
  return wx >= 0.0 &&
         wy >= 0.0 &&
         wx < info.width * info.resolution &&
         wy < info.height * info.resolution;
}



}  // namespace nav_grid

#endif  // NAV_GRID_COORDINATE_CONVERSION_H
