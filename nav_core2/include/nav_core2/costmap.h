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

#ifndef NAV_CORE2_COSTMAP_H
#define NAV_CORE2_COSTMAP_H

#include <nav_grid/nav_grid.h>
#include <nav_core2/common.h>
#include <nav_core2/bounds.h>
#include <boost/thread.hpp>
#include <memory>
#include <string>

namespace nav_core2
{

class Costmap : public nav_grid::NavGrid<unsigned char>
{
public:
  static const unsigned char NO_INFORMATION = 255;
  static const unsigned char LETHAL_OBSTACLE = 254;
  static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
  static const unsigned char FREE_SPACE = 0;

  using Ptr = std::shared_ptr<Costmap>;

  /**
   * @brief Virtual Destructor
   */
  virtual ~Costmap() {}

  /**
   * @brief  Initialization function for the Costmap
   *
   * ROS parameters/topics are expected to be in the parent/name namespace.
   * It is suggested that all NodeHandles in the costmap use the parent NodeHandle's callback queue.
   *
   * @param  parent NodeHandle to derive other NodeHandles from
   * @param  name The namespace for the costmap
   * @param  tf A pointer to a transform listener
   */
  virtual void initialize(const ros::NodeHandle& parent, const std::string& name, TFListenerPtr tf) {}

  inline unsigned char getCost(const unsigned int x, const unsigned int y)
  {
    return getValue(x, y);
  }

  inline unsigned char getCost(const nav_grid::Index& index)
  {
    return getValue(index.x, index.y);
  }

  inline void setCost(const unsigned int x, const unsigned int y, const unsigned char cost)
  {
    setValue(x, y, cost);
  }

  inline void setCost(const nav_grid::Index& index, const unsigned char cost)
  {
    setValue(index, cost);
  }

  /**
   * @brief Update the values in the costmap
   *
   * Note that this method can throw CostmapExceptions to indicate problems, like when it would be unsafe to navigate.
   * e.g. If the costmap expects laser data at a given rate, but laser data hasn't shown up in a while, this method
   * might throw a CostmapDataLagException.
   */
  virtual void update() {}

  using mutex_t = boost::recursive_mutex;
  /**
   * @brief Accessor for boost mutex
   */
  virtual mutex_t* getMutex() = 0;

  /**
   * @brief Flag to indicate whether this costmap is able to track how much has changed
   */
  virtual bool canTrackChanges() { return false; }

  /**
   * @brief If canTrackChanges, get the bounding box for how much of the costmap has changed
   *
   * Rather than querying based on time stamps (which can require an arbitrary amount of storage)
   * we instead query based on a namespace. The return bounding box reports how much of the costmap
   * has changed since the last time this method was called with a particular namespace. If a namespace
   * is new, then it returns a bounding box for the whole costmap. The max values are inclusive.
   *
   * Example Sequence with a 5x5 costmap: (results listed (min_x,min_y):(max_x, max_y))
   *    0) getChangeBounds("A") --> (0,0):(4,4)
   *    1) getChangeBounds("B") --> (0,0):(4,4)
   *    2) update cell 1, 1
   *    3) getChangeBounds("C") --> (0,0):(4,4)
   *    4) getChangeBounds("A") --> (1,1):(1,1)
   *    5) getChangeBounds("A") --> (empty bounds)    (i.e. nothing was updated since last call)
   *    6) updateCell 2, 4
   *    7) getChangeBounds("A") --> (2,4):(2,4)
   *    8) getChangeBounds("B") --> (1,1):(2,4)
   *
   * @param ns The namespace
   * @return Updated bounds
   * @throws std::runtime_error If canTrackChanges is false, the returned bounds would be meaningless
   */
  virtual UIntBounds getChangeBounds(const std::string& ns)
  {
    if (!canTrackChanges())
    {
      throw std::runtime_error("You called 'getChangeBounds()' on a derived Costmap type that is not capable of "
                               "tracking changes (i.e. canTrackChanges() returns false). You shouldn't do that.");
    }
    else
    {
      throw std::runtime_error("You called 'getChangeBounds()' on a derived Costmap type that is capable of tracking "
                               "changes but has not properly implemented this function. You should yell at the author "
                               "of the derived Costmap.");
    }
    return UIntBounds();
  }
};
}  // namespace nav_core2

#endif  // NAV_CORE2_COSTMAP_H
