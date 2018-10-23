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

#ifndef NAV_CORE2_BASIC_COSTMAP_H
#define NAV_CORE2_BASIC_COSTMAP_H

#include <nav_core2/costmap.h>
#include <string>
#include <vector>

namespace nav_core2
{
class BasicCostmap : public nav_core2::Costmap
{
public:
  // Standard Costmap Interface
  mutex_t* getMutex() override { return &my_mutex_; }

  // NavGrid Interface
  void reset() override;
  void setValue(const unsigned int x, const unsigned int y, const unsigned char& value) override;
  unsigned char getValue(const unsigned int x, const unsigned int y) const override;
  void setInfo(const nav_grid::NavGridInfo& new_info) override
  {
    info_ = new_info;
    reset();
  }

  // Index Conversion
  unsigned int getIndex(const unsigned int x, const unsigned int y) const;
protected:
  mutex_t my_mutex_;
  std::vector<unsigned char> data_;
};
}  // namespace nav_core2

#endif  // NAV_CORE2_BASIC_COSTMAP_H
