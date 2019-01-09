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
#include <costmap_queue/costmap_queue.h>
#include <algorithm>
#include <vector>

namespace costmap_queue
{

CostmapQueue::CostmapQueue(nav_core2::Costmap& costmap, bool manhattan) :
  MapBasedQueue(false), costmap_(costmap), seen_(0), manhattan_(manhattan), cached_max_distance_(-1)
{
  reset();
}

void CostmapQueue::reset()
{
  seen_.setInfo(costmap_.getInfo());
  seen_.reset();
  computeCache();
  MapBasedQueue::reset();
}

void CostmapQueue::enqueueCell(unsigned int x, unsigned int y)
{
  enqueueCell(x, y, x, y);
}

void CostmapQueue::enqueueCell(unsigned int cur_x, unsigned int cur_y,
                               unsigned int src_x, unsigned int src_y)
{
  if (seen_(cur_x, cur_y)) return;

  // we compute our distance table one cell further than the inflation radius dictates so we can make the check below
  double distance = distanceLookup(cur_x, cur_y, src_x, src_y);
  CellData data(distance, cur_x, cur_y, src_x, src_y);
  if (validCellToQueue(data))
  {
    seen_.setValue(cur_x, cur_y, 1);
    enqueue(distance, data);
  }
}

CellData CostmapQueue::getNextCell()
{
  // get the highest priority cell and pop it off the priority queue
  CellData current_cell = front();
  pop();

  unsigned int mx = current_cell.x_;
  unsigned int my = current_cell.y_;
  unsigned int sx = current_cell.src_x_;
  unsigned int sy = current_cell.src_y_;

  // attempt to put the neighbors of the current cell onto the queue
  if (mx > 0)
    enqueueCell(mx - 1, my, sx, sy);
  if (my > 0)
    enqueueCell(mx, my - 1, sx, sy);
  if (mx < costmap_.getWidth() - 1)
    enqueueCell(mx + 1, my, sx, sy);
  if (my < costmap_.getHeight() - 1)
    enqueueCell(mx, my + 1, sx, sy);

  return current_cell;
}

void CostmapQueue::computeCache()
{
  int max_distance = getMaxDistance();
  if (max_distance == cached_max_distance_) return;
  cached_distances_.clear();

  cached_distances_.resize(max_distance + 2);

  for (unsigned int i = 0; i < cached_distances_.size(); ++i)
  {
    cached_distances_[i].resize(max_distance + 2);
    for (unsigned int j = 0; j < cached_distances_[i].size(); ++j)
    {
      if (manhattan_)
      {
        cached_distances_[i][j] = i + j;
      }
      else
      {
        cached_distances_[i][j] = hypot(i, j);
      }
    }
  }
  cached_max_distance_ = max_distance;
}

}  // namespace costmap_queue
