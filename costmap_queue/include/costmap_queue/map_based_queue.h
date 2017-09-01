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

#ifndef COSTMAP_QUEUE_MAP_BASED_QUEUE_H
#define COSTMAP_QUEUE_MAP_BASED_QUEUE_H

#include <map>
#include <vector>

namespace costmap_queue
{
/**
 * @brief Templatized interface for a priority queue
 *
 * This is faster than the std::priority_queue implementation because iterating does
 * not require resorting after every element is examined.
 * Based on https://github.com/ros-planning/navigation/pull/525
 */
template <class item_t>
class MapBasedQueue
{
public:
  /**
   * @brief Default Constructor
   */
  MapBasedQueue() : iterator_initialized_(false), item_count_(0)
  {
  }

  /**
   * @brief Clear the queue
   */
  virtual void reset()
  {
    iterator_initialized_ = false;
    item_bins_.clear();
    item_count_ = 0;
  }

  /**
   * @brief Add a new item to the queue with a set priority
   * @param priority Priority of the item
   * @param item Payload item
   */
  void enqueue(const double priority, item_t item)
  {
    item_bins_[priority].push_back(item);
    item_count_++;
  }

  /**
   * @brief Check to see if there is anything in the queue
   * @return True if there is nothing in the queue
   *
   * Must be called prior to front/pop.
   */
  bool isEmpty()
  {
    return item_count_ == 0;
  }

  /**
   * @brief Return the item at the front of the queue
   * @return The item at the front of the queue
   */
  item_t& front()
  {
    precheck();
    return iter_->second.back();
  }

  /**
   * @brief Remove (and destroy) the item at the front of the queue
   */
  void pop()
  {
    precheck();
    iter_->second.pop_back();
    item_count_--;
    if (iter_ != item_bins_.end() && iter_->second.empty())
    {
      ++iter_;
    }
  }
private:
  /**
   * @brief Setup required before looking at or changing first item in queue
   */
  inline void precheck()
  {
    if (!iterator_initialized_)
    {
      iter_ = item_bins_.begin();
      iterator_initialized_ = true;
    }
  }

  std::map<double, std::vector<item_t> > item_bins_;
  bool iterator_initialized_;
  unsigned int item_count_;
  typename std::map<double, std::vector<item_t> >::iterator iter_;
};
}  // namespace costmap_queue

#endif  // COSTMAP_QUEUE_MAP_BASED_QUEUE_H
