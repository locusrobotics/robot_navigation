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

#include <locomotor/executor.h>
#include <memory>

namespace locomotor
{
Executor::Executor(const ros::NodeHandle& base_nh, bool create_cb_queue)
  : ex_nh_(base_nh)
{
  if (create_cb_queue)
  {
    queue_ = std::make_shared<ros::CallbackQueue>();
    ros::CallbackQueue* raw_queue_ptr = queue_.get();

    spinner_ = std::make_shared<ros::AsyncSpinner>(1, raw_queue_ptr);
    spinner_->start();

    ex_nh_.setCallbackQueue(raw_queue_ptr);
  }
  else
  {
    queue_ = nullptr;
    spinner_ = nullptr;
  }
}

const ros::NodeHandle& Executor::getNodeHandle() const
{
  return ex_nh_;
}

void Executor::addCallback(LocomotorCallback::Function f)
{
  getQueue().addCallback(boost::make_shared<LocomotorCallback>(f));
}

ros::CallbackQueue& Executor::getQueue()
{
  if (queue_) return *queue_;
  return *ros::getGlobalCallbackQueue();
}

}  // namespace locomotor
