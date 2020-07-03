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

#ifndef LOCOMOTOR_EXECUTOR_H
#define LOCOMOTOR_EXECUTOR_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <memory>

namespace locomotor
{
/**
 * @class LocomotorCallback
 * @brief Extension of ros::CallbackInterface so we can insert things on the ROS Callback Queue
 */
class LocomotorCallback : public ros::CallbackInterface
{
public:
  using Function = std::function<void()>;

  explicit LocomotorCallback(Function functor) : functor_(functor) {}
  ~LocomotorCallback() {}

  CallResult call()
  {
    functor_();
    return ros::CallbackInterface::Success;
  }
private:
  Function functor_;
};

/**
 * @class Executor
 * @brief Collection of objects used in ROS CallbackQueue threading.
 *
 * This light wrapper around the contained classes allows for intelligent cleanup of the objects
 */
class Executor
{
public:
  /**
   * @brief Constructor. Creates new CallbackQueue or uses global CallbackQueue
   * @param base_nh Base NodeHandle that this executor's NodeHandle will be derived from
   * @param create_cb_queue If true, creates a new CallbackQueue. If false, uses global CallbackQueue
   */
  explicit Executor(const ros::NodeHandle& base_nh, bool create_cb_queue = true);

  virtual ~Executor() {}

  /**
   * @brief Gets NodeHandle coupled with this executor's CallbackQueue
   */
  virtual const ros::NodeHandle& getNodeHandle() const;

  /**
   * @brief Add a callback to this executor's CallbackQueue
   * @param f LocomotorCallback
   */
  void addCallback(LocomotorCallback::Function f);
protected:
  /**
   * @brief Gets the queue for this executor
   */
  virtual ros::CallbackQueue& getQueue();

  std::shared_ptr<ros::CallbackQueue> queue_;
  std::shared_ptr<ros::AsyncSpinner> spinner_;
  ros::NodeHandle ex_nh_;
};
}  // namespace locomotor

#endif  // LOCOMOTOR_EXECUTOR_H
