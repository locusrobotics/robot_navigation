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

#ifndef NAV_CORE_ADAPTER_SHARED_POINTERS_H
#define NAV_CORE_ADAPTER_SHARED_POINTERS_H

#include <nav_core2/common.h>
#include <memory>

namespace nav_core_adapter
{

template <typename T>
void null_deleter(T* raw_ptr) {}

/**
 * @brief Custom Constructor for creating a shared pointer to an existing object that doesn't delete the ptr when done
 *
 * @note This is considered bad form, and is only done here for backwards compatibility purposes. The nav_core2
 *       interfaces require shared pointers, but the creation of the shared pointer from a raw pointer takes ownership
 *       of the object such that when the object containing the shared pointer is freed, the object pointed at by the
 *       shared pointer is also freed. This presents a problem, for instance, when switching from one planner to another
 *       if the costmap is freed by one planner.
 *
 * @param raw_ptr The raw pointer to an object
 * @return A Shared pointer pointing at the raw_ptr, but when it is freed, the raw_ptr remains valid
 */
template <typename T>
std::shared_ptr<T> createSharedPointerWithNoDelete(T* raw_ptr)
{
  return std::shared_ptr<T>(raw_ptr, null_deleter<T>);
}

}  // namespace nav_core_adapter

#endif  // NAV_CORE_ADAPTER_SHARED_POINTERS_H
