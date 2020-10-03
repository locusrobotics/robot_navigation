/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, Locus Robotics
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

#ifndef NAV_2D_UTILS_BOUNDS_H
#define NAV_2D_UTILS_BOUNDS_H

#include <nav_grid/nav_grid_info.h>
#include <nav_core2/bounds.h>
#include <vector>

/**
 * @brief A set of utility functions for Bounds objects interacting with other messages/types
 */
namespace nav_2d_utils
{

/**
 * @brief return a floating point bounds that covers the entire NavGrid
 * @param info Info for the NavGrid
 * @return bounds covering the entire NavGrid
 */
nav_core2::Bounds getFullBounds(const nav_grid::NavGridInfo& info);

/**
 * @brief return an integral bounds that covers the entire NavGrid
 * @param info Info for the NavGrid
 * @return bounds covering the entire NavGrid
 */
nav_core2::UIntBounds getFullUIntBounds(const nav_grid::NavGridInfo& info);

/**
 * @brief Translate real-valued bounds to uint coordinates based on nav_grid info
 * @param info Information used when converting the coordinates
 * @param bounds floating point bounds object
 * @returns corresponding UIntBounds for parameter
 */
nav_core2::UIntBounds translateBounds(const nav_grid::NavGridInfo& info, const nav_core2::Bounds& bounds);

/**
 * @brief Translate uint bounds to real-valued coordinates based on nav_grid info
 * @param info Information used when converting the coordinates
 * @param bounds UIntBounds object
 * @returns corresponding floating point bounds for parameter
 */
nav_core2::Bounds translateBounds(const nav_grid::NavGridInfo& info, const nav_core2::UIntBounds& bounds);

}  // namespace nav_2d_utils

#endif  // NAV_2D_UTILS_BOUNDS_H
