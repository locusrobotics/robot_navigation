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

#ifndef GLOBAL_PLANNER_TESTS_MANY_MAP_TEST_SUITE_H
#define GLOBAL_PLANNER_TESTS_MANY_MAP_TEST_SUITE_H

#include <nav_core2/global_planner.h>
#include <string>

namespace global_planner_tests
{
/**
 * @brief Run a collection of global_planner_tests on the provided maps
 *
 * @param planner The planner
 * @param tf Used when initializing the planner
 * @param planner_name Used when initializing the planner
 * @param maps_list_filename Filename to load the list of provided maps from
 * @param check_exception_type If true, requires the correct exception thrown
 * @param verbose If true, will print statistics at the end
 * @param quit_early Will stop running tests once there is one failure
 * @return False if there is a single failure
 */
bool many_map_test_suite(nav_core2::GlobalPlanner& planner, TFListenerPtr tf,
                         const std::string& planner_name,
                         const std::string& maps_list_filename =
                            "package://global_planner_tests/config/standard_tests.yaml",
                         bool check_exception_type = true, bool verbose = false, bool quit_early = true);
}  // namespace global_planner_tests

#endif  // GLOBAL_PLANNER_TESTS_MANY_MAP_TEST_SUITE_H
