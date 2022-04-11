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

#include <global_planner_tests/many_map_test_suite.h>
#include <global_planner_tests/global_planner_tests.h>
#include <global_planner_tests/easy_costmap.h>
#include <global_planner_tests/util.h>
#include <yaml-cpp/yaml.h>
#include <memory>
#include <string>
#include <vector>

namespace global_planner_tests
{
bool many_map_test_suite(nav_core2::GlobalPlanner& planner, TFListenerPtr tf,
                         const std::string& planner_name, const std::string& maps_list_filename,
                         bool check_exception_type, bool verbose, bool quit_early)
{
  // Initialize Costmap
  ros::NodeHandle nh("~");
  std::shared_ptr<EasyCostmap> easy_costmap = std::make_shared<EasyCostmap>();
  planner.initialize(nh, planner_name, tf, easy_costmap);

  // Read Configuration
  YAML::Node config = YAML::LoadFile(resolve_filename(maps_list_filename));
  std::string prefix = "";
  int max_failure_cases = 10;
  if (config["standard_prefix"])
  {
    prefix = config["standard_prefix"].as<std::string>();
  }
  if (config["max_failure_cases"])
  {
    max_failure_cases = config["max_failure_cases"].as<int>();
  }

  // Start the timer
  struct timeval start, end;
  double start_t, end_t, t_diff;
  gettimeofday(&start, nullptr);

  bool passes_all = true;

  // Check all the Full Coverage Maps
  for (const auto& base_filename : config["full_coverage_maps"].as<std::vector<std::string> >())
  {
    std::string map_filename = prefix + base_filename;
    ROS_INFO("Testing full coverage map \"%s\"", map_filename.c_str());
    easy_costmap->loadMapFromFile(map_filename);

    bool ret = global_planner_tests::hasCompleteCoverage(planner, *easy_costmap, max_failure_cases,
                                                         check_exception_type, verbose, quit_early);
    if (quit_early && !ret) return ret;
    passes_all &= ret;
  }

  // Check all the No Coverage Maps
  for (const auto& base_filename : config["no_coverage_maps"].as<std::vector<std::string> >())
  {
    std::string map_filename = prefix + base_filename;
    ROS_INFO("Testing no coverage map \"%s\"", map_filename.c_str());
    easy_costmap->loadMapFromFile(map_filename);

    bool invalid_test = global_planner_tests::hasNoPaths(planner, *easy_costmap,
                                                         check_exception_type, verbose, quit_early);
    if (quit_early && !invalid_test) return invalid_test;
    passes_all &= invalid_test;
  }

  gettimeofday(&end, nullptr);
  start_t = start.tv_sec + static_cast<double>(start.tv_usec) / 1e6;
  end_t = end.tv_sec + static_cast<double>(end.tv_usec) / 1e6;
  t_diff = end_t - start_t;
  ROS_INFO("%s Full Planning Time: %.9f", planner_name.c_str(), t_diff);

  return passes_all;
}
}  // namespace global_planner_tests
