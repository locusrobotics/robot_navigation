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

#ifndef GLOBAL_PLANNER_TESTS_GLOBAL_PLANNER_TESTS_H
#define GLOBAL_PLANNER_TESTS_GLOBAL_PLANNER_TESTS_H

#include <nav_core2/global_planner.h>
#include <string>
#include <vector>
#include <utility>

namespace global_planner_tests
{

using PoseList = std::vector<nav_2d_msgs::Pose2DStamped>;

/**
 * @brief Populate two lists of poses from a costmap. one with all the free cells, the other with all the occupied cells
 * @param costmap Source of the cells
 * @param free_cells Output parameter with one pose in every free cell of the costmap
 * @param occupied_cells Output parameter with one pose in every occupied cell of the costmap
 * @param include_edges True if you want to include the cells on the border of the costmap
 *
 * For simplicity, the edge cells are ignored, meaning planners need not implement robust boundary checking
 */
void groupCells(const nav_core2::Costmap& costmap, PoseList& free_cells, PoseList& occupied_cells,
                bool include_edges = true);

/**
 * @brief Create a list of poses that are outside the costmap's bounds
 * @param costmap Source of the bounds data
 * @return vector of four poses that are outside of the costmap's bounds
 */
PoseList createPosesOutsideCostmap(const nav_core2::Costmap& costmap);

/**
 * @brief Create a new list of poses from cells that is num_cells long
 * @param cells Source list of cells
 * @param num_cells Number of cells that should be in the return list
 * @return vector of poses populated from cells
 */
PoseList subsetPoseList(const PoseList& cells, unsigned int num_cells);

/**
 * @brief Simple check to see if a plan exists. Returns a boolean instead of returning path or throwing an exception
 * @param planner The planner
 * @param start Starting pose
 * @param goal Goal pose
 */
bool planExists(nav_core2::GlobalPlanner& planner, nav_2d_msgs::Pose2DStamped start, nav_2d_msgs::Pose2DStamped goal);

/**
 * @brief Check if a path exists between every pair of free_cells
 * @param planner The planner
 * @param free_cells List of poses to check
 * @param verbose If true, will print statistics at the end
 * @param quit_early Will stop running tests once there is one failure
 * @return False if there is a single failure
 */
bool checkValidPathCoverage(nav_core2::GlobalPlanner& planner, const PoseList& free_cells,
                            bool verbose = false, bool quit_early = true);

/**
 * @brief Check if the appropriate exception is thrown when attempting to plan to or from an occupied cell
 *
 * This attempts plans from every start_cell to every goal_cell. We want to test three different modes
 *    * Occupied cell to free cell
 *    * Free cell to occupied cell
 *    * Occupied cell to occupied cell
 *
 * In the first case, the start cell is invalid (invalid_starts=true) and we expect OccupiedStartException to be thrown
 * In the second, the goal cell is invalid (invalid_starts=false) and we expect OccupiedGoalException
 * In the third, both are invalid, so we don't check the precise exception type (check_exception_type=false)
 *
 * @param planner The planner
 * @param start_cells poses to try to plan from
 * @param goal_cells poses to try to plan to
 * @param test_name name of this test configuration for verbose printing
 * @param check_exception_type If true, requires that OccupiedStartException/OccupiedGoalException thrown appropriately
 * @param verbose If true, will print statistics at the end
 * @param quit_early Will stop running tests once there is one failure
 * @param invalid_starts Whether the start poses are invalid. Otherwise its the goals
 * @return False if there is a single failure
 */
bool checkOccupiedPathCoverage(nav_core2::GlobalPlanner& planner,
                               const PoseList& start_cells, const PoseList& goal_cells,
                               const std::string& test_name,
                               bool check_exception_type = true, bool verbose = false, bool quit_early = true,
                               bool invalid_starts = true);

/**
 * @brief Check if the appropriate exception is thrown when attempting to plan to or from a pose off the costmap
 *
 * This attempts plans from every start_cell to every goal_cell. We want to test three different modes
 *    * OutOfBounds cell to free cell
 *    * Free cell to OutOfBounds cell
 *    * OutOfBounds cell to OutOfBounds cell
 *
 * In the first case, the start cell is invalid (invalid_starts=true) and we expect StartBoundsException to be thrown
 * In the second, the goal cell is invalid (invalid_starts=false) and we expect GoalBoundsException
 * In the third, both are invalid, so we don't check the precise exception type (check_exception_type=false)
 *
 * @param planner The planner
 * @param start_cells poses to try to plan from
 * @param goal_cells poses to try to plan to
 * @param test_name name of this test configuration for verbose printing
 * @param check_exception_type If true, requires that StartBoundsException/GoalBoundsException thrown appropriately
 * @param verbose If true, will print statistics at the end
 * @param quit_early Will stop running tests once there is one failure
 * @param invalid_starts Whether the start poses are invalid. Otherwise its the goals
 * @return False if there is a single failure
 */
bool checkOutOfBoundsPathCoverage(nav_core2::GlobalPlanner& planner,
                                  const PoseList& start_cells, const PoseList& goal_cells,
                                  const std::string& test_name,
                                  bool check_exception_type = true, bool verbose = false, bool quit_early = true,
                                  bool invalid_starts = true);

/**
 * @brief Run a bunch of tests, assuming there is a valid path from any free cell to any other free cell
 *
 * Calls checkValidPathCoverage(), checkOccupiedPathCoverage() and checkOutOfBoundsPathCoverage()
 *
 * The first checks if there is a path between every set of free cells. It's likely a bit of overkill to
 * check for the exceptions getting thrown for every combination of free/occupied/OutOfBounds cells. Hence,
 * for the latter two, we limit our tests to a subset of the poses. If N=max_failure_cases, we check between
 * combinations of N free cells, N occupied cells and 4 OutOfBounds cells.
 *
 * We also allow for none of the tests to check the exact exception type in case you want to use this to
 * test a nav_core planner with the nav_core_adapter (which just throws the generic PlannerException whenever
 * path planning fails)
 *
 * @param planner The planner
 * @param costmap Costmap to get poses from
 * @param max_failure_cases Number of poses to limit our lists to when checking for path planning failures
 * @param check_exception_type If true, requires the correct exception thrown
 * @param verbose If true, will print statistics at the end
 * @param quit_early Will stop running tests once there is one failure
 * @return False if there is a single failure
 */
bool hasCompleteCoverage(nav_core2::GlobalPlanner& planner, const nav_core2::Costmap& costmap,
                         int max_failure_cases = 10, bool check_exception_type = true,
                         bool verbose = false, bool quit_early = true);

/**
 * @brief Run a bunch of tests, assuming there are no valid paths from any free cell to any other free cell
 *
 * @param planner The planner
 * @param costmap Costmap to get poses from
 * @param check_exception_type If true, requires the correct exception thrown
 * @param verbose If true, will print statistics at the end
 * @param quit_early Will stop running tests once there is one failure
 * @return False if there is a single failure
 */
bool hasNoPaths(nav_core2::GlobalPlanner& planner, const nav_core2::Costmap& costmap,
                bool check_exception_type = true, bool verbose = false, bool quit_early = true);
}  // namespace global_planner_tests

#endif  // GLOBAL_PLANNER_TESTS_GLOBAL_PLANNER_TESTS_H
