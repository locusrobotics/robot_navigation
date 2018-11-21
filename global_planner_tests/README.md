# global_planner_tests

This package provides an array of tests for implementations of the `nav_core2::GlobalPlanner` interface. By making this package a `test_depend` of your implementation, you can easily write tests that run a fairly comprehensive suite of tests for your planner.

## Example
For the highest view of a planner's functionality, simply run a test against `global_planner_tests::many_map_test_suite`.

``` c++
#include <global_planner_tests/many_map_test_suite.h>
#include <your_planner/your_planner.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <string>

TEST(YourPlanner, simple_planner_test)
{
  TFListenerPtr tf = std::make_shared<tf2_ros::Buffer>();
  your_planner::YourPlanner planner;
  EXPECT_TRUE(global_planner_tests::many_map_test_suite(planner, tf, "your_planner_namespace"));
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_tests");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
```

## The Tests
This package contains a collection of image files (in the [maps](maps) directory) that are used as static maps. For most of the maps, the following tests will be run.
 * Check if a plan is generated from every free cell in the map to every other free cell in the map (`checkValidPathCoverage()`)
 * Check if planning to or from cells that are occupied in the costmap results in the appropriate exception being thrown. (`checkOccupiedPathCoverage()`)
 * Check if planning to or from cells outside the costmap results in the appropriate exception being thrown. (`checkOutOfBoundsPathCoverage()`)

Collectively, these tests are run with `hasCompleteCoverage()`

The one exception is with the `nopaths.png` map, which instead will
 * Check if the planning from any of the free cells to any other free cell results in the appropriate exception being thrown. (`hasNoPaths()`)

You can run all the tests listed above with the appropriate maps using the `many_map_test_suite` mentioned in the example, or you can customize the maps and functions called using the `global_planner_tests` library directly.


## Executables
For debugging, this package provides three executable nodes.
 * `gpt_node` - Runs `hasCompleteCoverage()` with a planner loaded from `pluginlib`. The map used is also configurable on the parameter server.
 * `many_map_node` - Runs `many_map_test_suite` with a planner loaded from `pluginlib`
 * `heatmap_node` - Runs a variation on `checkValidPathCoverage()` that prints a heatmap to the console with which cells in the map were frequently unable to get paths planned to/from them. Useful for annoying corner cases where your planner fails. Definitely NOT speaking from experience.

 ### Example Heatmap
```
...............
.. 9         ..
.             .
.             .
.  .     .    .
.             .
.             .
.             .
.             .
. .        .  .
.  .      .   .
.   ......    .
.             .
..      44   ..
...............
```
#### Legend:
 * `.` - Cell is an obstacle
 * `(space)` - No problems planning to or from this cell
 * `(#)` - If the digit in the cell is `X`, greater than `10*X` percent of the failures came to/from this cell. In the above example, 100% of the failures involved the top cell with a `9`, and 50% involved each of the bottom cells with `4`s. Its a little convoluted, but such are the limitations of ASCII art. And it still gets the point across.
