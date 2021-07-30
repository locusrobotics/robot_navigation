## The robot_navigation Stack
### 2.5D Navigation in ROS

## Available Packages:

### Core Interaces
 * `nav_grid` - A templatized interface for overlaying a two dimensional grid on the world.
 * `nav_core2` - Core Costmap and Planner Interfaces
 * `nav_2d_msgs` - Basic message types for two and a half dimensional navigation.

### Local Planning
 * `dwb_local_planner` - The core planner logic and plugin interfaces.
 * `dwb_msgs` - ROS Interfaces for interacting with the dwb local planner.
 * `dwb_plugins` - Plugin implementations for velocity iteration and trajectory generation
 * `dwb_critics` - Critic plugin implementations needed for replicating behavior of dwa

### Global Planning
 * `dlux_global_planner` - The core planner logic and plugin interfaces.
 * `dlux_plugins` - Plugin implementations for dlux global planner interfaces.
 * `global_planner_tests` - Collection of tests for checking the validity and completeness of global planners.

### Planner Coordination
 * `locomotor` - Extensible path planning coordination engine that controls what happens when the global and local planners succeed and fail
 * `locomotor_msgs` - An action definition for Locomotor and other related messages
 * `locomove_base` - Extension of Locomotor that replicates `move_base`'s functionality.

### Utilities
 * `nav_2d_utils` - Message conversions, etc.
 * `nav_grid_iterators` - Iterator implementations for moving around the cells of a `nav_grid` in a number of common patterns.
 * `nav_grid_pub_sub` - Publishers and Subscribers for `nav_grid` data.
 * `costmap_queue` - Tool for iterating through the cells of a costmap to find the closest distance to a subset of cells.

### Backwards Compatibility
 * `nav_core_adapter` - Adapters between `nav_core` and `nav_core2`.

## ROS Buildfarm

|         | source | binary |
|---------|--------|--------|
| melodic | [![Build Status](http://build.ros.org/view/Msrc_uB/job/Msrc_uB__robot_navigation__ubuntu_bionic__source/badge/icon?style=flat-square)](http://build.ros.org/view/Msrc_uB/job/Msrc_uB__robot_navigation__ubuntu_bionic__source/) | [![Build Status](http://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__robot_navigation__ubuntu_bionic_amd64__binary/badge/icon?style=flat-square)](http://build.ros.org/view/Mbin_uB64/job/Mbin_uB64__robot_navigation__ubuntu_bionic_amd64__binary/)|
| noetic  | [![Build Status](http://build.ros.org/view/Nsrc_uF/job/Nsrc_uF__robot_navigation__ubuntu_focal__source/badge/icon?style=flat-square)](http://build.ros.org/view/Nsrc_uF/job/Nsrc_uF__robot_navigation__ubuntu_focal__source/) | [![Build Status](http://build.ros.org/view/Nbin_uF64/job/Nbin_uF64__robot_navigation__ubuntu_focal_amd64__binary/badge/icon?style=flat-square)](http://build.ros.org/view/Nbin_uF64/job/Nbin_uF64__robot_navigation__ubuntu_focal_amd64__binary/)|
