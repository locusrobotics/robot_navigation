cmake_minimum_required(VERSION 2.8.3)
project(dwb_local_planner)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  dwb_msgs
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  nav_core2
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf
  visualization_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES dwb_local_planner debug_dwb_local_planner
    CATKIN_DEPENDS
        costmap_2d dwb_msgs geometry_msgs nav_2d_msgs nav_2d_utils nav_core2
        nav_msgs pluginlib roscpp sensor_msgs tf visualization_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

add_library(dwb_local_planner src/dwb_local_planner.cpp
                              src/backwards_compatibility.cpp
                              src/publisher.cpp
                              src/illegal_trajectory_tracker.cpp
)
target_link_libraries(dwb_local_planner ${catkin_LIBRARIES})

add_library(debug_dwb_local_planner src/debug_dwb_local_planner.cpp)
target_link_libraries(debug_dwb_local_planner ${catkin_LIBRARIES} dwb_local_planner)

add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node ${catkin_LIBRARIES} debug_dwb_local_planner)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()
