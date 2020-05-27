cmake_minimum_required(VERSION 3.0.2)
project(dwb_critics)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  angles
  costmap_queue
  dwb_local_planner
  geometry_msgs
  nav_2d_msgs
  nav_2d_utils
  nav_core2
  nav_grid_iterators
  pluginlib
  roscpp
  sensor_msgs
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS angles costmap_queue dwb_local_planner geometry_msgs nav_2d_msgs nav_2d_utils nav_core2 nav_grid_iterators pluginlib roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/alignment_util.cpp
    src/map_grid.cpp
    src/goal_dist.cpp
    src/path_dist.cpp
    src/goal_align.cpp
    src/path_align.cpp
    src/base_obstacle.cpp
    src/obstacle_footprint.cpp
    src/oscillation.cpp
    src/prefer_forward.cpp
    src/rotate_to_goal.cpp
    src/twirling.cpp
)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES default_critics.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
