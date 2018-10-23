cmake_minimum_required(VERSION 2.8.3)
project(dlux_plugins)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED
    COMPONENTS dlux_global_planner nav_core2 nav_grid pluginlib
)

catkin_package(
    CATKIN_DEPENDS dlux_global_planner nav_core2 nav_grid pluginlib
    INCLUDE_DIRS include
    LIBRARIES dlux_plugins
)

add_library(dlux_plugins src/dijkstra.cpp src/astar.cpp src/grid_path.cpp src/gradient_path.cpp src/von_neumann_path.cpp)
target_link_libraries(dlux_plugins ${catkin_LIBRARIES})
include_directories(
    include ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(catkin REQUIRED COMPONENTS global_planner_tests dlux_global_planner roslint)
  include_directories(include ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})
  link_directories(${GTEST_LIBRARY_DIRS})

  add_executable(planner_test test/planner_test.cpp)
  target_link_libraries(planner_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  add_executable(full_planner_test test/full_planner_test.cpp)
  target_link_libraries(full_planner_test ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  # full_planner_test is a dependency just to ensure it compiles
  add_rostest(test/planner_test.launch DEPENDENCIES planner_test full_planner_test dlux_plugins)

  roslint_cpp()
  roslint_add_test()
endif()

install(
    TARGETS dlux_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
    DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
    FILES dlux_global_planner_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(
    FILES common.yaml node_test.launch planner_test.launch rviz.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/test
)
