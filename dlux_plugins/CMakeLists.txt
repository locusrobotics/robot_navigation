cmake_minimum_required(VERSION 3.0.2)
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

  find_package(global_planner_tests REQUIRED)
  include_directories(include ${global_planner_tests_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${GTEST_INCLUDE_DIRS})

  add_rostest_gtest(planner_test test/planner_test.launch test/planner_test.cpp)
  target_link_libraries(planner_test ${global_planner_tests_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  # full_planner_test contains many parameter variations and does not need to be run regularly
  # add_rostest_gtest(full_planner_test test/full_planner_test.launch test/full_planner_test.cpp)
  # target_link_libraries(full_planner_test ${global_planner_tests_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  add_rostest_gtest(got test/global_oscillation_test.launch test/global_oscillation_test.cpp)
  target_link_libraries(got ${global_planner_tests_LIBRARIES} ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

  find_package(roslint REQUIRED)
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
    DIRECTORY test
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
