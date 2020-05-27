cmake_minimum_required(VERSION 3.0.2)
project(nav_grid_iterators)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  nav_grid nav_2d_msgs nav_2d_utils nav_msgs roscpp
)

catkin_package(
    CATKIN_DEPENDS nav_grid nav_2d_msgs nav_2d_utils nav_msgs roscpp
    INCLUDE_DIRS include
    LIBRARIES nav_grid_iterators
)

add_library(nav_grid_iterators
  src/whole_grid.cpp
  src/sub_grid.cpp
  src/circle_fill.cpp
  src/circle_outline.cpp
  src/spiral.cpp
  src/bresenham.cpp
  src/ray_trace.cpp
  src/line.cpp
  src/polygon_outline.cpp
  src/polygon_fill.cpp
)

target_link_libraries(
    nav_grid_iterators ${catkin_LIBRARIES}
)

add_executable(demo demo/demo.cpp)
add_dependencies(demo ${catkin_EXPORTED_TARGETS})
target_link_libraries(demo nav_grid_iterators ${catkin_LIBRARIES})

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()

  catkin_add_gtest(line_tests test/line_tests.cpp)
  target_link_libraries(line_tests nav_grid_iterators)
  catkin_add_gtest(${PROJECT_NAME}_utest test/utest.cpp)
  target_link_libraries(${PROJECT_NAME}_utest nav_grid_iterators)
endif (CATKIN_ENABLE_TESTING)

install(TARGETS nav_grid_iterators
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
