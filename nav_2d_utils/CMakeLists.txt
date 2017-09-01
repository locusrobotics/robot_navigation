cmake_minimum_required(VERSION 2.8.3)
project(nav_2d_utils)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED
    COMPONENTS geometry_msgs nav_2d_msgs nav_core2 nav_msgs roscpp tf xmlrpcpp
)

catkin_package(
    CATKIN_DEPENDS geometry_msgs nav_2d_msgs nav_core2 nav_msgs roscpp tf xmlrpcpp
    INCLUDE_DIRS include
    LIBRARIES conversions path_ops
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_library(conversions src/conversions.cpp)
target_link_libraries(conversions ${catkin_LIBRARIES})

add_library(path_ops src/path_ops.cpp)
target_link_libraries(path_ops ${catkin_LIBRARIES})
