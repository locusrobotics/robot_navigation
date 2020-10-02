cmake_minimum_required(VERSION 3.0.2)
project(nav_grid_pub_sub)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        map_msgs
        nav_2d_msgs
        nav_2d_utils
        nav_core2
        nav_grid
        nav_grid_iterators
        nav_msgs
        roscpp
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        map_msgs
        nav_2d_msgs
        nav_2d_utils
        nav_core2
        nav_grid
        nav_grid_iterators
        nav_msgs
        roscpp
    INCLUDE_DIRS include
    LIBRARIES nav_grid_pub_sub
)
include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_library(nav_grid_pub_sub src/cost_interpretation_tables.cpp)
target_link_libraries(nav_grid_pub_sub ${catkin_LIBRARIES})
add_dependencies(nav_grid_pub_sub ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
    find_package(roslint REQUIRED)
    roslint_cpp()
    roslint_add_test()
endif()

install(
    TARGETS nav_grid_pub_sub
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
    DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
