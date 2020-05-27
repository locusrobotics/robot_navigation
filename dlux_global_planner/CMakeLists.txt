cmake_minimum_required(VERSION 3.0.2)
project(dlux_global_planner)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")
find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_2d_msgs
        nav_2d_utils
        nav_core2
        nav_grid
        nav_grid_pub_sub
        nav_msgs
        pluginlib
        roscpp
        visualization_msgs
)
catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        nav_2d_msgs
        nav_2d_utils
        nav_core2
        nav_grid
        nav_grid_pub_sub
        nav_msgs
        pluginlib
        roscpp
        visualization_msgs
    INCLUDE_DIRS
        include
    LIBRARIES
        dgp
)

add_library(dgp src/dlux_global_planner.cpp src/cost_interpreter.cpp)
add_dependencies(dgp ${catkin_EXPORTED_TARGETS})
target_link_libraries(dgp ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_planner_node src/planner_node.cpp)
add_dependencies(${PROJECT_NAME}_planner_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_planner_node dgp ${catkin_LIBRARIES})
set_target_properties(${PROJECT_NAME}_planner_node PROPERTIES OUTPUT_NAME planner_node PREFIX "")

include_directories(include ${catkin_INCLUDE_DIRS})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()

  find_package(rostest REQUIRED)
  catkin_add_gtest(kernel_test test/kernel_test.cpp)
  target_link_libraries(kernel_test dgp)
endif()

install(TARGETS ${PROJECT_NAME}_planner_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS dgp
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES nav_core2_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
