cmake_minimum_required(VERSION 3.0.2)
project(global_planner_tests)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS map_server nav_core2 roscpp nav_msgs pluginlib)

catkin_package(
    CATKIN_DEPENDS map_server nav_core2 roscpp nav_msgs pluginlib
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
)

add_library(${PROJECT_NAME} src/global_planner_tests.cpp src/easy_costmap.cpp src/many_map_test_suite.cpp src/util.cpp)
target_link_libraries(
    ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp
)

add_executable(gpt_node src/gpt_node.cpp)
target_link_libraries(gpt_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(heatmap_node src/heatmap_node.cpp)
target_link_libraries(heatmap_node ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(many_map_node src/many_map_node.cpp)
target_link_libraries(many_map_node ${PROJECT_NAME} ${catkin_LIBRARIES})

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(
    DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
    TARGETS global_planner_tests
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS gpt_node heatmap_node many_map_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY maps/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/maps
)
install(FILES config/standard_tests.yaml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
