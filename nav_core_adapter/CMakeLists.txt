cmake_minimum_required(VERSION 2.8.3)
project(nav_core_adapter)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED
    COMPONENTS dwb_msgs geometry_msgs nav_2d_msgs nav_core nav_core2 nav_2d_utils nav_msgs pluginlib tf visualization_msgs
)

catkin_package(
    CATKIN_DEPENDS dwb_msgs geometry_msgs nav_2d_msgs nav_2d_utils nav_core nav_core2 nav_msgs pluginlib tf visualization_msgs
    INCLUDE_DIRS include
    LIBRARIES local_planner_adapter global_planner_adapter global_planner_adapter2
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_library(local_planner_adapter src/local_planner_adapter.cpp)
add_dependencies(local_planner_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_planner_adapter ${catkin_LIBRARIES})

add_library(global_planner_adapter src/global_planner_adapter.cpp)
add_dependencies(global_planner_adapter ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_planner_adapter ${catkin_LIBRARIES})

add_library(global_planner_adapter2 src/global_planner_adapter2.cpp)
add_dependencies(global_planner_adapter2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(global_planner_adapter2 ${catkin_LIBRARIES})
if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS local_planner_adapter global_planner_adapter global_planner_adapter2
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES nav_core2_plugins.xml nav_core_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
