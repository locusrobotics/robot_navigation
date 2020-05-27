cmake_minimum_required(VERSION 3.0.2)
project(locomotor)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
  actionlib geometry_msgs locomotor_msgs nav_2d_msgs nav_2d_utils nav_core2 nav_msgs pluginlib roscpp rospy
)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        actionlib
        geometry_msgs
        locomotor_msgs
        nav_2d_msgs
        nav_2d_utils
        nav_core2
        nav_msgs
        pluginlib
        roscpp
        rospy
    LIBRARIES locomotor
)

add_library(
    locomotor
        src/locomotor.cpp
        src/executor.cpp
        src/publishers.cpp
        src/locomotor_action_server.cpp
)
target_link_libraries(
    locomotor ${catkin_LIBRARIES}
)

add_executable(locomotor1 src/single_thread_locomotor.cpp)
target_link_libraries(
    locomotor1 locomotor ${catkin_LIBRARIES}
)

add_executable(locomotor2 src/double_thread_locomotor.cpp)
target_link_libraries(
    locomotor2 locomotor ${catkin_LIBRARIES}
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS locomotor1 locomotor2
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS locomotor
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
catkin_install_python(PROGRAMS scripts/send_action.py DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
