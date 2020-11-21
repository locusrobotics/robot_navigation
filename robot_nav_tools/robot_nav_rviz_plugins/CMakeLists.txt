cmake_minimum_required(VERSION 3.0.2)
project(robot_nav_rviz_plugins)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_2d_msgs
    nav_2d_utils
    pluginlib
    rviz
)

set(CMAKE_AUTOMOC ON)

find_package(Qt5 REQUIRED COMPONENTS
  Core
  Widgets
)
set(QT_LIBRARIES Qt5::Widgets)

add_definitions(-DQT_NO_KEYWORDS)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        nav_2d_msgs
        nav_2d_utils
        pluginlib
        rviz
    LIBRARIES robot_nav_rviz_plugins
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

qt5_wrap_cpp(MOC_FILES
  include/robot_nav_rviz_plugins/path_display.h
)

add_library(robot_nav_rviz_plugins
    src/path_display.cpp
    ${MOC_FILES}
)
add_dependencies(robot_nav_rviz_plugins ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_nav_rviz_plugins ${QT_LIBRARIES} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS robot_nav_rviz_plugins
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES rviz_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY icons/classes/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons/classes
)
