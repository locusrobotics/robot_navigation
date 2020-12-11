cmake_minimum_required(VERSION 3.0.2)
project(robot_nav_rviz_plugins)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
    color_util
    geometry_msgs
    nav_2d_msgs
    nav_2d_utils
    nav_core2
    nav_grid
    nav_grid_iterators
    nav_grid_pub_sub
    nav_msgs
    pluginlib
    roscpp
    rviz
    std_msgs
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
        color_util
        geometry_msgs
        nav_2d_msgs
        nav_2d_utils
        nav_core2
        nav_grid
        nav_grid_iterators
        nav_grid_pub_sub
        nav_msgs
        pluginlib
        roscpp
        rviz
        std_msgs
    LIBRARIES robot_nav_rviz_plugins basic_palettes rnrp_utils
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(rnrp_utils src/spectrum_palette.cpp)
target_link_libraries(rnrp_utils ${catkin_LIBRARIES})

qt5_wrap_cpp(MOC_FILES
  include/robot_nav_rviz_plugins/nav_grid_display.h
  include/robot_nav_rviz_plugins/path_display.h
  include/robot_nav_rviz_plugins/polygon_display.h
  include/robot_nav_rviz_plugins/polygon3d_display.h
  include/robot_nav_rviz_plugins/polygons_display.h
)

add_library(robot_nav_rviz_plugins
    src/ogre_panel.cpp
    src/partial_ogre_panel.cpp
    src/nav_grid_display.cpp
    src/nav_grid_of_chars_display.cpp
    src/nav_grid_of_doubles_display.cpp
    src/occupancy_grid_display.cpp
    src/path_display.cpp
    src/polygon_display.cpp
    src/polygon3d_display.cpp
    src/polygons_display.cpp
    src/polygon_parts.cpp
    ${MOC_FILES}
)
add_dependencies(robot_nav_rviz_plugins ${catkin_EXPORTED_TARGETS})
target_link_libraries(robot_nav_rviz_plugins rnrp_utils ${QT_LIBRARIES} ${catkin_LIBRARIES})

add_library(basic_palettes src/basic_palettes.cpp)
target_link_libraries(basic_palettes rnrp_utils ${QT_LIBRARIES} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
endif()

install(TARGETS robot_nav_rviz_plugins rnrp_utils basic_palettes
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES robot_nav_rviz_plugins.xml rviz_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY icons/classes/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons/classes
)
