cmake_minimum_required(VERSION 2.8.3)
project(robot_nav_viz_demos)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
    color_util
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
        color_util
        pluginlib
        rviz
    LIBRARIES spectrum_display
    INCLUDE_DIRS include
)

include_directories(include ${catkin_INCLUDE_DIRS})

qt5_wrap_cpp(MOC_FILES
  include/robot_nav_viz_demos/spectrum_display.h
)

add_library(spectrum_display
    src/spectrum_display.cpp
    ${MOC_FILES}
)
add_dependencies(spectrum_display ${catkin_EXPORTED_TARGETS})
target_link_libraries(spectrum_display ${QT_LIBRARIES} ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslaunch roslint)
  roslint_cpp()
  roslint_add_test()
  roslaunch_add_file_check(launch)
endif()

install(TARGETS spectrum_display
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(FILES rviz_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES data/spectrum.rviz
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/data
)
install(FILES launch/spectrum_demo.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
