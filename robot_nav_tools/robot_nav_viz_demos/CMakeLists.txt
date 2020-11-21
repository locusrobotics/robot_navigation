cmake_minimum_required(VERSION 3.0.2)
project(robot_nav_viz_demos)

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
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()
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
