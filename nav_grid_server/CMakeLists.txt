cmake_minimum_required(VERSION 3.0.2)
project(nav_grid_server)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS
    nav_2d_utils
    nav_grid
    nav_grid_iterators
    nav_grid_pub_sub
    nav_msgs
    roscpp
)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)

catkin_package(
    DEPENDS OpenCV
    CATKIN_DEPENDS nav_2d_utils nav_grid nav_grid_iterators nav_grid_pub_sub nav_msgs roscpp
    INCLUDE_DIRS include
    LIBRARIES image_loader
)

include_directories(
    include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${YAMLCPP_INCLUDE_DIRS}
)

add_library(image_loader src/image_loader.cpp)
target_link_libraries(image_loader ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(server src/server.cpp)
target_link_libraries(server image_loader ${catkin_LIBRARIES}
    ${YAMLCPP_LIBRARIES})

add_executable(saver src/saver.cpp)
target_link_libraries(saver ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

if (CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  find_package(roslib REQUIRED)
  find_package(map_server REQUIRED)
  catkin_add_gtest(map_server_comparison test/map_server_comparison.cpp)
  target_link_libraries(map_server_comparison image_loader ${catkin_LIBRARIES} ${roslib_LIBRARIES} ${map_server_LIBRARIES})

  roslint_cpp()
  roslint_add_test()
endif()

install(
    DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
    TARGETS image_loader
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(TARGETS server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
