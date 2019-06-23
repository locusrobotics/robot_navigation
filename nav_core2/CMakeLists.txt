cmake_minimum_required(VERSION 2.8.3)
project(nav_core2)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

find_package(catkin REQUIRED
    COMPONENTS nav_2d_msgs nav_grid tf
)

catkin_package(
    CATKIN_DEPENDS nav_2d_msgs nav_grid tf
    INCLUDE_DIRS include
    LIBRARIES basic_costmap
)

add_library(basic_costmap src/basic_costmap.cpp)
target_link_libraries(
    basic_costmap ${catkin_LIBRARIES}
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()

  catkin_add_gtest(bounds_test test/bounds_test.cpp)
  catkin_add_gtest(exception_test test/exception_test.cpp)
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(
    TARGETS basic_costmap
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
