cmake_minimum_required(VERSION 2.8.3)
project(nav_grid)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11")

find_package(catkin)

catkin_package(
    INCLUDE_DIRS include
)

if (CATKIN_ENABLE_TESTING)
  find_package(roslint)
  include_directories(include ${catkin_INCLUDE_DIRS})
  roslint_cpp()
  roslint_add_test()
  catkin_add_gtest(utest test/utest.cpp)
endif (CATKIN_ENABLE_TESTING)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
