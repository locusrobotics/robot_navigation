cmake_minimum_required(VERSION 2.8.3)
project(nav_2d_utils)

set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED
    COMPONENTS
        geometry_msgs
        nav_2d_msgs
        nav_core2
        nav_grid
        nav_msgs
        pluginlib
        roscpp
        std_msgs
        tf
        xmlrpcpp
)

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        nav_2d_msgs
        nav_core2
        nav_grid
        nav_msgs
        pluginlib
        roscpp
        std_msgs
        tf
        xmlrpcpp
    INCLUDE_DIRS include
    LIBRARIES conversions path_ops polygons
)

include_directories(
    include ${catkin_INCLUDE_DIRS}
)

add_library(conversions src/conversions.cpp)
target_link_libraries(conversions ${catkin_LIBRARIES})
add_dependencies(conversions ${catkin_EXPORTED_TARGETS})

add_library(path_ops src/path_ops.cpp)
target_link_libraries(path_ops ${catkin_LIBRARIES})
add_dependencies(path_ops ${catkin_EXPORTED_TARGETS})

add_library(polygons src/polygons.cpp src/footprint.cpp)
target_link_libraries(polygons ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(roslint)
  roslint_cpp()
  roslint_add_test()

  catkin_add_gtest(polygon_tests test/polygon_tests.cpp)
  target_link_libraries(polygon_tests polygons ${catkin_LIBRARIES})

  catkin_add_gtest(compress_test test/compress_test.cpp)
  target_link_libraries(compress_test path_ops ${catkin_LIBRARIES})

  add_rostest_gtest(param_tests test/param_tests.launch test/param_tests.cpp)
  target_link_libraries(param_tests polygons ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
endif()

install(TARGETS conversions path_ops polygons
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
