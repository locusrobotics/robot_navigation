cmake_minimum_required(VERSION 3.0.2)
project(color_util)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED COMPONENTS std_msgs)

catkin_package(
    CATKIN_DEPENDS std_msgs
    INCLUDE_DIRS include
    LIBRARIES color_util
)
include_directories(include ${catkin_INCLUDE_DIRS})

add_library(color_util src/convert.cpp)
target_link_libraries(color_util ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_cpp()
  roslint_add_test()

  catkin_add_gtest(utest test/utest.cpp)
  target_link_libraries(utest color_util ${catkin_LIBRARIES})
  add_dependencies(utest ${catkin_EXPORTED_TARGETS} ${color_util_EXPORTED_TARGETS})
endif()

install(TARGETS color_util
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
