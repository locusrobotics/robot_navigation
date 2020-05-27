cmake_minimum_required(VERSION 3.0.2)
project(locomove_base)
set_directory_properties(PROPERTIES COMPILE_OPTIONS "-std=c++11;-Wall;-Werror")

find_package(catkin REQUIRED
    COMPONENTS locomotor move_base_msgs nav_2d_utils nav_core nav_core_adapter
)

catkin_package(
    CATKIN_DEPENDS locomotor move_base_msgs nav_2d_utils nav_core nav_core_adapter
    INCLUDE_DIRS include
)

add_executable(move_base src/locomove_base.cpp)
add_dependencies(move_base ${catkin_EXPORTED_TARGETS})
target_link_libraries(move_base ${catkin_LIBRARIES})
include_directories(
  include ${catkin_INCLUDE_DIRS}
)

if(CATKIN_ENABLE_TESTING)
  find_package(roslint REQUIRED)
  roslint_cpp()
  roslint_add_test()
endif()

install(
    TARGETS move_base
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
