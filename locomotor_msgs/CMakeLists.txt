cmake_minimum_required(VERSION 3.0.2)
project(locomotor_msgs)

find_package(catkin REQUIRED
    COMPONENTS actionlib_msgs message_generation nav_2d_msgs
)

add_message_files(FILES
    NavigationState.msg
    ResultCode.msg
)
add_action_files(
    FILES NavigateToPose.action
)
generate_messages(
    DEPENDENCIES actionlib_msgs nav_2d_msgs
)

catkin_package(
    CATKIN_DEPENDS actionlib_msgs message_runtime nav_2d_msgs
)

