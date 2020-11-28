cmake_minimum_required(VERSION 3.0.2)
project(nav_2d_msgs)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    message_generation
    std_msgs
)

add_message_files(FILES
    ComplexPolygon2D.msg
    NavGridInfo.msg
    NavGridOfChars.msg
    NavGridOfCharsUpdate.msg
    NavGridOfDoubles.msg
    NavGridOfDoublesUpdate.msg
    Path2D.msg
    Point2D.msg
    Polygon2D.msg
    Polygon2DCollection.msg
    Polygon2DStamped.msg
    Pose2D32.msg
    Pose2DStamped.msg
    Twist2D.msg
    Twist2D32.msg
    Twist2DStamped.msg
    UIntBounds.msg
)

add_service_files(FILES SwitchPlugin.srv)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
    CATKIN_DEPENDS geometry_msgs message_runtime std_msgs
)
