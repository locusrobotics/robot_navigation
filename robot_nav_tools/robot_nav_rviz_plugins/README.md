# robot_nav_rviz_plugins
RViz visualizations for robot_navigation datatypes


## PathDisplay
This is a simple port of `rviz::PathDisplay` but to work with `nav_2d_msgs::Path2D`.

## Polygon Displays
The existing [`rviz::PolygonDisplay`](https://github.com/ros-visualization/rviz/blob/noetic-devel/src/rviz/default_plugin/polygon_display.cpp) draws only the outline of a given polygon, and cannot fill the polygon in with color. This package has three new RViz displays for polygon data:
 * `robot_nav_rviz_plugins::Polygon3DDisplay` will display `geometry_msgs/PolygonStamped` messages just like `rviz::PolygonDisplay` except it can fill in the polygon.
 * `robot_nav_rviz_plugins::PolygonDisplay` displays `nav_2d_msgs/Polygon2DStamped` messages
 * `robot_nav_rviz_plugins::PolygonsDisplay` (note the S in PolygonS) will display `nav_2d_msgs/Polygon2DCollection` messages.

Each has three display modes, for displaying just the outline, just the filler, or both.

The behavior is showcased by running `roslaunch robot_nav_viz_demos polygons.launch`.
