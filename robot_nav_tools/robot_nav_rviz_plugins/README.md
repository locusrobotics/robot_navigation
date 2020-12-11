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

## NavGridDisplay
This is a robust refactoring of `rviz::MapDisplay` to not only support `nav_msgs::OccupancyGrid` but also the `nav_2d_msgs::NavGridOfChars` and `nav_2d_msgs::NavGridOfDoubles` datatypes. There are couple of noteworthy added features.

### Dynamic Color Support
`rviz::MapDisplay` had three hard-coded color schemes for coloring pixels based on the map data.
 * **Map** Gray scale for `[0, 100]`, green for `[101, 127]`, red/yellow for `[128, 254]` and bluish gray for `255`.
 * **Costmap**, Invisible for `0`, cyan for `99`, purple for `100`, bluish-gray for `255`, and then the same green/red/yellow as **Map** for `[101, 254]`.
 * **Raw** Gray scale for `[0, 255]`

The new `NavGridDisplay` types have those color schemes implemented, but also allows for arbitrary other mappings of colors using `pluginlib`. Just define an extension of `robot_nav_rviz_plugins::NavGridPalette` with a unique name and list of as many as 256 colors.

### Dynamic Value Scaling
In the past, if you wanted to display a grid of values, you would need to either convert it into an `OccupancyGrid` and use the hard-coded color schemes above, or you would need to convert it to a `PointCloud2` and use a Color Transform. The `NavGridOfDoublesDisplay` allows you to publish unbounded double values, and the scale will be dynamically calculated, so the minimum values appear on one side of the `NavGridPalette` and the maximum on the other, even as the extremes change.

Furthermore, there is an option to ignore particular values.
 * If you don't want the value `-1` to be included, you can set the `Ignore Value Type` to `Value`, and the `Ignore Value` to `-1`, and then `-1` will not be included in the dynamic bounds.
 * If you don't want any values above 2000 to be included, you can set the `Ignore Value Type` to `Limit` and the `Ignore Value` to `2000` and then any values greater than or equal to 2000 will be ignored.
