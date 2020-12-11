# robot_nav_viz_demos

Demos for testing/demonstrating `robot_nav_rviz_plugins` and `color_util` packages

## Color Util Demos
To see a simple tool for visualizing `color_util` blending, run
`roslaunch robot_nav_viz_demos spectrum_demo.launch`. To see what all the named colors look like, run `roslaunch robot_nav_viz_demos named_colors_demo.launch`

## Nav Grid Demo
`roslaunch robot_nav_viz_demos grid_demo.launch`

This will launch two processes.
 * `pong` is a simple script for generating `NavGridOfDoubles` and `NavGridOfDoublesUpdate` data with an ever-expanding range of values.
   * Note that `0` is set as the Ignore Value, which means that it is set to transparent (with some palettes) and the min/max range ignores that value.
 * The second plays back the data from `demo_grids.bag` which contains `NavGridOfChars` and `NavGridOfCharsUpdate` data.
   * The bag data only contains 6 values (by design) and thus requires custom palettes to fully appreciate. There are two contained within `demo_palettes.cpp`, called `mega` and `green`. This demonstrates the capabilities of the `pluginlib` approach for the palettes.

## Polygons Demo
`roslaunch robot_nav_viz_demos polygons.launch`

This shows off the new Polygon-based display types.
 * In the center, is a single Polygon2DStamped in the form of a simple star. Like all of the Polygon displays here, you can choose to display the outline, the filled in polygon, or both, as well as customize colors for each.
 * Outside of that is a Polygon2DCollection, i.e. an array of possibly Complex polygons (i.e. polygons with holes). Each of the rings represents a polygon with an outer and inner outline. There are three different fill-color styles you can choose. By default, it shows everything in one color. You can also switch it to use color information from the message, which should display a rainbow. You can also choose a set of unique colors from the `color_util` package if the color information is not included in the message.
 * The small star spinning farthest away has the type `geometry_msgs::PolygonStamped`. It is the same as the standard `rviz::PolygonDisplay` except it can also be filled in. You can enable the old-style polygon display for comparison.
