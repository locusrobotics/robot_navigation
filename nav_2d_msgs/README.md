# nav_2d_msgs

This package has basic message types for two dimensional navigation. Many of the messages are similar to those in `geometry_msgs` or `nav_msgs` but are streamlined to only be concerned with 2.5 dimensional `(x, y, theta)` navigation. This eliminates quaternions from the messages in many places, which make calculations faster and avoids that particular headache when only one orientation value is needed for 2.5D navigation.

## Points and Poses
 * [`Point2D`](msg/Point2D.msg) - like `geometry_msgs::Point` but just `x` and `y` (no `z`)
 * [`Pose2DStamped`](msg/Pose2DStamped.msg) - `geometry_msgs::Pose2D` with a header
 * [`Pose2D32`](msg/Pose2D32.msg) - `(x, y, theta)` with only 32bit floating point precision.
 * [`Path2D`](msg/Path2D.msg) - An array of `Pose2D` with a header. Similar to `nav_msgs::Path` but without redundant headers.

## Polygons
 * [`Polygon2D`](msg/Polygon2D.msg) - Like `geometry_msgs::Polygon` but with 64 bit precision and no `z` coordinate.
 * [`Polygon2DStamped`](msg/Polygon2DStamped.msg) - above with a header
 * [`ComplexPolygon2D`](msg/ComplexPolygon2D.msg) - Non-simple Polygon2D, i.e. polygon with inner holes
 * [`Polygon2DCollection`](msg/Polygon2DCollection.msg) - A list of complex polygons, with a header and an optional parallel list of colors.

## Twists
 * [`Twist2D`](msg/Twist2D.msg) - Like `geometry_msgs::Twist` but only `(x, y, theta)` (and not separated into linear and angular)
 * [`Twist2DStamped`](msg/Twist2DStamped.msg) - above with a header
 * [`Twist2D32`](msg/Twist2D32.msg) - `(x, y, theta)` with only 32bit floating point precision.

## NavGrids
 * [`NavGridInfo`](msg/NavGridInfo.msg) - Same data as `nav_grid::NavGridInfo`. Similar to `nav_msgs::MapMetadata`
 * [`NavGridOfChars`](msg/NavGridOfChars.msg) - Data for `nav_grid::NavGrid<unsigned char>`. Similar to `nav_msgs::OccupancyGrid`
 * [`NavGridOfDoubles`](msg/NavGridOfDoubles.msg) - Data for `nav_grid::NavGrid<double>`
 * [`NavGridOfCharsUpdate`](msg/NavGridOfCharsUpdate.msg) and [`NavGridOfDoublesUpdate`](msg/NavGridOfDoublesUpdate.msg) - Similar to `map_msgs::OccupancyGridUpdate`
 * [`UIntBounds`](msg/UIntBounds.msg) - Same data as `nav_core2::UIntBounds`. Used in both `Update` messages.

## Service
 * [`SwitchPlugin`](srv/SwitchPlugin.srv) - A simple service equivalent to [SetString.srv](https://discourse.ros.org/t/suggestions-for-std-srvs/1079) use by the PluginMux.