# nav_2d_utils
A handful of useful utility functions for nav_core2 packages.
 * Bounds - Utilities for `nav_core2::Bounds` objects interacting with other messages/types and for dividing a `UIntBounds` into multiple smaller bounds.
 * [Conversions](doc/Conversions.md) - Tools for converting between `nav_2d_msgs` and other types.
 * OdomSubscriber - subscribes to the standard `nav_msgs::Odometry` message and provides access to the velocity component as a `nav_2d_msgs::Twist`
 * Parameters - a couple ROS parameter patterns
 * PathOps - functions for working with `nav_2d_msgs::Path2D` objects (beyond strict conversion)
 * [Plugin Mux](doc/PluginMux.md) - tool for switching between multiple `pluginlib` plugins
 * [Polygons and Footprints](doc/PolygonsAndFootprints.md) - functions for working with `Polygon2D` objects
 * TF Help - Tools for transforming `nav_2d_msgs` and other common operations.
