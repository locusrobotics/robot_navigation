# nav_2d_utils Polygons and Footprints
This library represents a replacement for [costmap_2d/footprint.h](https://github.com/ros-planning/navigation/blob/a2837b5a9dc6dd4b4da176fca7d899d6a3722bf8/costmap_2d/include/costmap_2d/footprint.h) and deals with manipulating polygons. Note that implicitly all polygons here are assumed to be [simple polygons](https://en.wikipedia.org/wiki/Simple_polygon) without "holes."

## Polygons and the Parameter Server
There have historically been three primary ways to specify a polygon/footprint on the parameter server. The first is to simply specify a radius which is converted to a hexadecagon, i.e. polygon with sixteen sides. This can be read with
```
nav_2d_msgs::Polygon2D polygonFromRadius(const double radius, const unsigned int num_points = 16);
```

The second two ways involve specifying the points of the polygon individually. This can be done with either a string representing a bracketed array of arrays of doubles, `"[[1.0, 2.2], [3.3, 4.2], ...]"`. This can be read with

```
nav_2d_msgs::Polygon2D polygonFromString(const std::string& polygon_string);
```

Alternatively, with ROS, you can read the points directly from the parameter server in the form of an `XmlRpcValue`, which should be an array of arrays of doubles, which is read with

```
nav_2d_msgs::Polygon2D polygonFromXMLRPC(XmlRpc::XmlRpcValue& polygon_xmlrpc);
```

If the `XmlRpcValue` is a string, it will call the `polygonFromString` method.

The above are the traditional methods that were supported by the original `costmap_2d` code. However, we add a fourth method that requires two parallel arrays of x and y coordinates.

```
nav_2d_msgs::Polygon2D polygonFromParallelArrays(const std::vector<double>& xs, const std::vector<double>& ys);
```

All of the above methods (except the radius one) can be loaded as appropriate from the parameter server with
```
nav_2d_msgs::Polygon2D polygonFromParams(const ros::NodeHandle& nh, const std::string parameter_name,
                                         bool search = true);
```
to include the radius, use the logic in `footprint.h` which either uses "footprint" or "robot_radius"
```
nav_2d_msgs::Polygon2D footprintFromParams(ros::NodeHandle& nh, bool write = true);
```

Polygons can be written to parameters with
```
void polygontoParams(const nav_2d_msgs::Polygon2D& polygon, const ros::NodeHandle& nh, const std::string parameter_name,
                     bool array_of_arrays = true);
```

## Polygon Operations
There are also a handful of methods for examining/manipulating polygons
 * `equals` - check if two polygons are equal
 * `movePolygonToPose` - translate and rotate a polygon
 * `isInside` - check if a point is inside a polygon
 * `calculateMinAndMaxDistances` - Calculate the minimum and maximum distance from the origin of a polygon
 * `triangulate` - Decompose a polygon into a set of non-overlapping triangles using an open source implementation of the [earcut algorithm](https://github.com/mapbox/earcut.hpp)
