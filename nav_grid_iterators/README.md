# nav_grid_iterators

This package provides C++ iterators for iterating over some portion of a `NavGrid`. There are two sets. The first are signed line iterators which are not constrained to valid `NavGrid` indexes. Second, are the general iterators which are constrained to valid `NavGrid` indexes.

## Signed Line Iteration
As a building block for the general iterators, we provide two line iterators that iterate over `int` coordinates. Both take two pairs of coordinates for start and end points, as well as a boolean for whether to include the coordinates of the end point in the iteration.
 * [`Bresenham`](include/nav_grid_iterators/line/bresenham.h) takes integer coordinates as input and implements [Bresenham's line algorithm](https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm), which means that there is either only one cell per column or one cell per row.
 * [`RayTrace`](include/nav_grid_iterators/line/ray_trace.h) takes double coordinates as input and implements [Ray tracing](https://en.wikipedia.org/wiki/Ray_tracing_(graphics)), which means that it iterates over all cells that the line passes through, even if only briefly.

## General Iterators
 * [`WholeGrid`](include/nav_grid_iterators/whole_grid.h) iterates over every cell in the grid in row major order.
 * [`SubGrid`](include/nav_grid_iterators/sub_grid.h) iterates over a rectangular subportion of the grid in row major order.
 * [`Line`](include/nav_grid_iterators/line.h) iterates over a line, using either of the above algorithms, but the coordinates are constrained to the grid.
 * [`PolygonFill`](include/nav_grid_iterators/polygon_fill.h) iterates over all the cells whose centers fall within a `nav_2d_msgs::Polygon2D`
 * [`PolygonOutline`](include/nav_grid_iterators/polygon_outline.h) iterates over the outline of a `nav_2d_msgs::Polygon2D` using either of the two above line iterators.
 * [`CircleFill`](include/nav_grid_iterators/circle_fill.h) iterates over all the cells whose centers fall within a given circle, iterating in row major order.
 * [`Spiral`](include/nav_grid_iterators/spiral.h) iterates over the same cells as `CircleFill` but from the center of the circle outward.
 * [`CircleOutline`](include/nav_grid_iterators/circle_outline.h) iterates around the outline of a circle.

# Demo
A demonstration of all the general iterators can be seen by running `roslaunch nav_grid_iterators demo.launch` or by looking at [this video](demo/demo.mp4).
 * The purple iterator is `WholeGrid`
 * The bottom row, left to right, are
   * `SubGrid` (green)
   * `PolygonFill` (yellow)
   * `PolygonOutline+Bresenham` (blue)
   * `PolygonOutline+RayTrace` (blue)
 * In the middle are a `Line+Bresenham` (bottom) and a `Line+RayTrace` (top)
 * The top row, left to right, are
   * `CircleFill` (grey)
   * `Spiral` (green)
   * `CircleOutline` (cyan)
