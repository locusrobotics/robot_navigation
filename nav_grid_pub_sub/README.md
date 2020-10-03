# nav_grid_pub_sub

The [`nav_2d_msgs`](../nav_2d_msgs/README.md) package provides two message types for publishing / receiving the data for an entire  `nav_grid::NavGrid`.
 * `nav_2d_msgs::NavGridOfChars` for `nav_grid::NavGrid<unsigned char>` and
 * `nav_2d_msgs::NavGridOfDoubles` for `nav_grid::NavGrid<double>`

There are also two message types for publishing a rectangular sub-portion of the `NavGrid`.
 * `nav_2d_msgs::NavGridOfCharsUpdate`
 * `nav_2d_msgs::NavGridOfDoublesUpdate`

These messages are analagous to `nav_msgs::OccupancyGrid` and `map_msgs::OccupancyGridUpdate`. However, `OccupancyGrid` does not support floating point and its char values are scaled to [0, 100]. This class also provides support for publishing / receiving `OccupancyGrid` and `OccupancyGridUpdate` messages, but the data needs to be scaled, which is discussed [here.](doc/CostInterpretation.md)

This package provides mechanisms for publishing and receiving both the whole grid message and the update message in a unified manner.

## Publishing
The [publisher](include/nav_grid_pub_sub/nav_grid_publisher.h) requires a `nav_grid::NavGrid&` at construction, and is initialized with a `NodeHandle`.

By default, it will create publishers for
 * `NavGridOfX` messages (depending on the datatype of the NavGrid)
 * `OccupancyGrid` messages
 * The two related `Update` messages
 * A `PolygonStamped` message for visualizing the update area.

You can rename the topics these message are published on by passing different strings into the initialize method, or turn the publishing for certain messages off entirely by passing in the empty string.

The whole grid is published by calling `publish()` using data from the `NavGrid`. An update is published with `publish(bounds)`.

You can also limit how often the full `OccupancyGrid`/`NavGridOfChars` message is published with the `full_publish_cycle` parameter.
 * If the full_publish_cycle is 0, which is the default, the full grid will be published every time you call publish.
 * If the full_publish_cycle is negative, you avoid publishing all together.
 * Otherwise, the full grid will only be published when publish is called if `full_publish_cycle` time has passed since the last full grid publish.
 * Note that full grids may also be published when attempting to publish an update but the grid info has changed

You can control how often updates are published with similar logic and the update_publish_cycle argument. If the
* update_publish_cycle is positive, the Bounds from successive calls will be merged so the resulting update will
* cover the superset of all the bounds.

## Subscribing
The [subscribers](include/nav_grid_pub_sub/nav_grid_subscriber.h) also require a `nav_grid::NavGrid&` at construction.

However, each will only subscribe to `OccupancyGrid` OR `NavGridOfX` (and their updates), not both.
 * The standard way to subscribe to `OccupancyGrid` messages is with `nav_grid_pub_sub::NavGridSubscriber` initialized with `nav_grid=False` which will write into a `nav_grid::NavGrid<unsigned char>`. You could also theoretically use `nav_grid_pub_sub::NavGridOfDoublesSubscriber` to write to `nav_grid::NavGrid<double>`, but you would likely need to set your own interpretation function for that.
 * On the other hand, if you set `nav_grid=True` it will subscribe to `NavGridofChars` or `NavGridOfDoubles` depending on if you use `nav_grid_pub_sub::NavGridSubscriber` or `nav_grid_pub_sub::NavGridOfDoublesSubscriber`.

The data is automatically applied to the `NavGrid`, and new data will trigger a callback function that is passed in as a parameter so that other classes can be notified of how much of the costmap has changed.
