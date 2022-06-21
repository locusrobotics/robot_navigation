# nav_grid_server

`nav_grid_server` is a drop-in replacement for [`map_server`](http://wiki.ros.org/map_server) but with added flexibility. As such, it is a convenient tool for publishing maps (`nav_grid`s) from image files. It can also save image files from ROS topics.

## Map Server
#### Classic Usage
    rosrun nav_grid_server server path/to/map.yaml

This reads map meta-data from the yaml file and exposes the map in three different ways.
* Publishes a `nav_msgs/OccupancyGrid` message on the `/map` topic (along with `nav_msgs/MapMetaData` on the `/map_metadata` topic).
* Provides the `nav_msgs/GetMap` service with the name `/static_map`
* Publishes a `nav_2d_msgs/NavGridOfChars` message on the `/static_map` topic (which natively contains the metadata)

#### Direct Image Usage
    rosrun nav_grid_server server path/to/map.png

You can also just provide the image file as a command line argument and use the default metadata.


## Map Saver
    rosrun nav_grid_server saver

This retrieves a `nav_msgs/OccupancyGrid` message from the `/map` topic and saves as `map.png` and `map.yaml`.

## Images

Image data is managed using [`opencv2`](https://docs.opencv.org/4.x/d4/da8/group__imgcodecs.html#ga288b8b3da0892bd651fce07b3bbd3a56) and can use many formats, including (but not limited to)
 * bmp
 * jpg
 * png
 * webp
 * pgm

## Metadata
Classically, the map metadata has been contained in a yaml file stored alongside the image file, e.g.

    image: testmap.png
    resolution: 0.1
    origin: [0.0, 0.0, 0.0]
    occupied_thresh: 0.65
    free_thresh: 0.196
    negate: 0

In addition, `nav_grid_server` provides the ability to also specify this information as ROS parameters, or not at all. The order of precedence is
 * ROS Parameters (highest precedence)
 * YAML File
 * Default values (lowest precedence)

See [the `nav_grid` documentation](../nav_grid/README.md) for further definitions of the metadata.

### Server Params
The following parameters are able to be specified in either the yaml or ROS parameter server (with defaults in parentheses).

 * `resolution` (`0.05`) - Resolution of the map, meters / pixel
 * `negate` (`false`) - Whether the image intensity should be negated
 * `occupied_thresh` (`0.65`) - Pixels with intensity greater than this threshold (scaled 0-1)  are considered occupied.
 * `free_thresh`  (`0.196`) - Pixels with intensity less than this threshold (scaled 0-1)  are considered free.
 * `mode` (`"trinary"`) - Default cost interpretation. See [the documentation here](../nav_grid_pub_sub/doc/CostInterpretation.md).
 * `origin_x`/`origin_y` (`0.0` / `0.0`) Origin offset in meters. The yaw is assumed to be zero.
    **Note:**: For backwards compatibility, the origin in the yaml file is specified with the name `origin` as an array of doubles, the first to values of which are used for `origin_x` and `origin_y`.

If using a yaml file, `image` must also be specified as the path (absolute or relative) to the image file.

Additionally, you can set the following as ROS Parameters:
 * `frame_id` (`"map"`) - TF frame of the map
 * `occupancy_grid_topic` (`"map"`) - Topic on which the `nav_msgs/OccupancyGrid` is published.
 * `nav_grid_topic` (`"static_map"`) - Topic on which the `nav_2d_msgs/NavGridOfChars` is published.

### Saver Params
 * `topic` (`"map"`)- Topic to subscribe to
 * `nav_grid` (`false`) - If true, the topic is expected to be of type `nav_2d_msgs/NavGridOfChars`. Otherwise, `nav_msgs/OccupancyGrid`
 * `once` (`true`) - If true, only save the first map. Otherwise, save the map repeatedly (useful for mapping).
 * `trinary_output` (`true`) - If true, uses the trinary cost interpretation. Otherwise, the values are scaled [0, 100]
 * Filename Parameters
    * `map_extension` (`"png"`)
    * `map_prefix` (`"map"`)
    * `output_directory` (".")
    * `write_unstamped` (`true`) If true, will write to `output_directory/map_prefix.map_extension`
    * `write_stamped` (`false`) If true, will write to `output_directory/map_prefix-timestamp.map_extension`
