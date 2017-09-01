# nav_core_adapter
This package contains adapters for using `nav_core` plugins as `nav_core2` plugins and vice versa (more or less).
The `nav_core2` interfaces are designed to require additional information in the `local_planner` interface than its
`nav_core` counterpart. Therefore, it is impossible to use a `nav_core` local planner like `base_local_planner` or
`dwa_local_planner` as `nav_core2` plugins in `locomotor`.

In general, the adaptation process involves
 * Converting between 2d and 3d datatypes.
 * Converting between returning false and throwing exceptions on failure.


## Global Planner Adapters
### global_planner_adapter
`global_planner_adapter` is used for employing a `nav_core2` global planner interface as a `nav_core` plugin,
like in `move_base`.

### global_planner_adapter2
`global_planner_adapter2` is used for employing a `nav_core` global planner interface (such as `navfn`)
as a `nav_core2` plugin, like in `locomotor`.

## Local Planner Adapter
### local_planner_adapter
`local_planner_adapter` is used for employing a `nav_core2` local planner interface (such as `dwb_local_planner`)
as a `nav_core` plugin, like in `move_base`.

In addition to the standard conversions listed above, the local planner adapter also uses the costmap to grab the
global pose and subscribes to the odometry in order to get the current velocity.

## Parameter Setup
Let's look at a practical example of how to use `dwb_local_planner` in `move_base`.

If you were using `dwa` you would probably have parameters set up like this:
```
base_local_planner: dwa_local_planner/DWALocalPlanner
DWALocalPlanner:
  acc_lim_x: 0.42
  ...
```
i.e. you specify
 * The name of the planner
 * A bunch of additional parameters within the planner's namespace

To use the adapter, you have to provide additional information.
```
base_local_planner: nav_core_adapter::LocalPlannerAdapter
LocalPlannerAdapter:
  planner_name: dwb_local_planner::DWBLocalPlanner
DWBLocalPlanner:
  acc_lim_x: 0.42
  ...
```
i.e.
 * The name of the planner now points at the adapter
 * The name of the actual planner loaded into the adapter's namespace
 * The planner's parameters still in the planner's namespace.

The process for the global planners is similar.
