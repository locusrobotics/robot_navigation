# nav_core_adapter
This package contains adapters for using `nav_core` plugins as `nav_core2` plugins and vice versa (more or less). In general, the adaptation process involves
 * Converting between 2d and 3d datatypes.
 * Converting between returning false and throwing exceptions on failure.

We also provide an adapter for using a `costmap_2d::Costmap2DROS` as a plugin for the `nav_core2::Costmap` interface.

## Adapter Classes
 * Global Planner Adapters
   * `GlobalPlannerAdapter` is used for employing a `nav_core2` global planner interface (such as `dlux_global_planner`) as a `nav_core` plugin, like in `move_base`.
   * `GlobalPlannerAdapter2` is used for employing a `nav_core` global planner interface (such as `navfn`)
as a `nav_core2` plugin, like in `locomotor`.
 * Local Planner Adapter
   * `LocalPlannerAdapter` is used for employing a `nav_core2` local planner interface (such as `dwb_local_planner`) as a `nav_core` plugin, like in `move_base`. In addition to the standard adaptation steps listed above, the local planner adapter also uses the costmap to grab the global pose and subscribes to the odometry in order to get the current velocity.
   * There is no `LocalPlannerAdapter2`. The `nav_core2` interfaces use additional information (like velocity) in the `local_planner` interface than its `nav_core` counterpart. This information would be ignored by a `nav_core` planner, so no adapter is provided.
 * `CostmapAdapter` provides most of the functionality from `nav_core2::Costmap` and also provides a raw pointer to the `Costmap2DROS` object. It is not a perfect adaptation, because
   * `Costmap2DROS` starts its own update thread and updates on its own schedule, so calling `update()` does not actually cause the costmap to update. It does update some of the metadata though.
   * `setInfo` is not implemented.

## Parameter Setup
Let's look at a practical example of how to use `dwb_local_planner` in `move_base`.

If you were using `DWA` you would probably have parameters set up like this:
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
