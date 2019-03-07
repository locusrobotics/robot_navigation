
# Plugin Mux
`PluginMux` is an organizer for switching between multiple different plugins of the same type. It may be easiest to see how it operates through an example. Let's say we have multiple global planners we would like to use at different times, with only one being active at a given time.

This means we have multiple `pluginlib` plugins to load that extend the `BaseGlobalPlanner` interface from the `nav_core` package. We define multiple namespaces in the `global_planner_namespaces` and load each of them. Here's an example parameter config.

```
global_planner_namespaces:
  - boring_nav_fn
  - wacky_global_planner
boring_nav_fn:
  allow_unknown: true
  plugin_class: navfn/NavfnROS
wacky_global_planner:
  allow_unknown: false
  # default value commented out
  # plugin_class: global_planner/GlobalPlanner
```

The namespaces are arbitrary strings, and need not reflect the name of the planner. The package and class name for the plugin will be specified by the `plugin_class` parameter. By default, the first namespace will be loaded as the current plugin.

To advertise which plugin is active, we publish the namespace on a latched topic and set a parameter with the same name (`~/current_global_planner`). We can then switch among them with a `SetString` ROS service call, or the `usePlugin` C++ method.

This configuration is all created by creating a plugin mux with the following parameters (parameter names shown for convenience):
```
PluginMux(plugin_package = "nav_core",
          plugin_class = "BaseGlobalPlanner",
          parameter_name = "global_planner_namespaces",
          default_value = "global_planner/GlobalPlanner",
          ros_name = "current_global_planner",
          switch_service_name = "switch_global_planner");
```

If the parameter is not set or is an empty list, a namespace will be derived from the `default_value` and be loaded as the only plugin available.
```
global_planner_namespaces: []
GlobalPlanner:
  allow_unknown: true
```
