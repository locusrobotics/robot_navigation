# Multiple Planners
Locomotor can load any number of (local and global) planners into different namespaces. However, only one is marked as active at any particular time. This allows for easy switching between planners, done using the string namespace.

One could easily imagine handling different types of Goals by first setting which planners to use, i.e. if you receive a Docking goal, you could set the local planner to the docking local planner and then attempt to dock.
