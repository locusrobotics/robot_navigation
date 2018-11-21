# Locomotor

Locomotor is an extensible path planning coordination engine that replaces `move_base`. The goal is to provide a mechanism for controlling what happens when the global and local planners succeed and fail. It leverages ROS callback queues to coordinate multiple threads.

 * [Motivation](doc/Motivation.md)
 * [Primary Design](doc/PrimaryDesign.md)
 * [Example State Machines](doc/ExampleStateMachines.md)
 * [Error Handling](doc/ErrorHandling.md)
 * [Build on Top](doc/BuildingOnTop.md)
