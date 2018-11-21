# locomotor_msgs

This class provides the `NavigateToPose` action for use with `actionlib+locomotor`. The idea is to provide more useful feedback than [MoveBase.action](http://docs.ros.org/api/move_base_msgs/html/action/MoveBase.html).

Developers are encouraged to define their own actions that have statistics/constraints relative to their own domains.

## Progress City
The `percent_complete` field in the `NavigateToPoseFeedback` is provided as a convenience, and is equivalent to `distance_traveled / (distance_traveled + estimated_distance_remaining)`. Note that with the current implementation, it is possible for `percent_complete` to go down if the `estimated_distance_remaining` goes up, which can happen when the global plan is blocked and a new longer global plan is found instead.

## I Want Results!
When the action finishes, the action client can figure out whether the action succeeded by checking if the `state` is `SUCCEEDED` or `ABORTED`. The `SimpleActionServer/Client` also provides ["an optional text message"](https://github.com/ros/actionlib/blob/9210d811d105eabe72ff4741dece801f36e9064a/include/actionlib/server/simple_action_server.h#L165). While these two fields may provide sufficient information about the final state of the action, we provide one other method for providing final feedback.

The `NavigateToPoseResult` contains a `ResultCode` message, which contains the integer field `result_code` and some enums for possible values of `result_code`.


These Exception Codes are Based on

  1. Which of the four components (Global/Local Costmap, Global/Local Planner) the error originates from
  2. The particular exception thrown, based on the exception hierarchy defined in `nav_core2/exceptions.h`

These particular result codes are used in conjunction with the `StateMachine`s defined in the `locomotor` package, but other user-implemented `StateMachine`s could use different custom-error codes, or build onto the existing error codes, as some space is left between the existing enum values.
