# Motivation (a.k.a. the trouble with move_base)

There are three main problems with move_base that we would like to address.

 1. Too many dang threads.
 2. Inability to change what happens when a new global or local plan is generated or fails.
 3. Lack of contextual information about what happened, internally and externally

## 1. Too Many Dang Threads

In [Ingo Lütkebohle's talk at ROSCon 2017](https://vimeo.com/236186712) ([Slides](https://roscon.ros.org/2017/presentations/ROSCon%202017%20Determinism%20in%20ROS.pdf)), there was a unacceptable time lag between when the sensor data noted a new obstacle and when the generated plans actually changed. The key reason is that all the parts of `move_base` run in their own independent threads.

 * Sensor data enters the costmap in the standard callback queue for topic subscriptions.
 * Each costmap is updated [in its own thread](https://github.com/ros-planning/navigation/blob/69aebf5e3757235173d21e30c503eec6bf2c4ab9/costmap_2d/src/costmap_2d_ros.cpp#L318)
 * Within move_base, there is [a thread for updating the global plan](https://github.com/ros-planning/navigation/blob/69aebf5e3757235173d21e30c503eec6bf2c4ab9/move_base/src/move_base.cpp#L86)
 * The local plan is generated [within the actionlib callback](https://github.com/ros-planning/navigation/blob/69aebf5e3757235173d21e30c503eec6bf2c4ab9/move_base/src/move_base.cpp#L754)

In addition to the time lag, this makes tracing the execution very complex, and can cause an unknown number of race conditions.

## 2. Inflexible move_base execution

The internal logic / "state machine" of move_base is confusing and obtuse. Also, you can't change it. There are several things that cause transitions among the implicit "states".

 * Receiving a new goal
 * Getting a new global plan
 * Failing to calculate a global plan
 * Failing to calculate a local plan
 * Finishing a recovery behavior

If either plan fails to be calculated, it triggers the first recovery behavior. When the recovery behavior is finished, it will attempt to calculate a new global plan, and subsequently a new local plan. If the same failure happens again, the next recovery behavior in the list of recovery behaviors is executed.

Recovery behaviors have the maddeningly generic interface of [runBehavior()](https://github.com/ros-planning/navigation/blob/69aebf5e3757235173d21e30c503eec6bf2c4ab9/nav_core/include/nav_core/recovery_behavior.h#L61) which makes no distinction between atomic actions like clearing the costmaps and complex, local-planner-like behaviors which can take an arbitrary amount of time.

The same list of recovery behaviors is iterated no matter what kind of failure happened.

Generally, there is not much value in the concept of recovery behaviors. They are a special case of navigation that can be useful, but are practically difficult to understand and implement.

Based on Ingo's talk, we should also be aware of when the costmap finishes its update.

## 3. Lack of Contextual Information

There's about a billion different places where it would be beneficial to get more information about what is going on (and going wrong) with move_base.

 * Why did global planning fail?
 * Why did local planning fail?
 * How much longer do we anticipate the navigation will take?
 * What recovery behavior is currently being executed?
 * How many times did the robot have to replan?
 * Heck, is the robot even able to move?

Some of this information is obscured because of the limitations of the `nav_core` interfaces; some is obscured by the structure of the `move_base` code itself.
