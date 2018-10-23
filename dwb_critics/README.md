# dwb_critics
This package contains plugin implementations of `dwb_local_planner::TrajectoryCritic` sufficient to replicate the behavior of `dwa_local_planner`.

## Obstacle Avoidance
There are two options for critics that examine the contents of the costmap and reject trajectories that come close to obstacles.
 * `BaseObstacleCritic` assumes a circular robot, and therefore only needs to check one cell in the costmap for each pose in the trajectory (assuming the costmap is properly inflated).
 * `ObstacleFootprintCritic` uses the robot's footprint and checks all of the cells along the outline of the robot's footprint at each pose.

## Progress Toward the Goal along the Path
There are two critics which evaluate the robot's position at the end of the trajectory relative to the goal pose and the global plan.
 * `GoalDistCritic` estimates the distance from the last pose to the goal pose.
 * `PathDistCritic` estimates the distance from the last pose to the closest pose in the global plan.

## Alignment
There are also two critics for keeping the robot pointed in the right direction. They use a point on the front of the robot as a proxy to calculate which way the robot is pointed.
 * `GoalAlignCritic` estimates the distance from the front of the robot to the goal pose. This score will be higher if the robot is pointed away from the goal.
 * `PathAlignCritic` estimates the distance from the front of the robot to the closest point in the global plan. This score will be higher if the robot is pointed away from the global plan.

## Rotating to the Goal
There is a special case in the navigation when the robot reaches the correct XY position, but still needs to rotate to the proper yaw. The standard critics will not be useful in this case. Instead we have `RotateToGoalCritic` which operates in a couple different modes.
 * If the robot is not yet in the correct XY position, it has no effect on the trajectory score.
 * If the robot is at the correct XY position but still moving, this critic will score the trajecotries such that the robot slows down.
 * If the robot is at the correct XY position and stopped its forward motion, the critic will
    A) Disallow trajectories with forward motion
    B) Score trajectories (rotations) based on how close to the goal yaw they get.

## Other Critics
 * `OscillationCritic` detects oscillations by looking at the sign of the commanded motions. For example, if in a short window, the robot moves forward and then backward, it will penalize further trajectories that move forward again, as that is considered an oscillation.
 * `PreferForwardCritic` was implemented but not used by `DWA` and penalize trajectories that move backwards and/or turn too much.
 * `TwirlingCritic` penalizes trajectories with rotational velocities

