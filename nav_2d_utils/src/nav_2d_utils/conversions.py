from geometry_msgs.msg import Pose, Pose2D, Quaternion, Point
from nav_2d_msgs.msg import Twist2D, Path2D, Pose2DStamped, Point2D
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped

try:
    from tf.transformations import euler_from_quaternion, quaternion_from_euler

    def get_yaw(orientation):
        rpy = euler_from_quaternion((orientation.x, orientation.y, orientation.z, orientation.w))
        return rpy[2]

    def from_yaw(yaw):
        q = Quaternion()
        q.x, q.y, q.z, q.w = quaternion_from_euler(0, 0, yaw)
        return q
except ImportError:
    from math import sin, cos, atan2
    # From https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    def from_yaw(yaw):
        q = Quaternion()
        q.z = sin(yaw * 0.5)
        q.w = cos(yaw * 0.5)
        return q

    def get_yaw(q):
        siny_cosp = +2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return atan2(siny_cosp, cosy_cosp)


def twist2Dto3D(cmd_vel_2d):
    cmd_vel = Twist()
    cmd_vel.linear.x = cmd_vel_2d.x
    cmd_vel.linear.y = cmd_vel_2d.y
    cmd_vel.angular.z = cmd_vel_2d.theta
    return cmd_vel


def twist3Dto2D(cmd_vel):
    cmd_vel_2d = Twist2D()
    cmd_vel_2d.x = cmd_vel.linear.x
    cmd_vel_2d.y = cmd_vel.linear.y
    cmd_vel_2d.theta = cmd_vel.angular.z
    return cmd_vel_2d


def pointToPoint3D(point_2d):
    point = Point()
    point.x = point_2d.x
    point.y = point_2d.y
    return point


def pointToPoint2D(point):
    point_2d = Point2D()
    point_2d.x = point.x
    point_2d.y = point.y
    return point_2d


def poseToPose2D(pose):
    pose2d = Pose2D()
    pose2d.x = pose.position.x
    pose2d.y = pose.position.y
    pose2d.theta = get_yaw(pose.orientation)
    return pose2d


def poseStampedToPose2DStamped(pose):
    pose2d = Pose2DStamped()
    pose2d.header = pose.header
    pose2d.pose = poseToPose2D(pose.pose)
    return pose2d


def poseToPose2DStamped(pose, frame, stamp):
    pose2d = Pose2DStamped()
    pose2d.header.frame_id = frame
    pose2d.header.stamp = stamp
    pose2d.pose = poseToPose2D(pose)
    return pose2d


def pose2DToPose(pose2d):
    pose = Pose()
    pose.position.x = pose2d.x
    pose.position.y = pose2d.y
    pose.orientation = from_yaw(pose2d.theta)
    return pose


def pose2DStampedToPoseStamped(pose2d):
    pose = PoseStamped()
    pose.header = pose2d.header
    pose.pose = pose2DToPose(pose2d.pose)
    return pose


def pose2DToPoseStamped(pose2d, frame, stamp):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = stamp
    pose.pose.position.x = pose2d.x
    pose.pose.position.y = pose2d.y
    pose.pose.orientation = from_yaw(pose2d.theta)
    return pose


def pathToPath2D(path):
    path2d = Path2D()
    if len(path.poses) == 0:
        return path
    path2d.header.frame_id = path.poses[0].header.frame_id
    path2d.header.stamp = path.poses[0].header.stamp
    for pose in path.poses:
        stamped = poseStampedToPose2DStamped(pose)
        path2d.poses.append(stamped.pose)
    return path2d


def path2DToPath(path2d):
    path = Path()
    path.header = path2d.header
    for pose2d in path2d.poses:
        pose = PoseStamped()
        pose.header = path2d.header
        pose.pose = pose2DToPose(pose2d)
        path.poses.append(pose)
    return path
