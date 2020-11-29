/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <nav_2d_utils/conversions.h>
#include <vector>
#include <string>

namespace nav_2d_utils
{

geometry_msgs::Twist twist2Dto3D(const nav_2d_msgs::Twist2D& cmd_vel_2d)
{
  geometry_msgs::Twist cmd_vel;
  cmd_vel.linear.x = cmd_vel_2d.x;
  cmd_vel.linear.y = cmd_vel_2d.y;
  cmd_vel.angular.z = cmd_vel_2d.theta;
  return cmd_vel;
}


nav_2d_msgs::Twist2D twist3Dto2D(const geometry_msgs::Twist& cmd_vel)
{
  nav_2d_msgs::Twist2D cmd_vel_2d;
  cmd_vel_2d.x = cmd_vel.linear.x;
  cmd_vel_2d.y = cmd_vel.linear.y;
  cmd_vel_2d.theta = cmd_vel.angular.z;
  return cmd_vel_2d;
}

nav_2d_msgs::Point2D pointToPoint2D(const geometry_msgs::Point& point)
{
  nav_2d_msgs::Point2D output;
  output.x = point.x;
  output.y = point.y;
  return output;
}

nav_2d_msgs::Point2D pointToPoint2D(const geometry_msgs::Point32& point)
{
  nav_2d_msgs::Point2D output;
  output.x = point.x;
  output.y = point.y;
  return output;
}

geometry_msgs::Point pointToPoint3D(const nav_2d_msgs::Point2D& point)
{
  geometry_msgs::Point output;
  output.x = point.x;
  output.y = point.y;
  return output;
}

geometry_msgs::Point32 pointToPoint32(const nav_2d_msgs::Point2D& point)
{
  geometry_msgs::Point32 output;
  output.x = point.x;
  output.y = point.y;
  return output;
}

nav_2d_msgs::Pose2DStamped stampedPoseToPose2D(const tf::Stamped<tf::Pose>& pose)
{
  nav_2d_msgs::Pose2DStamped pose2d;
  pose2d.header.stamp = pose.stamp_;
  pose2d.header.frame_id = pose.frame_id_;
  pose2d.pose.x = pose.getOrigin().getX();
  pose2d.pose.y = pose.getOrigin().getY();
  pose2d.pose.theta = tf::getYaw(pose.getRotation());
  return pose2d;
}

nav_2d_msgs::Pose2DStamped poseStampedToPose2D(const geometry_msgs::PoseStamped& pose)
{
  nav_2d_msgs::Pose2DStamped pose2d;
  pose2d.header = pose.header;
  pose2d.pose.x = pose.pose.position.x;
  pose2d.pose.y = pose.pose.position.y;
  pose2d.pose.theta = tf::getYaw(pose.pose.orientation);
  return pose2d;
}

geometry_msgs::Pose pose2DToPose(const geometry_msgs::Pose2D& pose2d)
{
  geometry_msgs::Pose pose;
  pose.position.x = pose2d.x;
  pose.position.y = pose2d.y;
  pose.orientation = tf::createQuaternionMsgFromYaw(pose2d.theta);
  return pose;
}

geometry_msgs::PoseStamped pose2DToPoseStamped(const nav_2d_msgs::Pose2DStamped& pose2d)
{
  geometry_msgs::PoseStamped pose;
  pose.header = pose2d.header;
  pose.pose = pose2DToPose(pose2d.pose);
  return pose;
}

geometry_msgs::PoseStamped pose2DToPoseStamped(const geometry_msgs::Pose2D& pose2d,
                                               const std::string& frame, const ros::Time& stamp)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = frame;
  pose.header.stamp = stamp;
  pose.pose.position.x = pose2d.x;
  pose.pose.position.y = pose2d.y;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose2d.theta);
  return pose;
}

nav_2d_msgs::Path2D pathToPath(const nav_msgs::Path& path)
{
  nav_2d_msgs::Path2D path2d;
  path2d.header = path.header;
  nav_2d_msgs::Pose2DStamped stamped_2d;
  path2d.poses.resize(path.poses.size());
  for (unsigned int i = 0; i < path.poses.size(); i++)
  {
    stamped_2d = poseStampedToPose2D(path.poses[i]);
    path2d.poses[i] = stamped_2d.pose;
  }
  return path2d;
}

nav_msgs::Path posesToPath(const std::vector<geometry_msgs::PoseStamped>& poses)
{
  nav_msgs::Path path;
  if (poses.empty())
    return path;

  path.poses.resize(poses.size());
  path.header.frame_id = poses[0].header.frame_id;
  path.header.stamp = poses[0].header.stamp;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    path.poses[i] = poses[i];
  }
  return path;
}

nav_2d_msgs::Path2D posesToPath2D(const std::vector<geometry_msgs::PoseStamped>& poses)
{
  nav_2d_msgs::Path2D path;
  if (poses.empty())
    return path;

  nav_2d_msgs::Pose2DStamped pose;
  path.poses.resize(poses.size());
  path.header.frame_id = poses[0].header.frame_id;
  path.header.stamp = poses[0].header.stamp;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    pose = poseStampedToPose2D(poses[i]);
    path.poses[i] = pose.pose;
  }
  return path;
}


nav_msgs::Path poses2DToPath(const std::vector<geometry_msgs::Pose2D>& poses,
                             const std::string& frame, const ros::Time& stamp)
{
  nav_msgs::Path path;
  path.poses.resize(poses.size());
  path.header.frame_id = frame;
  path.header.stamp = stamp;
  for (unsigned int i = 0; i < poses.size(); i++)
  {
    path.poses[i] = pose2DToPoseStamped(poses[i], frame, stamp);
  }
  return path;
}

nav_msgs::Path pathToPath(const nav_2d_msgs::Path2D& path2d)
{
  nav_msgs::Path path;
  path.header = path2d.header;
  path.poses.resize(path2d.poses.size());
  for (unsigned int i = 0; i < path.poses.size(); i++)
  {
    path.poses[i].header = path2d.header;
    path.poses[i].pose = pose2DToPose(path2d.poses[i]);
  }
  return path;
}

geometry_msgs::Polygon polygon2Dto3D(const nav_2d_msgs::Polygon2D& polygon_2d)
{
  geometry_msgs::Polygon polygon;
  polygon.points.reserve(polygon_2d.points.size());
  for (const auto& pt : polygon_2d.points)
  {
    polygon.points.push_back(pointToPoint32(pt));
  }
  return polygon;
}

nav_2d_msgs::Polygon2D polygon3Dto2D(const geometry_msgs::Polygon& polygon_3d)
{
  nav_2d_msgs::Polygon2D polygon;
  polygon.points.reserve(polygon_3d.points.size());
  for (const auto& pt : polygon_3d.points)
  {
    polygon.points.push_back(pointToPoint2D(pt));
  }
  return polygon;
}

geometry_msgs::PolygonStamped polygon2Dto3D(const nav_2d_msgs::Polygon2DStamped& polygon_2d)
{
  geometry_msgs::PolygonStamped polygon;
  polygon.header = polygon_2d.header;
  polygon.polygon = polygon2Dto3D(polygon_2d.polygon);
  return polygon;
}

nav_2d_msgs::Polygon2DStamped polygon3Dto2D(const geometry_msgs::PolygonStamped& polygon_3d)
{
  nav_2d_msgs::Polygon2DStamped polygon;
  polygon.header = polygon_3d.header;
  polygon.polygon = polygon3Dto2D(polygon_3d.polygon);
  return polygon;
}

nav_2d_msgs::NavGridInfo toMsg(const nav_grid::NavGridInfo& info)
{
  nav_2d_msgs::NavGridInfo msg;
  msg.width = info.width;
  msg.height = info.height;
  msg.resolution = info.resolution;
  msg.frame_id = info.frame_id;
  msg.origin_x = info.origin_x;
  msg.origin_y = info.origin_y;
  return msg;
}

nav_grid::NavGridInfo fromMsg(const nav_2d_msgs::NavGridInfo& msg)
{
  nav_grid::NavGridInfo info;
  info.width = msg.width;
  info.height = msg.height;
  info.resolution = msg.resolution;
  info.frame_id = msg.frame_id;
  info.origin_x = msg.origin_x;
  info.origin_y = msg.origin_y;
  return info;
}

nav_grid::NavGridInfo infoToInfo(const nav_msgs::MapMetaData& metadata, const std::string& frame)
{
  nav_grid::NavGridInfo info;
  info.frame_id = frame;
  info.resolution = metadata.resolution;
  info.width = metadata.width;
  info.height = metadata.height;
  info.origin_x = metadata.origin.position.x;
  info.origin_y = metadata.origin.position.y;
  if (std::abs(tf::getYaw(metadata.origin.orientation)) > 1e-5)
  {
    ROS_WARN_NAMED("nav_2d_utils",
                   "Conversion from MapMetaData to NavGridInfo encountered a non-zero rotation. Ignoring.");
  }
  return info;
}

geometry_msgs::Pose getOrigin3D(const nav_grid::NavGridInfo& info)
{
  geometry_msgs::Pose origin;
  origin.position.x = info.origin_x;
  origin.position.y = info.origin_y;
  origin.orientation.w = 1.0;
  return origin;
}

geometry_msgs::Pose2D getOrigin2D(const nav_grid::NavGridInfo& info)
{
  geometry_msgs::Pose2D origin;
  origin.x = info.origin_x;
  origin.y = info.origin_y;
  return origin;
}

nav_msgs::MapMetaData infoToInfo(const nav_grid::NavGridInfo & info)
{
  nav_msgs::MapMetaData metadata;
  metadata.resolution = info.resolution;
  metadata.width = info.width;
  metadata.height = info.height;
  metadata.origin = getOrigin3D(info);
  return metadata;
}

nav_2d_msgs::UIntBounds toMsg(const nav_core2::UIntBounds& bounds)
{
  nav_2d_msgs::UIntBounds msg;
  msg.min_x = bounds.getMinX();
  msg.min_y = bounds.getMinY();
  msg.max_x = bounds.getMaxX();
  msg.max_y = bounds.getMaxY();
  return msg;
}

nav_core2::UIntBounds fromMsg(const nav_2d_msgs::UIntBounds& msg)
{
  return nav_core2::UIntBounds(msg.min_x, msg.min_y, msg.max_x, msg.max_y);
}


}  // namespace nav_2d_utils
